#include <fetch_grasp_suggestion/executor.h>

using std::string;
using std::stringstream;
using std::vector;

// Default scaling factor for arm motion velocities
const float MAX_VELOCITY_SCALING_FACTOR = 0.3;

// Default max velocity for velocities sent to the /cmd_vel topic
const float CARTESIAN_MOVE_VELOCITY = 0.7;

Executor::Executor() :
    pnh_("~"),
    tf_listener_(tf_buffer_),
    gripper_client_("gripper_controller/gripper_action"),
    execute_grasp_server_(pnh_, "execute_grasp", boost::bind(&Executor::executeGrasp, this, _1), false),
    prepare_robot_server_(pnh_, "prepare_robot", boost::bind(&Executor::prepareRobot, this, _1), false),
    drop_pose_server_(pnh_, "drop_position", boost::bind(&Executor::dropPosition, this, _1), false),
    preset_pose_server_(pnh_, "preset_position", boost::bind(&Executor::presetPosition, this, _1), false)
{
  gripper_names_.push_back("gripper_link");
  gripper_names_.push_back("l_gripper_finger_link");
  gripper_names_.push_back("r_gripper_finger_link");

  ready_pose_.name.push_back("shoulder_pan_joint");
  ready_pose_.name.push_back("shoulder_lift_joint");
  ready_pose_.name.push_back("upperarm_roll_joint");
  ready_pose_.name.push_back("elbow_flex_joint");
  ready_pose_.name.push_back("forearm_roll_joint");
  ready_pose_.name.push_back("wrist_flex_joint");
  ready_pose_.name.push_back("wrist_roll_joint");

  ready_pose_.position.push_back(1.5515);
  ready_pose_.position.push_back(-0.9132);
  ready_pose_.position.push_back(0.9883);
  ready_pose_.position.push_back(-1.1887);
  ready_pose_.position.push_back(0.1412);
  ready_pose_.position.push_back(-1.0679);
  ready_pose_.position.push_back(0.0);

  drop_pose_.name = ready_pose_.name;
  drop_pose_.position.push_back(1.2958);
  drop_pose_.position.push_back(-0.2712);
  drop_pose_.position.push_back(1.3963);
  drop_pose_.position.push_back(-0.8001);
  drop_pose_.position.push_back(1.5246);
  drop_pose_.position.push_back(-1.1917);
  drop_pose_.position.push_back(0.0);

  arm_group_ = new moveit::planning_interface::MoveGroupInterface("arm");
  arm_group_->startStateMonitor();
  arm_group_->setMaxVelocityScalingFactor(MAX_VELOCITY_SCALING_FACTOR);

  test1_ = pnh_.advertise<geometry_msgs::PoseStamped>("pose1", 1);
  test2_ = pnh_.advertise<geometry_msgs::PoseStamped>("pose2", 1);

  cartesian_pub_ = n_.advertise<geometry_msgs::TwistStamped>("/arm_controller/cartesian_twist/command", 10);

  compute_cartesian_path_client_ = n_.serviceClient<moveit_msgs::GetCartesianPath>("/compute_cartesian_path");
  detach_objects_client_ = n_.serviceClient<std_srvs::Empty>("/collision_scene_manager/detach_objects");
  toggle_gripper_collisions_client_ = n_.serviceClient<manipulation_actions::ToggleGripperCollisions>
      ("/collision_scene_manager/toggle_gripper_collisions");
  attach_closest_object_client_ = n_.serviceClient<std_srvs::Empty>("/collision_scene_manager/attach_closest_object");
  attach_arbitrary_object_client_ = n_.serviceClient<manipulation_actions::AttachArbitraryObject>
      ("/collision_scene_manager/attach_arbitrary_object");
  add_object_server_ = pnh_.advertiseService("add_object", &Executor::addObject, this);
  clear_objects_server_ = pnh_.advertiseService("clear_objects", &Executor::clearObjects, this);
  drop_object_server_ = pnh_.advertiseService("drop_object", &Executor::dropObjectCallback, this);

  execute_grasp_server_.start();
  prepare_robot_server_.start();
  drop_pose_server_.start();
  preset_pose_server_.start();
}

void Executor::prepareRobot(const fetch_grasp_suggestion::PresetMoveGoalConstPtr &goal)
{
  fetch_grasp_suggestion::PresetMoveResult result;

  ROS_INFO("Preparing robot for exciting grasp action...");

  arm_group_->setPlannerId("arm[RRTConnectkConfigDefault]");
  arm_group_->setPlanningTime(7.0);
  arm_group_->setStartStateToCurrentState();
  arm_group_->setJointValueTarget(ready_pose_);

  // Plan and execute while checking for preempts
  if (prepare_robot_server_.isPreemptRequested())
  {
    ROS_INFO("Preempted while moving to ready pose.");
    result.success = false;
    prepare_robot_server_.setPreempted(result);
  }
  result.error_code = arm_group_->move().val;
  if (result.error_code == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    ROS_INFO("Preempted from MoveIt! while moving to ready pose. Aborting");
    result.success = false;
    prepare_robot_server_.setAborted(result);
  }
  else if (result.error_code == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("Robot prepared!");
    result.success = true;
    prepare_robot_server_.setSucceeded(result);
  }
  else
  {
    ROS_INFO("Failed to move to ready pose.");
    result.success = false;
    prepare_robot_server_.setAborted(result);
  }
}

void Executor::dropPosition(const fetch_grasp_suggestion::PresetMoveGoalConstPtr &goal)
{
  fetch_grasp_suggestion::PresetMoveResult result;

  ROS_INFO("Preparing robot for object dropoff...");

  arm_group_->setPlannerId("arm[RRTConnectkConfigDefault]");
  arm_group_->setPlanningTime(7.0);
  arm_group_->setStartStateToCurrentState();
  arm_group_->setJointValueTarget(drop_pose_);

  if (drop_pose_server_.isPreemptRequested())
  {
    ROS_INFO("Preempted while moving to ready pose.");
    result.success = false;
    prepare_robot_server_.setPreempted(result);
  }
  result.error_code = arm_group_->move().val;
  if (result.error_code == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("Ready to drop object!");
    result.success = true;
    drop_pose_server_.setSucceeded(result);
  }
  else if (result.error_code == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    ROS_INFO("Preempted from MoveIt! while moving to dropoff. Aborting");
    result.success = false;
    drop_pose_server_.setAborted(result);
  }
  else
  {
    ROS_INFO("Failed to move to drop pose.");
    result.success = false;
    drop_pose_server_.setAborted(result);
  }
}

void Executor::presetPosition(const fetch_grasp_suggestion::PresetJointsMoveGoalConstPtr &goal)
{
  fetch_grasp_suggestion::PresetJointsMoveResult result;

  ROS_INFO("Preparing robot to preset pose...");

  sensor_msgs::JointState preset_pose;
  preset_pose.name = goal->name;
  preset_pose.position = goal->position;

  arm_group_->setPlannerId("arm[RRTConnectkConfigDefault]");
  arm_group_->setPlanningTime(7.0);
  arm_group_->setStartStateToCurrentState();
  arm_group_->setJointValueTarget(preset_pose);
  if (goal->max_velocity_scaling_factor > 0)
  {
    arm_group_->setMaxVelocityScalingFactor(goal->max_velocity_scaling_factor);
  }

  if (preset_pose_server_.isPreemptRequested())
  {
    ROS_INFO("Preempted while moving to preset pose.");
    result.success = false;
    preset_pose_server_.setPreempted(result);
    arm_group_->setMaxVelocityScalingFactor(MAX_VELOCITY_SCALING_FACTOR);
    return;
  }
  result.error_code = arm_group_->move().val;
  if (result.error_code != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("Failed to move to preset pose.");
    result.success = false;
    preset_pose_server_.setAborted(result);
  }
  else
  {
    ROS_INFO("Arm successfully reached the preset pose.");
    result.success = true;
    preset_pose_server_.setSucceeded(result);
  }

  // Reset the scaling factor
  arm_group_->setMaxVelocityScalingFactor(MAX_VELOCITY_SCALING_FACTOR);
}

void Executor::executeGrasp(const fetch_grasp_suggestion::ExecuteGraspGoalConstPtr &goal)
{
  boost::mutex::scoped_lock lock(object_mutex_);

  fetch_grasp_suggestion::ExecuteGraspResult result;
  
  string group_reference_frame = arm_group_->getPoseReferenceFrame();

  //transform pose to reference group coordinate frame (fixes an annoying bug that spams warnings to the terminal...)
  geometry_msgs::PoseStamped grasp_pose;
  if (goal->grasp_pose.header.frame_id != group_reference_frame)
  {
    grasp_pose.header.stamp = ros::Time(0);
    grasp_pose.header.frame_id = group_reference_frame;

    geometry_msgs::TransformStamped group_to_grasp_transform = tf_buffer_.lookupTransform(group_reference_frame,
        goal->grasp_pose.header.frame_id, ros::Time(0), ros::Duration(1.0));
    tf2::doTransform(goal->grasp_pose, grasp_pose, group_to_grasp_transform);

//    tf1_listener_.transformPose(group_reference_frame, goal->grasp_pose, grasp_pose);
  }
  else
  {
    grasp_pose = goal->grasp_pose;
  }

  ros::Time current_time = ros::Time::now();
  geometry_msgs::TransformStamped grasp_transform;
  grasp_transform.child_frame_id = "grasp_frame";
  grasp_transform.header.frame_id = grasp_pose.header.frame_id;
  grasp_transform.header.stamp = current_time;
  grasp_transform.transform.translation.x = grasp_pose.pose.position.x;
  grasp_transform.transform.translation.y = grasp_pose.pose.position.y;
  grasp_transform.transform.translation.z = grasp_pose.pose.position.z;
  grasp_transform.transform.rotation = grasp_pose.pose.orientation;
  tf_broadcaster_.sendTransform(grasp_transform);

  geometry_msgs::TransformStamped wrist_transform = tf_buffer_.lookupTransform("gripper_link", "wrist_roll_link",
                                                                               ros::Time(0), ros::Duration(3.0));

  geometry_msgs::PoseStamped wrist_grasp_pose;
  wrist_grasp_pose.header.frame_id = "grasp_frame";
  wrist_grasp_pose.pose.position.x = wrist_transform.transform.translation.x;
  wrist_grasp_pose.pose.position.y = wrist_transform.transform.translation.y;
  wrist_grasp_pose.pose.position.z = wrist_transform.transform.translation.z;
  wrist_grasp_pose.pose.orientation = wrist_transform.transform.rotation;

  //calculate grasp pose for wrist link in frame from goal grasp pose
  geometry_msgs::TransformStamped from_grasp_transform =
      tf_buffer_.lookupTransform(group_reference_frame, "grasp_frame", current_time, ros::Duration(3.0));
  geometry_msgs::PoseStamped transformed_grasp_pose;
  transformed_grasp_pose.header.frame_id = group_reference_frame;
  tf2::doTransform(wrist_grasp_pose, transformed_grasp_pose, from_grasp_transform);

  //calculate approach pose for wrist link in frame from goal grasp pose
  wrist_grasp_pose.pose.position.x -= 0.12;
  geometry_msgs::PoseStamped transformed_approach_pose;
  transformed_approach_pose.header.frame_id = group_reference_frame;
  tf2::doTransform(wrist_grasp_pose, transformed_approach_pose, from_grasp_transform);

  test1_.publish(transformed_approach_pose);

  //plan and move to approach pose
  arm_group_->setPlannerId("arm[RRTConnectkConfigDefault]");
  arm_group_->setPlanningTime(1.5);
  arm_group_->setStartStateToCurrentState();
  arm_group_->setPoseTarget(transformed_approach_pose, "wrist_roll_link");
  if (goal->max_velocity_scaling_factor > 0)
  {
    arm_group_->setMaxVelocityScalingFactor(goal->max_velocity_scaling_factor);
  }

  //send the goal, and also check for preempts
  if (execute_grasp_server_.isPreemptRequested())
  {
    ROS_INFO("Preempted while moving to approach pose.");
    result.error_code = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::APPROACH;
    execute_grasp_server_.setPreempted(result);
    return;
  }
  result.error_code = arm_group_->move().val;
  if (result.error_code == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    ROS_INFO("Preempted from MoveIt! while moving to approach pose. Aborting");
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::APPROACH;
    execute_grasp_server_.setAborted(result);
    return;
  }
  else if (result.error_code != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("Failed to move to approach pose.");
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::APPROACH;
    execute_grasp_server_.setAborted(result);
    return;
  }

  //open gripper
  control_msgs::GripperCommandGoal gripper_goal;
  gripper_goal.command.position = 0.1;
  gripper_goal.command.max_effort = 200;
  gripper_client_.sendGoal(gripper_goal);
  gripper_client_.waitForResult(ros::Duration(5.0));

  //disable collision between gripper links and object
  toggleGripperCollisions(
      goal->index >= 0
        ? manipulation_actions::ToggleGripperCollisions::Request::ALL_OBJECTS_NAME
        : manipulation_actions::ToggleGripperCollisions::Request::OCTOMAP_NAME,
      true
  );

  //linear plan to grasp pose
  test1_.publish(transformed_grasp_pose);

  // Try planning and replanning a few times before failing
  int max_planning_attempts = 3;
  moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
  for (int num_attempts=0; num_attempts < max_planning_attempts; num_attempts++)
  {
    ROS_INFO("Attempting to plan path to grasp. Attempt: %d/%d",
             num_attempts + 1, max_planning_attempts);

    // calculate short-distance plan to final grasp pose
    arm_group_->setPlannerId("arm[RRTConnectkConfigDefault]");
    arm_group_->setPlanningTime(1.5);
    arm_group_->setStartStateToCurrentState();
    arm_group_->setPoseTarget(transformed_grasp_pose, "wrist_roll_link");
    if (goal->max_velocity_scaling_factor > 0)
    {
      arm_group_->setMaxVelocityScalingFactor(goal->max_velocity_scaling_factor);
    }

    moveit::planning_interface::MoveItErrorCode plan_result = arm_group_->plan(grasp_plan);
    if (execute_grasp_server_.isPreemptRequested())
    {
      toggleGripperCollisions(
          goal->index >= 0
          ? manipulation_actions::ToggleGripperCollisions::Request::ALL_OBJECTS_NAME
          : manipulation_actions::ToggleGripperCollisions::Request::OCTOMAP_NAME,
          false
      );

      ROS_INFO("Preempted while planning grasp");
      result.error_code = moveit_msgs::MoveItErrorCodes::PREEMPTED;
      result.success = false;
      result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::GRASP_PLAN;
      execute_grasp_server_.setPreempted(result);
      return;
    }
    else if (plan_result.val != moveit::planning_interface::MoveItErrorCode::SUCCESS
             && num_attempts >= max_planning_attempts - 1)
    {
      toggleGripperCollisions(
          goal->index >= 0
          ? manipulation_actions::ToggleGripperCollisions::Request::ALL_OBJECTS_NAME
          : manipulation_actions::ToggleGripperCollisions::Request::OCTOMAP_NAME,
          false
      );

      ROS_INFO("Could not plan to the final grasp pose, aborting!");
      result.error_code = plan_result.val;
      result.success = false;
      result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::GRASP_PLAN;
      execute_grasp_server_.setAborted(result);
      return;
    }
    else if (plan_result.val != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      // Try again
      continue;
    }

    // make sure the plan doesn't do some roundabout RRT thing...
    size_t check_index = static_cast<size_t>(grasp_plan.trajectory_.joint_trajectory.points.size() / 2.0);
    double joint_error_check = 0.0;
    double joint_error_thresh = 0.75;  // TODO: this may need tuning to be more lenient in accepting plans
    for (size_t i = 0; i < grasp_plan.trajectory_.joint_trajectory.joint_names.size(); i ++)
    {
      joint_error_check += fabs(grasp_plan.trajectory_.joint_trajectory.points[0].positions[i]
                                - grasp_plan.trajectory_.joint_trajectory.points[check_index].positions[i]);
    }
    ROS_INFO("Joint error check on final grasp trajectory: %f", joint_error_check);

    if (execute_grasp_server_.isPreemptRequested())
    {
      toggleGripperCollisions(
          goal->index >= 0
          ? manipulation_actions::ToggleGripperCollisions::Request::ALL_OBJECTS_NAME
          : manipulation_actions::ToggleGripperCollisions::Request::OCTOMAP_NAME,
          false
      );

      ROS_INFO("Preempted while checking thresholds on plan.");
      result.error_code = moveit_msgs::MoveItErrorCodes::PREEMPTED;
      result.success = false;
      result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::GRASP_PLAN;
      execute_grasp_server_.setPreempted(result);
      return;
    }
    else if (joint_error_check > joint_error_thresh && num_attempts >= max_planning_attempts - 1)
    {
      toggleGripperCollisions(
          goal->index >= 0
          ? manipulation_actions::ToggleGripperCollisions::Request::ALL_OBJECTS_NAME
          : manipulation_actions::ToggleGripperCollisions::Request::OCTOMAP_NAME,
          false
      );

      ROS_INFO("Could not safely plan to the final grasp pose, aborting!");
      result.error_code = plan_result.val;
      result.success = false;
      result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::GRASP_PLAN;
      execute_grasp_server_.setAborted(result);
      return;
    }
    else if (joint_error_check <= joint_error_thresh)
    {
      // This is valid. Exit the loop!
      break;
    }
  }

  // execute grasp plan
  result.error_code = arm_group_->execute(grasp_plan).val;
  if (result.error_code == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    toggleGripperCollisions(
        goal->index >= 0
        ? manipulation_actions::ToggleGripperCollisions::Request::ALL_OBJECTS_NAME
        : manipulation_actions::ToggleGripperCollisions::Request::OCTOMAP_NAME,
        false
    );

    ROS_INFO("Preempted from MoveIt! while moving to final grasp pose. Aborting.");
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::GRASP_EXECUTION;
    execute_grasp_server_.setAborted(result);
    return;
  }
  else if (result.error_code != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    toggleGripperCollisions(
        goal->index >= 0
        ? manipulation_actions::ToggleGripperCollisions::Request::ALL_OBJECTS_NAME
        : manipulation_actions::ToggleGripperCollisions::Request::OCTOMAP_NAME,
        false
    );

    ROS_INFO("Failed to move to final grasp pose.");
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::GRASP_EXECUTION;
    execute_grasp_server_.setAborted(result);
    return;
  }

//  // Linear approach (to be replaced by above code)
//  moveit_msgs::GetCartesianPath grasp_path;
//  grasp_path.request.waypoints.push_back(transformed_grasp_pose.pose);
//  grasp_path.request.max_step = 0.01;
//  grasp_path.request.jump_threshold = 1.5;  // From nimbus
//  grasp_path.request.avoid_collisions = false;
//  grasp_path.request.group_name = "arm";
//  moveit::core::robotStateToRobotStateMsg(*(arm_group_->getCurrentState()), grasp_path.request.start_state);
//
//  int max_planning_attempts = 10;
//  for (int num_attempts=0; num_attempts < max_planning_attempts; num_attempts++)
//  {
//    ROS_INFO("Attempting to plan path to grasp. Attempt: %d/%d",
//             num_attempts + 1, max_planning_attempts);
//    if (execute_grasp_server_.isPreemptRequested())
//    {
//      toggleGripperCollisions(
//          goal->index >= 0
//          ? manipulation_actions::ToggleGripperCollisions::Request::ALL_OBJECTS_NAME
//          : manipulation_actions::ToggleGripperCollisions::Request::OCTOMAP_NAME,
//          false
//      );
//
//      ROS_INFO("Preempted while planning grasp.");
//      result.error_code = moveit_msgs::MoveItErrorCodes::PREEMPTED;
//      result.success = false;
//      result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::GRASP_PLAN;
//      execute_grasp_server_.setPreempted(result);
//      return;
//    }
//    else if (grasp_path.response.fraction >= 0.5)
//    {
//      ROS_INFO("Succeeded in computing %f of the path to grasp",
//               grasp_path.response.fraction);
//      break;
//    }
//    else if (!compute_cartesian_path_client_.call(grasp_path)
//             || grasp_path.response.fraction < 0
//             || num_attempts >= max_planning_attempts - 1)
//    {
//      toggleGripperCollisions(
//          goal->index >= 0
//          ? manipulation_actions::ToggleGripperCollisions::Request::ALL_OBJECTS_NAME
//          : manipulation_actions::ToggleGripperCollisions::Request::OCTOMAP_NAME,
//          false
//      );
//
//      ROS_INFO("Could not calculate a Cartesian path for grasp!");
//      result.error_code = grasp_path.response.error_code.val;
//      result.success = false;
//      result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::GRASP_PLAN;
//      execute_grasp_server_.setAborted(result);
//      return;
//    }
//  }
//
//  //execute the grasp plan
//  moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
//  grasp_plan.trajectory_ = grasp_path.response.solution;
//  moveit::core::robotStateToRobotStateMsg(*(arm_group_->getCurrentState()), grasp_plan.start_state_);
//  if (execute_grasp_server_.isPreemptRequested())
//  {
//    toggleGripperCollisions(
//        goal->index >= 0
//        ? manipulation_actions::ToggleGripperCollisions::Request::ALL_OBJECTS_NAME
//        : manipulation_actions::ToggleGripperCollisions::Request::OCTOMAP_NAME,
//        false
//    );
//
//    ROS_INFO("Preempted while executing grasp.");
//    result.error_code = moveit_msgs::MoveItErrorCodes::PREEMPTED;
//    result.success = false;
//    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::GRASP_EXECUTION;
//    execute_grasp_server_.setPreempted(result);
//    return;
//  }
//  result.error_code = arm_group_->execute(grasp_plan).val;
//  if (result.error_code == moveit_msgs::MoveItErrorCodes::PREEMPTED)
//  {
//    toggleGripperCollisions(
//        goal->index >= 0
//        ? manipulation_actions::ToggleGripperCollisions::Request::ALL_OBJECTS_NAME
//        : manipulation_actions::ToggleGripperCollisions::Request::OCTOMAP_NAME,
//        false
//    );
//
//    ROS_INFO("Preempted while moving to executing grasp.");
//    result.success = false;
//    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::GRASP_EXECUTION;
//    execute_grasp_server_.setPreempted(result);
//    return;
//  }
//  else if (result.error_code != moveit_msgs::MoveItErrorCodes::SUCCESS)
//  {
//    toggleGripperCollisions(
//        goal->index >= 0
//        ? manipulation_actions::ToggleGripperCollisions::Request::ALL_OBJECTS_NAME
//        : manipulation_actions::ToggleGripperCollisions::Request::OCTOMAP_NAME,
//        false
//    );
//
//    ROS_INFO("Failed to move to execute grasp.");
//    result.success = false;
//    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::GRASP_EXECUTION;
//    execute_grasp_server_.setAborted(result);
//    return;
//  }

  //close gripper
  gripper_goal.command.position = 0;
  gripper_goal.command.max_effort = 100;
  gripper_client_.sendGoal(gripper_goal);
  while (!gripper_client_.getState().isDone())
  {
    if (execute_grasp_server_.isPreemptRequested())
    {
      gripper_client_.cancelGoal();
    }
  }
  if (gripper_client_.getState() == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    toggleGripperCollisions(
        goal->index >= 0
        ? manipulation_actions::ToggleGripperCollisions::Request::ALL_OBJECTS_NAME
        : manipulation_actions::ToggleGripperCollisions::Request::OCTOMAP_NAME,
        false
    );

    ROS_INFO("Preempted while closing gripper.");
    result.error_code = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::GRASP_EXECUTION;
    execute_grasp_server_.setPreempted(result);
    return;
  }
  else if (gripper_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    toggleGripperCollisions(
        goal->index >= 0
        ? manipulation_actions::ToggleGripperCollisions::Request::ALL_OBJECTS_NAME
        : manipulation_actions::ToggleGripperCollisions::Request::OCTOMAP_NAME,
        false
    );

    ROS_INFO("Failed while closing gripper.");
    result.error_code = moveit_msgs::MoveItErrorCodes::FAILURE;
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::GRASP_EXECUTION;
    execute_grasp_server_.setAborted(result);
    return;
  }

  //attach objects
  if (goal->index >= 0)
  {
    //attach nearby object if there was a specific object to pick.
    std_srvs::Empty attach_object_srv;
    if (!attach_closest_object_client_.call(attach_object_srv))
    {
      ROS_INFO("Failed to attach an object to the gripper");
    }
    else
    {
      ROS_INFO("Picked object attached to the gripper");
    }
  }
  else
  {
    //attach an arbitrary object if this was a cluttered pick.
    manipulation_actions::AttachArbitraryObject attach_object_srv;
    attach_object_srv.request.challenge_object.object = manipulation_actions::ChallengeObject::NONE;
    if (!attach_arbitrary_object_client_.call(attach_object_srv))
    {
      ROS_INFO("Failed to attach a virtual object to the gripper");
    }
    else
    {
      ROS_INFO("Virtual object attached to the gripper");
    }
  }

  //sleep to allow the attachments to propagate
  ros::Duration(0.2).sleep();

  //reenable collisions on the gripper
  toggleGripperCollisions(
      goal->index >= 0
      ? manipulation_actions::ToggleGripperCollisions::Request::ALL_OBJECTS_NAME
      : manipulation_actions::ToggleGripperCollisions::Request::OCTOMAP_NAME,
      false
  );

  //unplanned move directly upwards
  float arm_velocity = CARTESIAN_MOVE_VELOCITY * (goal->max_velocity_scaling_factor > 0
                                                  ? goal->max_velocity_scaling_factor
                                                  : MAX_VELOCITY_SCALING_FACTOR);
  float desired_move_amount = 0.05;                                   // meters
  float desired_duration = fabs(desired_move_amount / arm_velocity);  // seconds
  ros::Rate vel_publish_freq(30);                                     // Hz
  geometry_msgs::TwistStamped vel_msg;
  vel_msg.header.frame_id = "base_link";
  vel_msg.twist.linear.z = arm_velocity;

  double start_time = ros::Time::now().toSec();
  while (ros::Time::now().toSec() < start_time + desired_duration)
  {
    vel_msg.header.stamp = ros::Time::now();
    cartesian_pub_.publish(vel_msg);

    // check for a preempt
    if (execute_grasp_server_.isPreemptRequested())
    {
      ROS_INFO("Preempted during linear move up.");
      result.error_code = moveit_msgs::MoveItErrorCodes::PREEMPTED;
      result.success = false;
      result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::PICK_UP_EXECUTION;
      execute_grasp_server_.setPreempted(result);
      return;
    }

    // sleep
    vel_publish_freq.sleep();
  }
  ROS_INFO("Completed linear move upward");

  // Linear planned move upwards. Replaced by the unplanned move above
//  //linear move to post grasp pose
//  moveit_msgs::GetCartesianPath lift_path;
//  transformed_grasp_pose.pose.position.z += 0.2;
//  test2_.publish(transformed_grasp_pose);
//  lift_path.request.waypoints.push_back(transformed_grasp_pose.pose);
//  lift_path.request.max_step = 0.01;
//  lift_path.request.jump_threshold = 1.5;  // from above
//  lift_path.request.avoid_collisions = false;
//  lift_path.request.group_name = "arm";
//  moveit::core::robotStateToRobotStateMsg(*(arm_group_->getCurrentState()), lift_path.request.start_state);
//
//  if (execute_grasp_server_.isPreemptRequested())
//  {
//    ROS_INFO("Preempted while planning pick up.");
//    result.error_code = moveit_msgs::MoveItErrorCodes::PREEMPTED;
//    result.success = false;
//    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::PICK_UP_PLAN;
//    execute_grasp_server_.setPreempted(result);
//    return;
//  }
//  else if(!compute_cartesian_path_client_.call(lift_path)
//          || lift_path.response.fraction < 0)
//  {
//    ROS_INFO("Could not calculate a Cartesian path for pick up!");
//    result.error_code = lift_path.response.error_code.val;
//    result.success = false;
//    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::PICK_UP_PLAN;
//    execute_grasp_server_.setAborted(result);
//    return;
//  }
//  else
//  {
//    ROS_INFO("Succeeded in computing %f of the path to pick up", lift_path.response.fraction);
//  }
//
//  //execute the lift plan
//  moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
//  lift_plan.trajectory_ = lift_path.response.solution;
//  moveit::core::robotStateToRobotStateMsg(*(arm_group_->getCurrentState()), lift_plan.start_state_);
//  if (execute_grasp_server_.isPreemptRequested())
//  {
//    ROS_INFO("Preempted while picking up.");
//    result.error_code = moveit_msgs::MoveItErrorCodes::PREEMPTED;
//    result.success = false;
//    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::PICK_UP_EXECUTION;
//    execute_grasp_server_.setPreempted(result);
//    return;
//  }
//  result.error_code = arm_group_->execute(lift_plan).val;
//  if (result.error_code == moveit_msgs::MoveItErrorCodes::PREEMPTED)
//  {
//    ROS_INFO("Preempted while picking up.");
//    result.success = false;
//    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::PICK_UP_EXECUTION;
//    execute_grasp_server_.setPreempted(result);
//    return;
//  }
//  else if (result.error_code != moveit_msgs::MoveItErrorCodes::SUCCESS)
//  {
//    ROS_INFO("Failed to pick up.");
//    result.success = false;
//    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::PICK_UP_EXECUTION;
//    execute_grasp_server_.setAborted(result);
//    return;
//  }
//
//  //complete the rest of the trajectory, if there's anything left
//  arm_group_->setStartStateToCurrentState();
//  arm_group_->setPoseTarget(transformed_grasp_pose, "wrist_roll_link");
//  if (execute_grasp_server_.isPreemptRequested())
//  {
//    ROS_INFO("Preempted while picking up.");
//    result.error_code = moveit_msgs::MoveItErrorCodes::PREEMPTED;
//    result.success = false;
//    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::PICK_UP_EXECUTION;
//    execute_grasp_server_.setPreempted(result);
//    return;
//  }
//  result.error_code = arm_group_->move().val;
//  if (result.error_code == moveit_msgs::MoveItErrorCodes::PREEMPTED)
//  {
//    ROS_INFO("Preempted while picking up.");
//    result.success = false;
//    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::PICK_UP_EXECUTION;
//    execute_grasp_server_.setPreempted(result);
//    return;
//  }
//  else if (result.error_code != moveit_msgs::MoveItErrorCodes::SUCCESS)
//  {
//    ROS_INFO("Failed to pick up.");
//    result.success = false;
//    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::PICK_UP_EXECUTION;
//    execute_grasp_server_.setAborted(result);
//    return;
//  }

  // Reset the scaling factor
  arm_group_->setMaxVelocityScalingFactor(MAX_VELOCITY_SCALING_FACTOR);

  //DONE
  result.success = true;
  execute_grasp_server_.setSucceeded(result);
}

bool Executor::toggleGripperCollisions(std::string object, bool allow_collisions)
{
  manipulation_actions::ToggleGripperCollisions toggle_gripper_collisions_srv;
  toggle_gripper_collisions_srv.request.enable_collisions = allow_collisions;
  toggle_gripper_collisions_srv.request.object_name = object;

  bool return_state = toggle_gripper_collisions_client_.call(toggle_gripper_collisions_srv);
  if (!return_state)
  {
    ROS_INFO("Could not update the collisions in the current planning scene!");
  }
  else
  {
    ROS_INFO_STREAM("Updated planning scene collisions with "
                    << toggle_gripper_collisions_srv.request.object_name
                    << " to " << allow_collisions);
  }

  return return_state;
}

bool Executor::addObject(fetch_grasp_suggestion::AddObject::Request &req,
    fetch_grasp_suggestion::AddObject::Response &res)
{
  // boost::mutex::scoped_lock lock(object_mutex_);

  // vector<moveit_msgs::CollisionObject> collision_objects;
  // collision_objects.resize(req.point_clouds.size());

  // for (size_t i = 0; i < collision_objects.size(); i++) {
  //   //create collision object
  //   collision_objects[i].header.frame_id = req.point_clouds[i].header.frame_id;
  //   stringstream ss;
  //   ss << "object" << req.indices[i];
  //   collision_objects[i].id = ss.str();

  //   //calculate bounding box
  //   fetch_grasp_suggestion::BoundingBox bounding_box = BoundingBoxCalculator::computeBoundingBox(req.point_clouds[i]);

  //   //set object shape
  //   shape_msgs::SolidPrimitive bounding_volume;
  //   bounding_volume.type = shape_msgs::SolidPrimitive::BOX;
  //   bounding_volume.dimensions.resize(3);
  //   bounding_volume.dimensions[shape_msgs::SolidPrimitive::BOX_X] = bounding_box.dimensions.x;
  //   bounding_volume.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = bounding_box.dimensions.y;
  //   bounding_volume.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = bounding_box.dimensions.z;
  //   collision_objects[i].primitives.push_back(bounding_volume);
  //   collision_objects[i].primitive_poses.push_back(bounding_box.pose.pose);
  //   collision_objects[i].operation = moveit_msgs::CollisionObject::ADD;
  // }

  // planning_scene_interface_->addCollisionObjects(collision_objects);

  // NOTE: For Fetchit, the CollisionSceneManager will have made this redundant
  ROS_INFO("NOOP: CollisionSceneManager will have updated the objects in the scene");

  return true;
}

bool Executor::clearObjects(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  boost::mutex::scoped_lock lock(object_mutex_);

  return this->clearAll();
}

bool Executor::clearAll()
{
  std_srvs::Empty detach_objects_srv;
  return detach_objects_client_.call(detach_objects_srv);
}

bool Executor::dropObjectCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  boost::mutex::scoped_lock lock(object_mutex_);

  //open gripper
  control_msgs::GripperCommandGoal gripper_goal;
  gripper_goal.command.position = 0.15;
  gripper_client_.sendGoal(gripper_goal);
  gripper_client_.waitForResult(ros::Duration(5.0));

  this->clearAll();
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "executor");

  Executor e;

  ros::spin();

  return EXIT_SUCCESS;
}
