#include <fetch_grasp_suggestion/executor.h>

using std::string;
using std::stringstream;
using std::vector;

const float MAX_VELOCITY_SCALING_FACTOR = 0.3;

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

  planning_scene_interface_ = new moveit::planning_interface::PlanningSceneInterface();

  planning_scene_publisher_ = n_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
  test1_ = pnh_.advertise<geometry_msgs::PoseStamped>("pose1", 1);
  test2_ = pnh_.advertise<geometry_msgs::PoseStamped>("pose2", 1);

  compute_cartesian_path_client_ = n_.serviceClient<moveit_msgs::GetCartesianPath>("/compute_cartesian_path");
  planning_scene_client_ = n_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  add_object_server_ = pnh_.advertiseService("add_object", &Executor::addObject, this);
  clear_objects_server_ = pnh_.advertiseService("clear_objects", &Executor::clearObjects, this);
  detach_objects_server_ = pnh_.advertiseService("detach_objects", &Executor::detachObjectsCallback, this);
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
    ROS_INFO("Preempted while moving to ready pose.");
    result.success = false;
    prepare_robot_server_.setPreempted(result);
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
    ROS_INFO("Preempted while moving to dropoff.");
    result.success = false;
    drop_pose_server_.setPreempted(result);
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

  stringstream ss;
  ss << "object" << goal->index;
  string object = ss.str();
  moveit_msgs::Grasp grasp;

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
    ROS_INFO("Preempted while moving to approach pose.");
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::APPROACH;
    execute_grasp_server_.setPreempted(result);
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
  moveit_msgs::GetPlanningScene planning_scene_srv;
  vector<string> collision_objects;
  planning_scene_srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

  if (!planning_scene_client_.call(planning_scene_srv))
  {
    ROS_INFO("Could not get the current planning scene!");
  }
  else
  {
    collision_detection::AllowedCollisionMatrix acm(planning_scene_srv.response.scene.allowed_collision_matrix);

    if (goal->index >= 0)
    {
      // disable all object collisions for final short-distance move
      collision_objects = planning_scene_interface_->getKnownObjectNames(false);
      for (size_t i = 0; i < collision_objects.size(); i++)
      {
        acm.setEntry(collision_objects[i], gripper_names_, true);
      }
    }
    else  // no-object mode
    {
      // disable collisions between gripper links and octomap
      acm.setEntry("<octomap>", gripper_names_, true);
    }
    moveit_msgs::PlanningScene planning_scene_update;
    acm.getMessage(planning_scene_update.allowed_collision_matrix);
    planning_scene_update.is_diff = true;
    planning_scene_publisher_.publish(planning_scene_update);

    ros::Duration(0.5).sleep(); //delay for publish to go through
  }

// TODO: Remove this commented out section when we have verified that the code works!
// <<<<<<< HEAD
//   //linear move to grasp pose
//   arm_group_->setStartStateToCurrentState();
//   arm_group_->setPoseTarget(transformed_grasp_pose, "wrist_roll_link");
//   result.error_code = arm_group_->move().val;
//   if (result.error_code != moveit_msgs::MoveItErrorCodes::SUCCESS)
// =======
  //linear plan to grasp pose
  test1_.publish(transformed_grasp_pose);
  moveit_msgs::GetCartesianPath grasp_path;
  grasp_path.request.waypoints.push_back(transformed_grasp_pose.pose);
  grasp_path.request.max_step = 0.01;
  grasp_path.request.jump_threshold = 1.5;  // From nimbus
  grasp_path.request.avoid_collisions = false;
  grasp_path.request.group_name = "arm";
  moveit::core::robotStateToRobotStateMsg(*(arm_group_->getCurrentState()), grasp_path.request.start_state);

  int max_planning_attempts = 10;
  for (int num_attempts=0; num_attempts < max_planning_attempts; num_attempts++)
  {
    ROS_INFO("Attempting to plan path to grasp. Attempt: %d/%d",
             num_attempts + 1, max_planning_attempts);
    if (execute_grasp_server_.isPreemptRequested())
    {
      ROS_INFO("Preempted while planning grasp.");
      result.error_code = moveit_msgs::MoveItErrorCodes::PREEMPTED;
      result.success = false;
      result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::GRASP_PLAN;
      execute_grasp_server_.setPreempted(result);
      return;
    }
    else if (grasp_path.response.fraction >= 0.5)
    {
      ROS_INFO("Succeeded in computing %f of the path to grasp",
               grasp_path.response.fraction);
      break;
    }
    else if (!compute_cartesian_path_client_.call(grasp_path)
             || grasp_path.response.fraction < 0
             || num_attempts >= max_planning_attempts - 1)
    {
      ROS_INFO("Could not calculate a Cartesian path for grasp!");
      result.error_code = grasp_path.response.error_code.val;
      result.success = false;
      result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::GRASP_PLAN;
      execute_grasp_server_.setAborted(result);
      return;
    }
  }

  //execute the grasp plan
  moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
  grasp_plan.trajectory_ = grasp_path.response.solution;
  moveit::core::robotStateToRobotStateMsg(*(arm_group_->getCurrentState()), grasp_plan.start_state_);
  if (execute_grasp_server_.isPreemptRequested())
// >>>>>>> 89d554a... Local updates to the package.
  {
    ROS_INFO("Preempted while executing grasp.");
    result.error_code = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::GRASP_EXECUTION;
    execute_grasp_server_.setPreempted(result);
    return;
  }
  result.error_code = arm_group_->execute(grasp_plan).val;
  if (result.error_code == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    ROS_INFO("Preempted while moving to executing grasp.");
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::GRASP_EXECUTION;
    execute_grasp_server_.setPreempted(result);
    return;
  }
  else if (result.error_code != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("Failed to move to execute grasp.");
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::GRASP_EXECUTION;
    execute_grasp_server_.setAborted(result);
    return;
  }

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
    ROS_INFO("Preempted while closing gripper.");
    result.error_code = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::GRASP_EXECUTION;
    execute_grasp_server_.setPreempted(result);
    return;
  }
  else if (gripper_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Failed while closing gripper.");
    result.error_code = moveit_msgs::MoveItErrorCodes::FAILURE;
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::GRASP_EXECUTION;
    execute_grasp_server_.setPreempted(result);
    return;
  }

  //attach nearby objects
  geometry_msgs::TransformStamped eef_transform =
      tf_buffer_.lookupTransform(group_reference_frame, "gripper_link", ros::Time(0), ros::Duration(3.0));
  std::map<string, geometry_msgs::Pose> object_poses = planning_scene_interface_->getObjectPoses(
      planning_scene_interface_->getKnownObjectNames(false));
  if (goal->index >= 0)
  {
    vector<string> attach_objects;
    bool target_object_attached = false;
    for (std::map<string, geometry_msgs::Pose>::iterator it = object_poses.begin(); it != object_poses.end(); it ++)
    {
      geometry_msgs::Pose object_pose = it->second;
      if (sqrt(pow(eef_transform.transform.translation.x - object_pose.position.x, 2)
               + pow(eef_transform.transform.translation.y - object_pose.position.y, 2)
               + pow(eef_transform.transform.translation.z - object_pose.position.z, 2)) < .15)
      {
        attach_objects.push_back(it->first);
        target_object_attached = target_object_attached || (object == it->first);
      }
    }
    if (!target_object_attached)
    {
      attach_objects.push_back(object);
    }
    for (size_t i = 0; i < attach_objects.size(); i++)
    {
      ROS_INFO("Attaching %s", attach_objects[i].c_str());
      arm_group_->attachObject(attach_objects[i], arm_group_->getEndEffectorLink(), gripper_names_);
      attached_objects_.push_back(attach_objects[i]);
      ros::Duration(0.2).sleep();  // wait for change to go through (seems to have a race condition otherwise)
    }
    //attached_objects_.push_back(object);
    for (size_t j = 0; j < collision_objects.size(); j++)
    {
      this->enableGripperCollision(collision_objects[j]);
    }
  }
  else
  {
    // create virtual object at gripper
    vector<moveit_msgs::CollisionObject> virtual_objects;
    virtual_objects.resize(1);
    virtual_objects[0].header.frame_id = group_reference_frame;
    virtual_objects[0].id = "virtual_object";

    // set object shape
    shape_msgs::SolidPrimitive bounding_volume;
    bounding_volume.type = shape_msgs::SolidPrimitive::SPHERE;
    bounding_volume.dimensions.resize(1);
    bounding_volume.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = 0.1;
    virtual_objects[0].primitives.push_back(bounding_volume);

    // set object pose
    geometry_msgs::Pose virtual_object_pose;
    virtual_object_pose.position.x = eef_transform.transform.translation.x;
    virtual_object_pose.position.y = eef_transform.transform.translation.y;
    virtual_object_pose.position.z = eef_transform.transform.translation.z;
    virtual_object_pose.orientation = eef_transform.transform.rotation;
    virtual_objects[0].primitive_poses.push_back(virtual_object_pose);

    virtual_objects[0].operation = moveit_msgs::CollisionObject::ADD;
    planning_scene_interface_->addCollisionObjects(virtual_objects);
    ros::Duration(0.5).sleep();  // wait for scene to be updated...

    // attach virtual object to gripper
    attached_objects_.push_back("virtual_object");
    arm_group_->attachObject("virtual_object", arm_group_->getEndEffectorLink(), gripper_names_);
    ros::Duration(0.2).sleep();  // wait for change to go through (seems to have a race condition otherwise)

    // enable gripper collision with octomap
    this->enableGripperCollision("<octomap>");
  }

  //linear move to post grasp pose
  moveit_msgs::GetCartesianPath lift_path;
  transformed_grasp_pose.pose.position.z += 0.2;
  test2_.publish(transformed_grasp_pose);
  lift_path.request.waypoints.push_back(transformed_grasp_pose.pose);
  lift_path.request.max_step = 0.01;
  lift_path.request.jump_threshold = 1.5;  // from above
  lift_path.request.avoid_collisions = false;
  lift_path.request.group_name = "arm";
  moveit::core::robotStateToRobotStateMsg(*(arm_group_->getCurrentState()), lift_path.request.start_state);

  if (execute_grasp_server_.isPreemptRequested())
  {
    ROS_INFO("Preempted while planning pick up.");
    result.error_code = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::PICK_UP_PLAN;
    execute_grasp_server_.setPreempted(result);
    return;
  }
  else if(!compute_cartesian_path_client_.call(lift_path)
          || lift_path.response.fraction < 0)
  {
    ROS_INFO("Could not calculate a Cartesian path for pick up!");
    result.error_code = lift_path.response.error_code.val;
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::PICK_UP_PLAN;
    execute_grasp_server_.setAborted(result);
    return;
  }
  else
  {
// TODO: Remove this commented out section when we have verified that the code works!
// <<<<<<< HEAD
//     moveit::planning_interface::MoveGroupInterface::Plan liftPlan;
//     liftPlan.trajectory_ = lift_path.response.solution;
//     moveit::core::robotStateToRobotStateMsg(*(arm_group_->getCurrentState()), liftPlan.start_state_);
//     result.error_code = arm_group_->execute(liftPlan).val;

//     arm_group_->setStartStateToCurrentState();
//     arm_group_->setPoseTarget(transformed_grasp_pose, "wrist_roll_link");
//     result.error_code = arm_group_->move().val;
//     if (result.error_code != moveit_msgs::MoveItErrorCodes::SUCCESS)
//     {
//       ROS_INFO("Failed to move to lift pose.");
//       result.success = false;
//       result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::PICK_UP_EXECUTION;
//       execute_grasp_server_.setSucceeded(result);
//       return;
//     }
//   }
//   moveit::planning_interface::MoveGroupInterface::Plan liftPlan;
//   liftPlan.trajectory_ = lift_path.response.solution;
//   moveit::core::robotStateToRobotStateMsg(*(arm_group_->getCurrentState()), liftPlan.start_state_);
//   result.error_code = arm_group_->execute(liftPlan).val;
// =======
    ROS_INFO("Succeeded in computing %f of the path to pick up", lift_path.response.fraction);
  }

  //execute the lift plan
  moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
  lift_plan.trajectory_ = lift_path.response.solution;
  moveit::core::robotStateToRobotStateMsg(*(arm_group_->getCurrentState()), lift_plan.start_state_);
  if (execute_grasp_server_.isPreemptRequested())
  {
    ROS_INFO("Preempted while picking up.");
    result.error_code = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::PICK_UP_EXECUTION;
    execute_grasp_server_.setPreempted(result);
    return;
  }
  result.error_code = arm_group_->execute(lift_plan).val;
  if (result.error_code == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    ROS_INFO("Preempted while picking up.");
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::PICK_UP_EXECUTION;
    execute_grasp_server_.setPreempted(result);
    return;
  }
  else if (result.error_code != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("Failed to pick up.");
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::PICK_UP_EXECUTION;
    execute_grasp_server_.setAborted(result);
    return;
  }

  //complete the rest of the trajectory, if there's anything left
  arm_group_->setStartStateToCurrentState();
  arm_group_->setPoseTarget(transformed_grasp_pose, "wrist_roll_link");
  if (execute_grasp_server_.isPreemptRequested())
  {
    ROS_INFO("Preempted while picking up.");
    result.error_code = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::PICK_UP_EXECUTION;
    execute_grasp_server_.setPreempted(result);
    return;
  }
  result.error_code = arm_group_->move().val;
  if (result.error_code == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    ROS_INFO("Preempted while picking up.");
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::PICK_UP_EXECUTION;
    execute_grasp_server_.setPreempted(result);
    return;
  }
  else if (result.error_code != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("Failed to pick up.");
    result.success = false;
    result.failure_point = fetch_grasp_suggestion::ExecuteGraspResult::PICK_UP_EXECUTION;
    execute_grasp_server_.setAborted(result);
    return;
  }
// >>>>>>> 89d554a... Local updates to the package.

  //DONE
  result.success = true;
  execute_grasp_server_.setSucceeded(result);
}

void Executor::enableGripperCollision(string object)
{
  moveit_msgs::GetPlanningScene planning_scene_srv;
  planning_scene_srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  if (!planning_scene_client_.call(planning_scene_srv))
  {
    ROS_INFO("Could not get the current planning scene!");
  }
  else
  {
    collision_detection::AllowedCollisionMatrix acm(planning_scene_srv.response.scene.allowed_collision_matrix);
    acm.setEntry(object, gripper_names_, false);

    moveit_msgs::PlanningScene planning_scene_update;
    acm.getMessage(planning_scene_update.allowed_collision_matrix);
    planning_scene_update.is_diff = true;
    planning_scene_publisher_.publish(planning_scene_update);
  }
}

bool Executor::addObject(fetch_grasp_suggestion::AddObject::Request &req,
    fetch_grasp_suggestion::AddObject::Response &res)
{
  boost::mutex::scoped_lock lock(object_mutex_);

  vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(req.point_clouds.size());

  for (size_t i = 0; i < collision_objects.size(); i++) {
    //create collision object
    collision_objects[i].header.frame_id = req.point_clouds[i].header.frame_id;
    stringstream ss;
    ss << "object" << req.indices[i];
    collision_objects[i].id = ss.str();

    //calculate bounding box
    fetch_grasp_suggestion::BoundingBox bounding_box = BoundingBoxCalculator::computeBoundingBox(req.point_clouds[i]);

    //set object shape
    shape_msgs::SolidPrimitive bounding_volume;
    bounding_volume.type = shape_msgs::SolidPrimitive::BOX;
    bounding_volume.dimensions.resize(3);
    bounding_volume.dimensions[shape_msgs::SolidPrimitive::BOX_X] = bounding_box.dimensions.x;
    bounding_volume.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = bounding_box.dimensions.y;
    bounding_volume.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = bounding_box.dimensions.z;
    collision_objects[i].primitives.push_back(bounding_volume);
    collision_objects[i].primitive_poses.push_back(bounding_box.pose.pose);
    collision_objects[i].operation = moveit_msgs::CollisionObject::ADD;
  }

  planning_scene_interface_->addCollisionObjects(collision_objects);

  ROS_INFO("Collision objects updated.");

  return true;
}

bool Executor::detachObjectsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  boost::mutex::scoped_lock lock(object_mutex_);

  this->detachObjects();
  return true;
}

void Executor::detachObjects()
{
  for (size_t i = 0; i < attached_objects_.size(); i++)
  {
    ROS_INFO("Detaching %s", attached_objects_[i].c_str());
    arm_group_->detachObject(attached_objects_[i]);
  }
  attached_objects_.clear();
}

bool Executor::clearObjects(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  boost::mutex::scoped_lock lock(object_mutex_);

  this->clearAll();

  return true;
}

void Executor::clearAll()
{
  this->detachObjects();
  planning_scene_interface_->removeCollisionObjects(planning_scene_interface_->getKnownObjectNames(false));
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
