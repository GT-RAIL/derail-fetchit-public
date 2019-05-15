#include <manipulation_actions/SchunkGearGrasper.h>

using std::ios;
using std::string;
using std::stringstream;
using std::vector;


SchunkGearGrasper::SchunkGearGrasper() :
    pnh("~"),
    tf_listener(tf_buffer),
    gripper_client("gripper_controller/gripper_action"),
    linear_move_client("linear_controller/linear_move"),
    schunk_gear_grasp_server(pnh, "grasp_schunk_gear", boost::bind(&SchunkGearGrasper::executeSchunkGearGrasp, this, _1), false),
    schunk_gear_retrieve_server(pnh, "retrieve_schunk_gear", boost::bind(&SchunkGearGrasper::executeSchunkGearRetrieve, this, _1), false)
//    debug_attach_object_server(pnh, "attach_gear_to_gripper", boost::bind(&SchunkGearGrasper::debugAttachObject, this, _1), false)
{
  pnh.param<double>("approach_offset_in_x", approach_offset_in_x, -0.12);
  pnh.param<int>("max_planning_attempts_in_", max_planning_attempts, 3);
  pnh.param<double>("retrieve_offset", retrieve_offset_in_x, -0.05);

  approach_pose_debug = pnh.advertise<geometry_msgs::PoseStamped>("approach_pose_debug", 1);
  grasp_pose_debug = pnh.advertise<geometry_msgs::PoseStamped>("grasp_pose_debug", 1);
  takeout_pose_debug = pnh.advertise<geometry_msgs::PoseStamped>("takeout_pose_debug", 1);

  attach_simple_geometry_client =
    n.serviceClient<manipulation_actions::AttachSimpleGeometry>("/collision_scene_manager/attach_simple_geometry");

  arm_group = new moveit::planning_interface::MoveGroupInterface("arm");
  arm_group->startStateMonitor();

  planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();

  initGraspPoses();

  schunk_gear_grasp_server.start();
  schunk_gear_retrieve_server.start();
//  debug_attach_object_server.start();
}


void SchunkGearGrasper::initGraspPoses()
{
  // Grasp pose is defined for gripper link in respective frame
  // More than one grasp poses can be defined. Each one will be executed until success.
  grasp_poses.resize(1);

  grasp_poses[0].header.frame_id = "template_pose";
  grasp_poses[0].pose.position.x = 0.073;
  grasp_poses[0].pose.position.y = -0.001;
  grasp_poses[0].pose.position.z = 0;
  grasp_poses[0].pose.orientation.x = 0.846;
  grasp_poses[0].pose.orientation.y = -0.062;
  grasp_poses[0].pose.orientation.z = -0.457;
  grasp_poses[0].pose.orientation.w = 0.267;

  current_grasp_pose = 0;
}



void SchunkGearGrasper::executeSchunkGearGrasp(const manipulation_actions::SchunkGraspGoalConstPtr &goal)
{
  manipulation_actions::SchunkGraspResult result;
  bool grasp_succeeded = false;
  bool approach_succeeded = false;

  for (size_t i = 0; i < grasp_poses.size(); i ++)
  {
    current_grasp_pose = i;

    // group reference frame is wrist_roll_link
    string group_reference_frame = arm_group->getPoseReferenceFrame();

    // 1. Move to approach pose
    // grasp pose is defined for gripper in a specified coordinate frame
    // transform this pose to reference group coordinate frame
    geometry_msgs::PoseStamped grasp_pose;
    if (grasp_poses[i].header.frame_id != group_reference_frame)
    {
        grasp_pose.header.stamp = ros::Time(0);
        grasp_pose.header.frame_id = group_reference_frame;

        geometry_msgs::TransformStamped group_to_grasp_transform = tf_buffer.lookupTransform(group_reference_frame,
                                                                                              grasp_poses[i].header.frame_id,
                                                                                              ros::Time(0),
                                                                                              ros::Duration(1.0));
        tf2::doTransform(grasp_poses[i], grasp_pose, group_to_grasp_transform);
    } else
    {
        grasp_pose = grasp_poses[i];
    }

    // create grasp frame with regard to reference group coordinate frame
    geometry_msgs::TransformStamped grasp_transform;
    ros::Time current_time = ros::Time::now();
    grasp_transform.child_frame_id = "grasp_frame";
    grasp_transform.header.frame_id = grasp_pose.header.frame_id;
    grasp_transform.header.stamp = current_time;
    grasp_transform.transform.translation.x = grasp_pose.pose.position.x;
    grasp_transform.transform.translation.y = grasp_pose.pose.position.y;
    grasp_transform.transform.translation.z = grasp_pose.pose.position.z;
    grasp_transform.transform.rotation = grasp_pose.pose.orientation;
    tf_broadcaster.sendTransform(grasp_transform);

    // look up goal pose that wrist should move to in order to have gripper aligned with grasp pose (in grasp frame).
    // MoveIt plans for wrist instead of gripper
    // look up transform between gripper and wrist
    geometry_msgs::TransformStamped wrist_transform = tf_buffer.lookupTransform("gripper_link", "wrist_roll_link",
                                                                                 ros::Time(0), ros::Duration(3.0));
    // calculate goal pose for wrist
    geometry_msgs::PoseStamped wrist_grasp_pose;
    wrist_grasp_pose.header.frame_id = "grasp_frame";
    wrist_grasp_pose.pose.position.x = wrist_transform.transform.translation.x;
    wrist_grasp_pose.pose.position.y = wrist_transform.transform.translation.y;
    wrist_grasp_pose.pose.position.z = wrist_transform.transform.translation.z;
    wrist_grasp_pose.pose.orientation = wrist_transform.transform.rotation;

    // transform goal pose and approach pose for wrist in wrist_roll_link frame
    geometry_msgs::TransformStamped from_grasp_transform = tf_buffer.lookupTransform(group_reference_frame,
                                                                                     "grasp_frame", current_time, ros::Duration(3.0));
    // calculate goal pose for wrist in wrist_roll_link frame
    geometry_msgs::PoseStamped transformed_grasp_pose;
    transformed_grasp_pose.header.frame_id = group_reference_frame;
    tf2::doTransform(wrist_grasp_pose, transformed_grasp_pose, from_grasp_transform);

    // calculate approach pose for wrist in wrist_roll_link frame
    wrist_grasp_pose.pose.position.x = approach_offset_in_x;
    geometry_msgs::PoseStamped transformed_approach_pose;
    transformed_approach_pose.header.frame_id = group_reference_frame;
    tf2::doTransform(wrist_grasp_pose, transformed_approach_pose, from_grasp_transform);

    approach_pose_debug.publish(transformed_approach_pose);

    // combine plan and execute
    // plan and move to approach pose
    // arm_group->setPlannerId("arm[RRTConnectkConfigDefault]");
    // arm_group->setPlanningTime(1.5);
    // arm_group->setStartStateToCurrentState();
    // arm_group->setPoseTarget(transformed_approach_pose, "wrist_roll_link");

    // moveit_msgs::MoveItErrorCodes error_code = arm_group->move();
    // if (error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
    // {
    //   ROS_INFO("Preempted while moving to approach pose. Will try again");
    //   result.error_code = manipulation_actions::SchunkGraspResult::PREP_FAILURE;
    //   schunk_gear_grasp_server.setAborted(result);
    //   return;
    // } else if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    // {
    //   ROS_INFO("Failed to move to approach pose %lu.", i);
    //   continue;
    // } else
    // {
    //   approach_succeeded = true;
    // }
    

    // separate plan and execute
    // a. plan
    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    if (!planToPose(transformed_approach_pose, approach_plan))
    {
      if (schunk_gear_grasp_server.isPreemptRequested())
      {
        ROS_INFO("Preempted while planning for approach planning");
        result.error_code = manipulation_actions::SchunkGraspResult::PREP_FAILURE;
        schunk_gear_grasp_server.setPreempted(result);
        return;
      } else
      {
        ROS_INFO("Failed to plan to this approach pose.");
        continue;
      }
    }
    // b. execute
    ROS_INFO("Moving to approach pose");
    moveit_msgs::MoveItErrorCodes error_code = arm_group->execute(approach_plan);
    if (error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
    {
      ROS_INFO("Preempted while moving to final grasp pose.");
      result.error_code = manipulation_actions::SchunkGraspResult::EXECUTION_FAILURE;
      schunk_gear_grasp_server.setAborted(result);
      return;
    } else if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_INFO("Failed to move to this pose, giving up on this pose...");
      continue;
    } else {
      approach_succeeded = true;
    }

    // 2. Open gripper
    control_msgs::GripperCommandGoal gripper_goal;
    gripper_goal.command.position = 1.0;
    gripper_goal.command.max_effort = 200;
    gripper_client.sendGoal(gripper_goal);
    ros::Rate gripper_open_wait_rate(100);
    while (!gripper_client.getState().isDone())
    {
      ros::spinOnce();
      gripper_open_wait_rate.sleep();
    }

    // 3. Move to grasp pose (execute with the linear controller)
    manipulation_actions::LinearMoveGoal grasp_goal;
    grasp_goal.hold_final_pose = true;

    // linear controller requires goals to be in the base_link frame
    if (grasp_pose.header.frame_id != "base_link")
    {
      geometry_msgs::PoseStamped grasp_pose_base;
      grasp_pose_base.header.stamp = ros::Time(0);
      grasp_pose_base.header.frame_id = "base_link";

      geometry_msgs::TransformStamped grasp_to_base_transform = tf_buffer.lookupTransform("base_link",
                                                                                           grasp_pose.header.frame_id,
                                                                                           ros::Time(0),
                                                                                           ros::Duration(1.0));
      tf2::doTransform(grasp_pose, grasp_pose_base, grasp_to_base_transform);

      grasp_goal.point.x = grasp_pose_base.pose.position.x;
      grasp_goal.point.y = grasp_pose_base.pose.position.y;
      grasp_goal.point.z = grasp_pose_base.pose.position.z;
    }
    else
    {
      grasp_goal.point.x = grasp_pose.pose.position.x;
      grasp_goal.point.y = grasp_pose.pose.position.y;
      grasp_goal.point.z = grasp_pose.pose.position.z;
    }

    grasp_pose_debug.publish(grasp_pose);

    linear_move_client.sendGoal(grasp_goal);
    linear_move_client.waitForResult();
    manipulation_actions::LinearMoveResultConstPtr linear_result = linear_move_client.getResult();
    actionlib::SimpleClientGoalState move_state = linear_move_client.getState();
    if (move_state.state_ != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Failed to move to final grasp pose, giving up on this pose...");
      continue;
    }
    else
    {
      grasp_succeeded = true;
      break;
    }
  }

  if (!approach_succeeded)
  {
    result.error_code = manipulation_actions::SchunkGraspResult::EXECUTION_FAILURE;
    schunk_gear_grasp_server.setAborted(result);
    return;
  }
  if (!grasp_succeeded)
  {
    result.error_code = manipulation_actions::SchunkGraspResult::EXECUTION_FAILURE;
    schunk_gear_grasp_server.setAborted(result);
    return;
  }

  // 4. Close gripper
  control_msgs::GripperCommandGoal close_goal;
  close_goal.command.position = 0;
  close_goal.command.max_effort = 100;
  gripper_client.sendGoal(close_goal);
  gripper_client.waitForResult(ros::Duration(5.0));

  // 5. finish
  result.error_code = manipulation_actions::SchunkGraspResult::SUCCESS;
  schunk_gear_grasp_server.setSucceeded(result);
  return;
}


void SchunkGearGrasper::executeSchunkGearRetrieve(const manipulation_actions::SchunkRetrieveGoalConstPtr &goal)
{
  manipulation_actions::SchunkRetrieveResult result;

  // Move horizontally to get gear out
  geometry_msgs::PoseStamped takeout_pose;
  takeout_pose.header.frame_id = "template_pose";
  takeout_pose.pose.position.x = retrieve_offset_in_x;
  takeout_pose.pose.position.y = 0;
  takeout_pose.pose.position.z = 0;
  takeout_pose.pose.orientation.x = 0;
  takeout_pose.pose.orientation.y = 0;
  takeout_pose.pose.orientation.z = 0;
  takeout_pose.pose.orientation.w = 1.0;

  geometry_msgs::PoseStamped takeout_pose_base;
  takeout_pose_base.header.stamp = ros::Time(0);
  takeout_pose_base.header.frame_id = "base_link";

  geometry_msgs::TransformStamped template_pose = tf_buffer.lookupTransform("base_link", "template_pose", ros::Time(0), ros::Duration(1.0));
  tf2::doTransform(takeout_pose, takeout_pose_base, template_pose);
  geometry_msgs::TransformStamped current_gripper_pose = tf_buffer.lookupTransform("base_link", "gripper_link", ros::Time(0), ros::Duration(1.0));
  takeout_pose_base.pose.orientation = current_gripper_pose.transform.rotation;

  takeout_pose_debug.publish(takeout_pose_base);

  manipulation_actions::LinearMoveGoal takeout_goal;
  takeout_goal.hold_final_pose = true;
  takeout_goal.point.x = takeout_pose_base.pose.position.x;
  takeout_goal.point.y = takeout_pose_base.pose.position.y;
  takeout_goal.point.z = takeout_pose_base.pose.position.z;
  linear_move_client.sendGoal(takeout_goal);
  linear_move_client.waitForResult();
  manipulation_actions::LinearMoveResultConstPtr linear_result = linear_move_client.getResult();
  actionlib::SimpleClientGoalState move_state = linear_move_client.getState();

  if (move_state.state_ != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Failed to move to retrieve pose");
    result.error_code = manipulation_actions::SchunkRetrieveResult::EXECUTION_FAILURE;
    schunk_gear_retrieve_server.setAborted(result);
    return;
  }

  // TODO: create and attach a fixed collision object representing the kit (wrist_roll_link),
  // TODO: and representing the mount (base_link)
  manipulation_actions::AttachSimpleGeometry collision;
  collision.request.name = "gear_in_gripper";
  collision.request.shape = manipulation_actions::AttachSimpleGeometryRequest::CYLINDER;
  collision.request.location = manipulation_actions::AttachSimpleGeometryRequest::END_EFFECTOR;
  collision.request.use_touch_links = true;
  collision.request.dims.resize(2);
  collision.request.dims[0] = 0.1206;  // height
  collision.request.dims[1] = 0.0349;  // radius
  collision.request.pose.header.frame_id = "template_pose";

  current_gripper_pose = tf_buffer.lookupTransform("template_pose", "gripper_link", ros::Time(0), ros::Duration(1.0));
  collision.request.pose.pose.position.x = current_gripper_pose.transform.translation.x;
  collision.request.pose.pose.position.y = current_gripper_pose.transform.translation.y;
  collision.request.pose.pose.position.z = current_gripper_pose.transform.translation.z;
  // rotate so that the base of the cylinder is in y and z.
  collision.request.pose.pose.orientation.x = 0;
  collision.request.pose.pose.orientation.y = 0.7071;
  collision.request.pose.pose.orientation.z = 0;
  collision.request.pose.pose.orientation.w = 0.7071;
  if (!attach_simple_geometry_client.call(collision))
  {
    ROS_INFO("Could not call attach simple geometry client!  Aborting.");
    result.error_code = manipulation_actions::SchunkRetrieveResult::EXECUTION_FAILURE;
    schunk_gear_retrieve_server.setAborted(result);
  }
  ros::Duration(2.0).sleep();  // let MoveIt! catch up after adding collision objects (this can be very slow)

  result.error_code = manipulation_actions::SchunkRetrieveResult::SUCCESS;
  schunk_gear_retrieve_server.setSucceeded(result);
  return;
}

//void SchunkGearGrasper::debugAttachObject(const manipulation_actions::KitManipGoalConstPtr &goal)
//{
//  // TODO: create and attach a fixed collision object representing the kit (wrist_roll_link),
//  // TODO: and representing the mount (base_link)
//  manipulation_actions::AttachSimpleGeometry collision;
//  collision.request.name = "gear_in_gripper";
//  collision.request.shape = manipulation_actions::AttachSimpleGeometryRequest::CYLINDER;
//  collision.request.location = manipulation_actions::AttachSimpleGeometryRequest::END_EFFECTOR;
//  collision.request.use_touch_links = true;
//  collision.request.dims.resize(2);
//  collision.request.dims[0] = 0.1206;  // height
//  collision.request.dims[1] = 0.0349;  // radius
//  collision.request.pose.header.frame_id = "template_pose";
//
//  geometry_msgs::TransformStamped current_gripper_pose = tf_buffer.lookupTransform("template_pose", "gripper_link", ros::Time(0), ros::Duration(1.0));
//  collision.request.pose.pose.position.x = current_gripper_pose.transform.translation.x;
//  collision.request.pose.pose.position.y = current_gripper_pose.transform.translation.y;
//  collision.request.pose.pose.position.z = current_gripper_pose.transform.translation.z;
//  collision.request.pose.pose.orientation.x = 0;
//  collision.request.pose.pose.orientation.y = 0.7071;
//  collision.request.pose.pose.orientation.z = 0;
//  collision.request.pose.pose.orientation.w = 0.7071;
//  if (!attach_simple_geometry_client.call(collision))
//  {
//    ROS_INFO("Could not call attach simple geometry client!  Aborting.");
//    debug_attach_object_server.setAborted();
//  }
//  ros::Duration(2.0).sleep();  // let MoveIt! catch up after adding collision objects (this can be very slow)
//}


bool SchunkGearGrasper::planToPose(geometry_msgs::PoseStamped& goal_pose,
                                   moveit::planning_interface::MoveGroupInterface::Plan& pose_plan)
{
  for (int num_attempts = 0; num_attempts < max_planning_attempts; num_attempts++)
  {
    ROS_INFO("Planning path to pose. Attempt: %d/%d", num_attempts + 1, max_planning_attempts);

    arm_group->setPlannerId("arm[RRTConnectkConfigDefault]");
    arm_group->setPlanningTime(1.5);
    arm_group->setStartStateToCurrentState();
    //        if (motion_speed_scale_factor_ != 1.0)
    //            arm_group_->setMaxVelocityScalingFactor(motion_speed_scale_factor);
    ROS_INFO("Goal pose frame: %s", goal_pose.header.frame_id.c_str());
    arm_group->setPoseTarget(goal_pose, "wrist_roll_link");

    // plans to the target pose
    moveit::planning_interface::MoveItErrorCode plan_result = arm_group->plan(pose_plan);

    // checks the results
    if (schunk_gear_grasp_server.isPreemptRequested())
    {
      ROS_INFO("Preempted while planning grasp");
      return false;
    } else if (plan_result.val != moveit::planning_interface::MoveItErrorCode::SUCCESS
               && num_attempts >= max_planning_attempts - 1)
    {
      ROS_INFO("Could not plan to this goal pose.");
      return false;
    } else if (plan_result.val != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      continue;
    } else
    {
      break; // successful plan so return
    }
  }
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "schunk_gear_retrieve_server");

  SchunkGearGrasper sgg;

  ros::spin();

  return EXIT_SUCCESS;
}
