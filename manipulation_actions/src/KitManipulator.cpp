#include <manipulation_actions/KitManipulator.h>

using std::ios;
using std::string;
using std::stringstream;
using std::vector;

KitManipulator::KitManipulator() :
    pnh("~"),
    tf_listener(tf_buffer),
    gripper_client("gripper_controller/gripper_action"),
    linear_move_client("linear_controller/linear_move"),
    store_object_server(pnh, "store_object", boost::bind(&KitManipulator::executeStore, this, _1), false),
    kit_pick_server(pnh, "pick_kit", boost::bind(&KitManipulator::executeKitPick, this, _1), false),
    kit_place_server(pnh, "place_kit_base", boost::bind(&KitManipulator::executeKitPlace, this, _1), false)
{
  pnh.param<double>("low_place_height", low_place_height, 0.13);
  pnh.param<double>("high_place_height", high_place_height, 0.2);
  pnh.param<bool>("add_object", attach_arbitrary_object, false);
  pnh.param("plan_final_execution", plan_mode, false);
  pnh.param<bool>("debug", debug, true);
  pnh.param<bool>("pause_for_verification", pause_for_verification, false);

  object_place_pose_debug = pnh.advertise<geometry_msgs::PoseStamped>("object_place_debug", 1);
  place_pose_bin_debug = pnh.advertise<geometry_msgs::PoseStamped>("place_bin_debug", 1);
  place_pose_base_debug = pnh.advertise<geometry_msgs::PoseStamped>("place_base_debug", 1);
  arm_cartesian_cmd_publisher = n.advertise<geometry_msgs::TwistStamped>("/arm_controller/cartesian_twist/command", 1);

  attach_arbitrary_object_client =
      n.serviceClient<manipulation_actions::AttachArbitraryObject>("collision_scene_manager/attach_arbitrary_object");
  attach_closest_object_client = n.serviceClient<std_srvs::Empty>("/collision_scene_manager/attach_closest_object");
  detach_objects_client = n.serviceClient<std_srvs::Empty>("collision_scene_manager/detach_objects");
  toggle_gripper_collisions_client = n.serviceClient<manipulation_actions::ToggleGripperCollisions>
      ("/collision_scene_manager/toggle_gripper_collisions");

  arm_group = new moveit::planning_interface::MoveGroupInterface("arm");
  arm_group->startStateMonitor();

  planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();

  initPickPoses();

  store_object_server.start();
  kit_pick_server.start();
  kit_place_server.start();
}

void KitManipulator::initPickPoses()
{
  kit_pick_poses.resize(2);

  kit_pick_poses[0].header.frame_id = "kit_frame";
  kit_pick_poses[0].pose.position.x = 0.012;
  kit_pick_poses[0].pose.position.y = 0.072;
  kit_pick_poses[0].pose.position.z = 0.149;
  kit_pick_poses[0].pose.orientation.x = 0.413;
  kit_pick_poses[0].pose.orientation.y = 0.367;
  kit_pick_poses[0].pose.orientation.z = -0.601;
  kit_pick_poses[0].pose.orientation.w = 0.578;

  kit_pick_poses[1].header.frame_id = "kit_frame";
  kit_pick_poses[1].pose.position.x = -0.005;
  kit_pick_poses[1].pose.position.y = -0.111;
  kit_pick_poses[1].pose.position.z = 0.134;
  kit_pick_poses[1].pose.orientation.x = -0.304;
  kit_pick_poses[1].pose.orientation.y = 0.326;
  kit_pick_poses[1].pose.orientation.z = 0.642;
  kit_pick_poses[1].pose.orientation.w = 0.623;

//  kit_pick_poses[2].header.frame_id = "kit_frame";
//  kit_pick_poses[2].pose.position.x = -0.023;
//  kit_pick_poses[2].pose.position.y = -0.073;
//  kit_pick_poses[2].pose.position.z = 0.154;
//  kit_pick_poses[2].pose.orientation.x = -0.351;
//  kit_pick_poses[2].pose.orientation.y = 0.399;
//  kit_pick_poses[2].pose.orientation.z = 0.536;
//  kit_pick_poses[2].pose.orientation.w = 0.656;

  kit_place_poses.resize(kit_pick_poses.size());

  for (size_t i = 0; i < kit_place_poses.size(); i ++)
  {
    kit_place_poses[i].name.push_back("shoulder_pan_joint");
    kit_place_poses[i].name.push_back("shoulder_lift_joint");
    kit_place_poses[i].name.push_back("upperarm_roll_joint");
    kit_place_poses[i].name.push_back("elbow_flex_joint");
    kit_place_poses[i].name.push_back("forearm_roll_joint");
    kit_place_poses[i].name.push_back("wrist_flex_joint");
    kit_place_poses[i].name.push_back("wrist_roll_joint");
  }

  kit_place_poses[0].position.push_back(-0.94);
  kit_place_poses[0].position.push_back(0.76);
  kit_place_poses[0].position.push_back(-2.57);
  kit_place_poses[0].position.push_back(-2.24);
  kit_place_poses[0].position.push_back(0.44);
  kit_place_poses[0].position.push_back(1.95);
  kit_place_poses[0].position.push_back(-0.23);

  kit_place_poses[1].position.push_back(-1.30);
  kit_place_poses[1].position.push_back(0.11);
  kit_place_poses[1].position.push_back(0.46);
  kit_place_poses[1].position.push_back(2.19);
  kit_place_poses[1].position.push_back(-2.41);
  kit_place_poses[1].position.push_back(0.64);
  kit_place_poses[1].position.push_back(-0.36);

//  kit_place_poses[2].position.push_back(0);
//  kit_place_poses[2].position.push_back(0);
//  kit_place_poses[2].position.push_back(0);
//  kit_place_poses[2].position.push_back(0);
//  kit_place_poses[2].position.push_back(0);
//  kit_place_poses[2].position.push_back(0);
//  kit_place_poses[2].position.push_back(0);

  current_grasp_pose = 0;
}

void KitManipulator::executeKitPick(const manipulation_actions::KitManipGoalConstPtr &goal)
{
  manipulation_actions::KitManipResult result;
  bool grasp_succeeded = false;
  bool approach_succeeded = false;

  geometry_msgs::PoseStamped kit_goal_pose; //, grasp_pose;
  geometry_msgs::PoseStamped kit_approach_pose;
  //string arm_group_reference_frame = arm_group->getPoseReferenceFrame();

  for (size_t i = 0; i < kit_pick_poses.size(); i ++)
  {
    current_grasp_pose = i;

    // preset grasp pose calculated on kit frame
    kit_goal_pose = kit_pick_poses[i];
    /*if (kit_goal_pose.header.frame_id != arm_group_reference_frame)
    {
      grasp_pose.header.stamp = ros::Time(0);
      grasp_pose.header.frame_id = arm_group_reference_frame;
      geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(arm_group_reference_frame,
                                                                            kit_goal_pose.header.frame_id,
                                                                            ros::Time(0),
                                                                            ros::Duration(1.0));
      tf2::doTransform(kit_goal_pose, grasp_pose, transform);
    }
    else
    {
      grasp_pose = kit_goal_pose;
      }*/

    ros::Time current_time = ros::Time::now();
    geometry_msgs::TransformStamped grasp_transform;
    grasp_transform.child_frame_id = "grasp_frame";
    grasp_transform.header.frame_id = kit_goal_pose.header.frame_id;
    grasp_transform.header.stamp = current_time;
    grasp_transform.transform.translation.x = kit_goal_pose.pose.position.x;
    grasp_transform.transform.translation.y = kit_goal_pose.pose.position.y;
    grasp_transform.transform.translation.z = kit_goal_pose.pose.position.z;
    grasp_transform.transform.rotation = kit_goal_pose.pose.orientation;
    tf_broadcaster.sendTransform(grasp_transform);

    /*// preset approach pose calculated above grasp pose; assumes kit frame has a vertical z-axis
    geometry_msgs::PoseStamped grasp_approach_pose;
    grasp_approach_pose.header.frame_id = "grasp_frame";
    grasp_approach_pose.pose.position.x = -0.15;

    geometry_msgs::TransformStamped from_grasp_transform = tf_buffer.lookupTransform(arm_group_reference_frame,
        "grasp_frame", current_time, ros::Duration(3.0));
    kit_approach_pose.header.frame_id = arm_group_reference_frame;
    tf2::doTransform(grasp_approach_pose, kit_approach_pose, from_grasp_transform);
    */
    kit_approach_pose.header = kit_goal_pose.header;
    kit_approach_pose.pose.position.x = kit_goal_pose.pose.position.x;
    kit_approach_pose.pose.position.y = kit_goal_pose.pose.position.y;
    kit_approach_pose.pose.position.z = kit_goal_pose.pose.position.z + 0.15;
    kit_approach_pose.pose.orientation = kit_goal_pose.pose.orientation;

    // plan and move to approach pose
    arm_group->setPlannerId("arm[RRTConnectkConfigDefault]");
    arm_group->setPlanningTime(1.5);
    arm_group->setStartStateToCurrentState();
    arm_group->setPoseTarget(kit_approach_pose, "wrist_roll_link");

    moveit_msgs::MoveItErrorCodes error_code = arm_group->move();
    if (error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
    {
      ROS_INFO("Preempted while moving to approach pose. Will try again");
      result.error_code = manipulation_actions::KitManipResult::PREP_FAILURE;
      kit_pick_server.setAborted(result);
      return;
    } else if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_INFO("Failed to move to approach pose %lu.", i);
      continue;
    } else
    {
      approach_succeeded = true;
    }

    // open gripper
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

    // plan to grasp pose
    toggleGripperCollisions("all_objects", true);

    if (plan_mode)
    {
      // Try planning and replanning a few times before failing
      int max_planning_attempts = 3;
      moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
      bool planning_succeeded = false;
      for (int num_attempts = 0; num_attempts < max_planning_attempts; num_attempts++)
      {
        ROS_INFO("Attempting to plan path to grasp. Attempt: %d/%d",
                 num_attempts + 1, max_planning_attempts);

        // calculate short-distance plan to final grasp pose
        arm_group->setPlannerId("arm[RRTConnectkConfigDefault]");
        arm_group->setPlanningTime(1.5);
        arm_group->setStartStateToCurrentState();
        arm_group->setPoseTarget(kit_goal_pose, "wrist_roll_link");

        moveit::planning_interface::MoveItErrorCode plan_result = arm_group->plan(grasp_plan);
        if (kit_pick_server.isPreemptRequested())
        {
          toggleGripperCollisions("all_objects", false);

          ROS_INFO("Preempted while planning grasp");
          result.error_code = manipulation_actions::KitManipResult::PREP_FAILURE;
          kit_pick_server.setPreempted(result);
          return;
        } else if (plan_result.val != moveit::planning_interface::MoveItErrorCode::SUCCESS
                   && num_attempts >= max_planning_attempts - 1)
        {
          toggleGripperCollisions("all_objects", false);

          ROS_INFO("Could not plan to the final grasp pose, giving up on this pose...");
        } else if (plan_result.val != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
          // Try again
          continue;
        }

        // make sure the plan doesn't do some roundabout RRT thing...
        size_t check_index = static_cast<size_t>(grasp_plan.trajectory_.joint_trajectory.points.size() / 2.0);
        double joint_error_check = 0.0;
        double joint_error_thresh = 0.75;  // TODO: this may need tuning to be more lenient in accepting plans
        for (size_t i = 0; i < grasp_plan.trajectory_.joint_trajectory.joint_names.size(); i++)
        {
          joint_error_check += fabs(grasp_plan.trajectory_.joint_trajectory.points[0].positions[i]
                                    - grasp_plan.trajectory_.joint_trajectory.points[check_index].positions[i]);
        }
        ROS_INFO("Joint error check on final grasp trajectory: %f", joint_error_check);

        if (kit_pick_server.isPreemptRequested())
        {
          toggleGripperCollisions("all_objects", false);

          ROS_INFO("Preempted while planning grasp");
          result.error_code = manipulation_actions::KitManipResult::PREP_FAILURE;
          kit_pick_server.setPreempted(result);
          return;
        } else if (joint_error_check > joint_error_thresh && num_attempts >= max_planning_attempts - 1)
        {
          toggleGripperCollisions("all_objects", false);

          ROS_INFO("Could not safely plan to the final grasp pose, giving up on this pose...");
        } else if (joint_error_check <= joint_error_thresh)
        {
          planning_succeeded = true;
          // This is valid. Exit the loop!
          break;
        }
      }

      if (!planning_succeeded)  // move to a new pose if planning failed
      {
        toggleGripperCollisions("all_objects", false);
        result.error_code = manipulation_actions::KitManipResult::PREP_FAILURE;
        continue;
      }

      // execute grasp plan
      error_code = arm_group->execute(grasp_plan);
      if (error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
      {
        toggleGripperCollisions("all_objects", false);

        ROS_INFO("Preempted while moving to final grasp pose.");
        result.error_code = manipulation_actions::KitManipResult::EXECUTION_FAILURE;
        kit_pick_server.setAborted(result);
        return;
      }
      else if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
      {
        toggleGripperCollisions("all_objects", false);

        result.error_code = manipulation_actions::KitManipResult::EXECUTION_FAILURE;
        ROS_INFO("Failed to move to final grasp pose, giving up on this pose...");
        continue;
      } else
      {
        grasp_succeeded = true;
        break;
      }
    }
    else
    {
      // execute with the linear controller
      manipulation_actions::LinearMoveGoal grasp_goal;
      grasp_goal.hold_final_pose = false;

      geometry_msgs::PoseStamped grasp_pose_base;
      grasp_pose_base.header.stamp = ros::Time(0);
      grasp_pose_base.header.frame_id = "base_link";

      // linear controller requires goals to be in the base_link frame
      if (kit_goal_pose.header.frame_id != "base_link")
      {
        geometry_msgs::TransformStamped grasp_to_base_transform = tf_buffer.lookupTransform("base_link",
                                                                                            kit_goal_pose.header.frame_id,
                                                                                            ros::Time(0),
                                                                                            ros::Duration(1.0));
        tf2::doTransform(kit_goal_pose, grasp_pose_base, grasp_to_base_transform);
//        grasp_goal.point.x = grasp_pose_base.pose.position.x;
//        grasp_goal.point.y = grasp_pose_base.pose.position.y;
//        grasp_goal.point.z = grasp_pose_base.pose.position.z;
      }
      else
      {
        grasp_pose_base.pose.position.x = kit_goal_pose.pose.position.x;
        grasp_pose_base.pose.position.y = kit_goal_pose.pose.position.y;
        grasp_pose_base.pose.position.z = kit_goal_pose.pose.position.z;
        grasp_pose_base.pose.orientation = kit_goal_pose.pose.orientation;

//        grasp_goal.point.x = kit_goal_pose.pose.position.x;
//        grasp_goal.point.y = kit_goal_pose.pose.position.y;
//        grasp_goal.point.z = kit_goal_pose.pose.position.z;
      }

      // move pose from wrist_roll_link to gripper_link
      tf2::Transform transform;
      tf2::Quaternion transform_q;
      tf2::fromMsg(grasp_pose_base.pose.orientation, transform_q);
      transform.setRotation(transform_q);
      tf2::Vector3 transform_t;
      tf2::fromMsg(grasp_pose_base.pose.position, transform_t);
      transform.setOrigin(transform_t);
      tf2::Vector3 transform_point, transformed_point;
      transform_point.setX(.166);  // TODO: this shifts from the wrist_roll_link to the gripper_link, which should be a lookup instead of hardcoded
      transform_point.setY(0);
      transform_point.setZ(0);

      transformed_point = transform.inverse() * transform_point;
      tf2::toMsg(transformed_point, grasp_goal.point);

      geometry_msgs::PoseStamped debug_pose;
      debug_pose.header.frame_id = "base_link";
      debug_pose.pose.orientation = grasp_pose_base.pose.orientation;
      debug_pose.pose.position = grasp_goal.point;
      object_place_pose_debug.publish(debug_pose);
//      Eigen::Affine3d transform_matrix;
//      transform.rotation = grasp_pose.orientation;
//      transform.translation.x = grasp_pose.position.x;
//      transform.translation.y = grasp_pose.position.y;
//      transform.translation.z = grasp_pose.position.z;
//      tf2::transformMsgToEigen(transform, transform_matrix);

//      Eigen::Vector3d transform_point, transformed_point;
//      transform_point[0] = .166;
//      transform_point[1] = 0;
//      transform_point[2] = 0;
//      transformed_point = transform_matrix*transform_point;
//      tf::pointEigenToMsg(transformed_point, result.position);

      ROS_INFO("Skipping execution for debugging purposes...");
      ros::Duration(5.0).sleep();
      grasp_succeeded = true;
      break;

      linear_move_client.sendGoal(grasp_goal);
      linear_move_client.waitForResult(ros::Duration(5.0));
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
  }

  if (!approach_succeeded)
  {
    result.error_code = manipulation_actions::KitManipResult::PREP_FAILURE;
    kit_pick_server.setAborted(result);
    return;
  }

  if (!grasp_succeeded)
  {
    kit_pick_server.setAborted(result);
    return;
  }

  //close gripper
  control_msgs::GripperCommandGoal close_goal;
  close_goal.command.position = 0;
  close_goal.command.max_effort = 100;
  gripper_client.sendGoal(close_goal);
  gripper_client.waitForResult(ros::Duration(5.0));

  // attach nearby object if there was a specific object to pick.
  std_srvs::Empty attach_object_srv;
  if (!attach_closest_object_client.call(attach_object_srv))
  {
    ROS_INFO("Failed to attach an object to the gripper");
  }
  else
  {
    ROS_INFO("Picked object attached to the gripper");
  }

  // sleep to allow the attachments to propagate
  ros::Duration(0.2).sleep();

  // reenable collisions on the gripper
  toggleGripperCollisions("all_objects", false);

  // do a short arm raise with a Cartesian command
  geometry_msgs::TwistStamped raise_cmd;
  raise_cmd.header.frame_id = "base_link";
  raise_cmd.twist.linear.z = 0.5;
  ros::Rate vel_publish_freq(30);

  ros::Time start_time = ros::Time::now();
  while (ros::Time::now() < start_time + ros::Duration(1.0))
  {
    raise_cmd.header.stamp = ros::Time::now();
    arm_cartesian_cmd_publisher.publish(raise_cmd);

    // sleep
    ros::spinOnce();
    vel_publish_freq.sleep();
  }

  // publish stop arm command
  raise_cmd.twist.linear.z = 0.0;
  arm_cartesian_cmd_publisher.publish(raise_cmd);

  // TODO (enhancement): replace this with a Cartesian velocity command to the fetch?
//  arm_group->setStartStateToCurrentState();
//  arm_group->setPoseTarget(kit_approach_pose, "wrist_roll_link");
//  moveit_msgs::MoveItErrorCodes error_code = arm_group->move();
//  if (error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
//  {
//    ROS_INFO("Preempted while picking up; action still considered successful.");
//  }
//  else if (result.error_code != moveit_msgs::MoveItErrorCodes::SUCCESS)
//  {
//    ROS_INFO("Failed to pick up; action still considered successful.");
//  }

  result.error_code = manipulation_actions::KitManipResult::SUCCESS;
  kit_pick_server.setSucceeded(result);
}

bool KitManipulator::toggleGripperCollisions(std::string object, bool allow_collisions)
{
  manipulation_actions::ToggleGripperCollisions toggle_gripper_collisions_srv;
  toggle_gripper_collisions_srv.request.enable_collisions = allow_collisions;
  toggle_gripper_collisions_srv.request.object_name = object;

  bool return_state = toggle_gripper_collisions_client.call(toggle_gripper_collisions_srv);
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

void KitManipulator::executeKitPlace(const manipulation_actions::KitManipGoalConstPtr &goal)
{
  manipulation_actions::KitManipResult result;

  arm_group->setPlannerId("arm[RRTConnectkConfigDefault]");
  arm_group->setPlanningTime(7.0);
  arm_group->setStartStateToCurrentState();
  arm_group->setJointValueTarget(kit_place_poses[current_grasp_pose]);

  // Plan and execute pose
  moveit_msgs::MoveItErrorCodes error_code = arm_group->move();
  if (error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    ROS_INFO("Preempted while moving to place pose.");
    result.error_code = manipulation_actions::KitManipResult::EXECUTION_FAILURE;
    kit_place_server.setAborted(result);
    return;
  }
  else if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("Failed to move to place pose.");
    result.error_code = manipulation_actions::KitManipResult::SUCCESS;
    kit_place_server.setAborted(result);
    return;
  }

  // open gripper
  control_msgs::GripperCommandGoal gripper_goal;
  gripper_goal.command.position = 0.1;
  gripper_goal.command.max_effort = 200;
  gripper_client.sendGoal(gripper_goal);
  gripper_client.waitForResult(ros::Duration(5.0));

  // detach object(s) in gripper
  std_srvs::Empty detach_srv;
  if (!detach_objects_client.call(detach_srv))
  {
    ROS_INFO("Could not call moveit collision scene manager service!");
  }

  ROS_INFO("Kit placed on base.");
  result.error_code = manipulation_actions::KitManipResult::SUCCESS;
  kit_place_server.setSucceeded(result);
}

void KitManipulator::executeStore(const manipulation_actions::StoreObjectGoalConstPtr &goal)
{
  manipulation_actions::StoreObjectResult result;

  if (attach_arbitrary_object)
  {
    // add the largest arbitrary object to planning scene for testing (typically this would be done at grasp time)
    manipulation_actions::AttachArbitraryObject attach_srv;
    if (goal->challenge_object.object == manipulation_actions::ChallengeObject::BOLT)
    {
      attach_srv.request.challenge_object.object = manipulation_actions::ChallengeObject::BOLT;
    }
    else if (goal->challenge_object.object == manipulation_actions::ChallengeObject::SMALL_GEAR)
    {
      attach_srv.request.challenge_object.object = manipulation_actions::ChallengeObject::SMALL_GEAR;
    }
    else if (goal->challenge_object.object == manipulation_actions::ChallengeObject::LARGE_GEAR)
    {
      attach_srv.request.challenge_object.object = manipulation_actions::ChallengeObject::LARGE_GEAR;
    }
    else if (goal->challenge_object.object == manipulation_actions::ChallengeObject::GEARBOX_TOP)
    {
      attach_srv.request.challenge_object.object = manipulation_actions::ChallengeObject::GEARBOX_TOP;
    }
    else if (goal->challenge_object.object == manipulation_actions::ChallengeObject::GEARBOX_BOTTOM)
    {
      attach_srv.request.challenge_object.object = manipulation_actions::ChallengeObject::GEARBOX_BOTTOM;
    }

    if (!attach_arbitrary_object_client.call(attach_srv))
    {
      ROS_INFO("Could not call moveit collision scene manager service!");
    }
  }

  // Consider multiple poses and order them based on which will be most likely to cleanly drop the object
  // (i.e. gripper pointing down)

  vector<ScoredPose> sorted_place_poses;
  sorted_place_poses.clear();

  geometry_msgs::PoseStamped object_pose;
  geometry_msgs::PoseStamped place_pose_bin;
  geometry_msgs::PoseStamped place_pose_base;
  object_pose.header.frame_id = "kit_frame";
  object_pose.pose.orientation.w = 1.0;
  object_pose.pose.position.z += low_place_height;

  if (goal->challenge_object.object == manipulation_actions::ChallengeObject::BOLT)
  {
    object_pose.pose.position.x += 0.05;
    object_pose.pose.position.y += 0.05;
  }
  else if (goal->challenge_object.object == manipulation_actions::ChallengeObject::SMALL_GEAR
           || goal->challenge_object.object == manipulation_actions::ChallengeObject::LARGE_GEAR)
  {
    object_pose.pose.position.x += 0.05;
    object_pose.pose.position.y -= 0.05;
  }
  else if (goal->challenge_object.object == manipulation_actions::ChallengeObject::GEARBOX_TOP
           || goal->challenge_object.object == manipulation_actions::ChallengeObject::GEARBOX_BOTTOM)
  {
    object_pose.pose.position.x -= 0.05;
  }

  geometry_msgs::TransformStamped object_to_wrist = tf_buffer.lookupTransform("object_frame", "wrist_roll_link",
                                                                              ros::Time(0), ros::Duration(1.0));
  tf2::Transform object_to_wrist_tf;
  tf2::fromMsg(object_to_wrist.transform, object_to_wrist_tf);

  for (int i = 0; i < 16; i ++)
  {
    for (int j = 0; j < 2; j ++)
    {
      geometry_msgs::PoseStamped pose_candidate;
      tf2::Transform place_object_tf;
      tf2::fromMsg(object_pose.pose, place_object_tf);

      // 90 degree rotation about z-axis to align with kit_frame y-axis + optional 180 degree rotation about z-axis
      // to cover all valid alignments of the object_frame's x-axis pose to the kit_frame's y-axis alignments
      tf2::Quaternion initial_adjustment;
      initial_adjustment.setRPY(0, 0, M_PI_2 + j*M_PI);

      place_object_tf.setRotation(place_object_tf.getRotation() * initial_adjustment);

      if (pause_for_verification)
      {
        geometry_msgs::PoseStamped pose_candidate_initial;
        pose_candidate_initial.header.frame_id = object_pose.header.frame_id;
        pose_candidate_initial.pose.position.x = place_object_tf.getOrigin().x();
        pose_candidate_initial.pose.position.y = place_object_tf.getOrigin().y();
        pose_candidate_initial.pose.position.z = place_object_tf.getOrigin().z();
        pose_candidate_initial.pose.orientation = tf2::toMsg(place_object_tf.getRotation());

        object_place_pose_debug.publish(pose_candidate_initial);
      }

      // special case objects (large gear doesn't fit in it's compartment unless it's standing upright)
      tf2::Quaternion special_case_adjustment;
      special_case_adjustment.setRPY(0, 0, 0);
      if (goal->challenge_object.object == manipulation_actions::ChallengeObject::LARGE_GEAR)
      {
        special_case_adjustment.setRPY(0, -M_PI_2, 0);
      }

      // rotate pose around x-axis to generate candidates (longest axis, which most constrains place)
      tf2::Quaternion adjustment;
      adjustment.setRPY(i * M_PI/8.0, 0, 0);
      place_object_tf.setRotation(place_object_tf.getRotation()
                                  * special_case_adjustment * adjustment);

      if (pause_for_verification)
      {
        geometry_msgs::PoseStamped pose_candidate_base;
        pose_candidate_base.header.frame_id = object_pose.header.frame_id;
        pose_candidate_base.pose.position.x = place_object_tf.getOrigin().x();
        pose_candidate_base.pose.position.y = place_object_tf.getOrigin().y();
        pose_candidate_base.pose.position.z = place_object_tf.getOrigin().z();
        pose_candidate_base.pose.orientation = tf2::toMsg(place_object_tf.getRotation());
        place_pose_bin_debug.publish(pose_candidate_base);
      }

      // determine wrist frame pose that will give the desired grasp
      tf2::Transform place_candidate_tf;
      place_candidate_tf = place_object_tf * object_to_wrist_tf;

      // scoring with respect to "downward pointing"
      tf2::Vector3 gravity_vector(0, 0, -1);
      tf2::Matrix3x3 rotation_mat(place_candidate_tf.getRotation());
      tf2::Vector3 x_vector(1, 0, 0);
      tf2::Vector3 pose_x_vector = rotation_mat * x_vector;

      double score = acos(pose_x_vector.dot(gravity_vector));

      if (score < M_PI / 3.0)
      {
        pose_candidate.header.frame_id = object_pose.header.frame_id;
        pose_candidate.pose.position.x = place_candidate_tf.getOrigin().x();
        pose_candidate.pose.position.y = place_candidate_tf.getOrigin().y();
        pose_candidate.pose.position.z = place_candidate_tf.getOrigin().z();
        pose_candidate.pose.orientation = tf2::toMsg(place_candidate_tf.getRotation());

        if (pause_for_verification)
        {
          place_pose_base_debug.publish(pose_candidate);
        }

        sorted_place_poses.emplace_back(ScoredPose(pose_candidate, score));

        if (pause_for_verification)
        {
          std::cout << "user input? (enter any non-empty string to continue)" << std::endl;
          string str;
          std::cin >> str;
        }
      }
    }
  }

  // sort poses
  sort(sorted_place_poses.begin(), sorted_place_poses.end());

  // execute best executable pose
  geometry_msgs::TransformStamped bin_to_base = tf_buffer.lookupTransform("base_link", "kit_frame",
                                                                          ros::Time(0), ros::Duration(1.0));
  bool execution_failed = true;
  double lower_height = low_place_height;
  for (size_t attempt = 0; attempt < 2; attempt ++)
  {
    for (size_t i = 0; i < sorted_place_poses.size(); i++)
    {
      place_pose_base.header.frame_id = "base_link";
      tf2::doTransform(sorted_place_poses[i].pose, place_pose_base, bin_to_base);
      place_pose_base.header.frame_id = "base_link";
      place_pose_base.pose.position.z += attempt*(high_place_height - low_place_height);

      place_pose_base_debug.publish(place_pose_base);

      ROS_INFO("Moving to place pose...");
      arm_group->setPlannerId("arm[RRTConnectkConfigDefault]");
      arm_group->setPlanningTime(1.5);
      arm_group->setStartStateToCurrentState();
      arm_group->setPoseTarget(place_pose_base);

      moveit::planning_interface::MoveItErrorCode move_result = arm_group->move();
      std::cout << "MoveIt! error code: " << move_result.val << std::endl;
      if (move_result.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
      {
        execution_failed = false;
        lower_height += attempt*(high_place_height - low_place_height);
        break;
      }
      else if (move_result.val == moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE
               || move_result.val == moveit_msgs::MoveItErrorCodes::CONTROL_FAILED
               || move_result.val == moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA
               || move_result.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT
               || move_result.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
      {
        // give the same pose a second chance, this is sometimes just controller failure right at the goal
        arm_group->setStartStateToCurrentState();
        arm_group->setPoseTarget(place_pose_base);

        move_result = arm_group->move();
        std::cout << "Replan MoveIt! error code: " << move_result.val << std::endl;
        if (move_result.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
          execution_failed = false;
          lower_height += attempt*(high_place_height - low_place_height);
          break;
        }
      }
    }
    if (!execution_failed)
    {
      break;
    }
    ROS_INFO("Switching to higher poses...");
  }

  if (execution_failed)
  {
    store_object_server.setAborted(result);
    return;
  }

  if (pause_for_verification)
  {
    if (pause_for_verification)
    {
      std::cout << "user input? (enter any non-empty string to continue) " << std::endl;
      string str;
      std::cin >> str;
    }
  }

  // move down with linear controller
  manipulation_actions::LinearMoveGoal lower_goal;
  manipulation_actions::LinearMoveGoal raise_goal;
  geometry_msgs::TransformStamped gripper_tf = tf_buffer.lookupTransform("base_link", "gripper_link", ros::Time(0),
                                                                         ros::Duration(1.0));
  lower_goal.hold_final_pose = false;
  lower_goal.point.x = gripper_tf.transform.translation.x;
  lower_goal.point.y = gripper_tf.transform.translation.y;
  lower_goal.point.z = gripper_tf.transform.translation.z - lower_height;
  raise_goal.hold_final_pose = false;
  raise_goal.point.x = lower_goal.point.x;
  raise_goal.point.y = lower_goal.point.y;
  raise_goal.point.z = lower_goal.point.z + 0.1;
  linear_move_client.sendGoal(lower_goal);
  linear_move_client.waitForResult(ros::Duration(5.0));
  manipulation_actions::LinearMoveResultConstPtr linear_result = linear_move_client.getResult();

  // open gripper
  control_msgs::GripperCommandGoal gripper_goal;
  gripper_goal.command.position = 0.1;
  gripper_goal.command.max_effort = 200;
  gripper_client.sendGoal(gripper_goal);
  gripper_client.waitForResult(ros::Duration(5.0));

  // detach collision object
  std_srvs::Empty detach_srv;
  if (!detach_objects_client.call(detach_srv))
  {
    ROS_INFO("Could not call moveit collision scene manager service!");
  }

  // raise gripper
  linear_move_client.sendGoal(raise_goal);
  linear_move_client.waitForResult(ros::Duration(5.0));

  store_object_server.setSucceeded(result);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "kit_manipulator");

  KitManipulator km;

  ros::spin();

  return EXIT_SUCCESS;
}
