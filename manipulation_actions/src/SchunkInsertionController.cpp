#include <manipulation_actions/SchunkInsertionController.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
// #include <moveit/robot_model_loader/robot_model_loader.h>

using std::max;

SchunkInsertionController::SchunkInsertionController():
    pnh("~"),
    tf_listener(tf_buffer),
    schunk_insert_server(pnh, "schunk_insert", boost::bind(&SchunkInsertionController::executeInsertion, this, _1), false),
    schunk_pullback_server(pnh, "schunk_pullback", boost::bind(&SchunkInsertionController::executePullback, this, _1), false),
    arm_control_client("arm_controller/follow_joint_trajectory"),
    linear_move_client("schunk_linear_controller/linear_move"),
    gripper_control_client("gripper_controller/gripper_action")
    // TODO Remove
    // loader("robot_description")
{

  // Setup parameters. TODO: Velocity should be 0.05
  pnh.param<int>("command_rate", command_rate, 50); // identify the ideal rate to run the controller
  pnh.param<double>("max_force", max_force, 22); // identify the ideal threshold for detecting collision
  pnh.param<double>("insert_duration", insert_duration, 3); // find out the ideal duration
  pnh.param<double>("insert_tol", insert_tol, 0.14); // identify the ideal tolerance for detection insertion
  pnh.param<int>("num_trial_max", num_trial_max, 10); // identify the ideal num of trails
  pnh.param<double>("reset_duration", reset_duration, insert_duration); // find out the ideal duration
  pnh.param<double>("search_dist", search_dist, 0.02); // distance for circular search
  pnh.param<bool>("linear_hold_pos", linear_hold_pos, false); // have linear controller hold position or not
  pnh.param<double>("gripper_closed_value", gripper_closed_value, 0.005);


  jnt_goal.trajectory.joint_names.push_back("shoulder_pan_joint");
  jnt_goal.trajectory.joint_names.push_back("shoulder_lift_joint");
  jnt_goal.trajectory.joint_names.push_back("upperarm_roll_joint");
  jnt_goal.trajectory.joint_names.push_back("elbow_flex_joint");
  jnt_goal.trajectory.joint_names.push_back("forearm_roll_joint");
  jnt_goal.trajectory.joint_names.push_back("wrist_flex_joint");
  jnt_goal.trajectory.joint_names.push_back("wrist_roll_joint");
  jnt_goal.trajectory.points.resize(1);


  // TODO Remove
  // // initialize vectors
  // eef_force_.emplace_back(0);
  // eef_force_.emplace_back(0);
  // eef_force_.emplace_back(0);
  // base_eef_force_.emplace_back(0);
  // base_eef_force_.emplace_back(0);
  // base_eef_force_.emplace_back(0);

  // joint_states subscriber to get feedback on effort
  joint_states_subscriber = n.subscribe("joint_states", 1, &SchunkInsertionController::jointStatesCallback, this);

  // cart_twist publisher to send command to the controller
  cart_twist_cmd_publisher = n.advertise<geometry_msgs::TwistStamped>("/arm_controller/cartesian_twist/command", 1);

  // Setup collision scene clearing service client
  CollisionSceneClient = n.serviceClient<std_srvs::Empty>("collision_scene_manager/detach_objects");

  // // Setup MoveIt
  // kinematic_model = loader.getModel();
  // robot_state::RobotStatePtr temp_kinematic_state(new robot_state::RobotState(kinematic_model));
  // joint_model_group = kinematic_model->getJointModelGroup("arm");
  // kinematic_state = temp_kinematic_state;

  // ROS_INFO("Completed MoveIt setup");


  // Set delta theta for circular search
  search_delta_theta = 2 * PI / (num_trial_max - 2);

  // Start controller
  schunk_insert_server.start();
  ROS_INFO("schunk_insert_server started");


  schunk_pullback_server.start();
  ROS_INFO("schunk_pullback_server started");
}

void SchunkInsertionController::jointStatesCallback(const sensor_msgs::JointState &msg)
{
    boost::mutex::scoped_lock lock(joint_states_mutex);
    if (msg.position.size() > 3)
    {
      joint_states = msg;
    }
}

void SchunkInsertionController::executeInsertion(const manipulation_actions::SchunkInsertGoalConstPtr& goal)
{
  ROS_INFO("Settng up schunk insertion execution...");

  manipulation_actions::SchunkInsertResult result;


  // setup command messages
  geometry_msgs::TwistStamped cmd;
  cmd.header.frame_id = "end_effector_frame";

  // get the transform for large gear
  ROS_INFO("Transforming command to end-effector frame");
  geometry_msgs::TransformStamped object_transform_msg = tf_buffer.lookupTransform("gripper_link", "object_frame", ros::Time(0), ros::Duration(1.0));

  tf2::Transform object_tf;
  tf2::fromMsg(object_transform_msg.transform,object_tf);


  ROS_INFO("Getting gripper location in object frame");
  geometry_msgs::TransformStamped gripper_in_object_tf_msg = tf_buffer.lookupTransform("object_frame", "gripper_link", ros::Time(0), ros::Duration(1.0));
  // get the gear to gripper linear position offset
  geometry_msgs::Vector3 object_gripper_offset = gripper_in_object_tf_msg.transform.translation;


  // get the transform from initial "object_frame" to "base_link"
  ROS_INFO("Getting TF from object_frame to base_link");
  geometry_msgs::TransformStamped object_to_base_transform_msg = tf_buffer.lookupTransform("base_link", "object_frame", ros::Time(0), ros::Duration(1.0));

  tf2::Transform object_to_base_tf;
  tf2::fromMsg(object_to_base_transform_msg.transform, object_to_base_tf);


  // Convert desired velocity from object frame to end effector frame
  object_twist_goal_msg.x = goal->object_twist_goal.twist.linear.x;
  object_twist_goal_msg.y = goal->object_twist_goal.twist.linear.y;
  object_twist_goal_msg.z = goal->object_twist_goal.twist.linear.z;

  tf2::fromMsg(object_twist_goal_msg,object_twist_goal);

  eef_twist_goal = object_tf * object_twist_goal;

  cmd.twist.linear = tf2::toMsg(eef_twist_goal);

  ros::Rate controller_rate(command_rate);

  // Save initial configuration and eef position
  {
    boost::mutex::scoped_lock lock(joint_states_mutex);
    jnt_pos_start = joint_states.position;
  }
  ROS_INFO("Saving initial joint configuratin on [%f, %f, %f, %f, %f, %f, %f]", jnt_pos_start[6], jnt_pos_start[7], jnt_pos_start[8],
            jnt_pos_start[9], jnt_pos_start[10], jnt_pos_start[11], jnt_pos_start[12]);
  jnt_goal.trajectory.points.resize(1);
  jnt_goal.trajectory.points[0].positions.clear();
  jnt_goal.trajectory.points[0].time_from_start = ros::Duration(reset_duration);

  for (size_t i = 6; i < 6 + jnt_goal.trajectory.joint_names.size(); i ++)
  {
    jnt_goal.trajectory.points[0].positions.push_back(jnt_pos_start[i]);
  }

  // Start insertion attempts
  bool success = false;

  ROS_INFO("Starting insertion attempts...");

  for (unsigned int k = 0 ; k < num_trial_max ; ++k)
  {
    ros::Duration(0.5).sleep();
    cmd.twist.linear = tf2::toMsg(eef_twist_goal);

    // Compute interaction forces
    updateJointEffort(); // This updates jnt_eff_

    //TODO remove
    // updateJacobian(); // This updates jacobian_


    // TODO Remove
    // std::cout << "Base EEF force jacobian: " << std::endl << jacobian_ << std::endl;

    // TODO Remove
    // Calculate the eef force at the start to setup an offset
    // double base_joint_norm = 0.0;
    // double base_force_total_norm = 0.0;
    // std::cout << "Joint efforts: " << std::endl;
    // for (unsigned int i = 0 ; i < 3 ; ++i)
    // {
    //   base_eef_force_[i] = 0;
    //   for (unsigned int j = 0 ; j < 7; ++j)
    //   {
    //     base_eef_force_[i] += jacobian_(i,j) * jnt_eff_[j + 6];
    //     if (i == 0)
    //     {
    //       base_joint_norm += pow(jnt_eff_[j + 6], 2);
	  // std::cout << j << ": " << jnt_eff_[j + 6] << std::endl;
    //     }
    //   }
    //   base_force_total_norm += pow(base_eef_force_[i], 2);
    // }
    // ROS_INFO("Base EEF Force (x, y, z, norm): %f, %f, %f, %f\n", base_eef_force_[0], base_eef_force_[1], base_eef_force_[2], base_joint_norm);

    // TODO Remove
    // std::cout << "Base EEF force jacobian: " << std::endl << jacobian_ << std::endl;

    ros::Time end_time = ros::Time::now() + ros::Duration(insert_duration);

    geometry_msgs::TransformStamped gripper_transform_start_msg = tf_buffer.lookupTransform("base_link", "gripper_link",
                                                                                        ros::Time(0), ros::Duration(1.0));
    gripper_pos_start = gripper_transform_start_msg.transform.translation; // extract only the position
    ROS_INFO("Initial gripper_link position saved!");

    ROS_INFO("Attempt #%u of %d", k+1, num_trial_max);

    // TODO Remove
    // // Set initial eef_force to zero
    // eef_force_[0] = 0;
    // eef_force_[1] = 0;
    // eef_force_[2] = 0;
    geometry_msgs::TransformStamped gripper_transform_end_msg;
    while (ros::Time::now() < end_time)
    {

      // TODO Remove
      // if (!(!isnan(base_eef_force_[0]) && !isnan(base_eef_force_[1]) && !isnan(base_eef_force_[2]))) {
      //   ROS_INFO("eef force feedback has NaNs. Check Jacobian.");
      //   break;
      // }
      // if (base_eef_force_[0] == 0 && base_eef_force_[1] == 0 && base_eef_force_[0] == 0) {
      //   ROS_INFO("eef force feedback is all zeros. Check Jacobian.");
      //   break;
      // }

      // TODO Remove or replace
      //       if (!(fabs(eef_force_[0]) < max_force && fabs(eef_force_[1]) < max_force
      //           && fabs(eef_force_[2]) < max_force))
      //       {
      // //        ROS_INFO("Force feedback exceeded!");
      //           ROS_INFO(" ");
      // //        break;
      //       }

      // Publish the command
      cart_twist_cmd_publisher.publish(cmd);

      // Compute interaction forces
      updateJointEffort(); // This updates jnt_eff_

      // TODO Remove
      // updateJacobian(); // This updates jacobian_

      // float joint_norm = 0;
      // double force_total_norm = 0.0;
      // double force_diff = 0.0;
      // for (unsigned int i = 0 ; i < 3 ; ++i)
      // {
      //   eef_force_[i] = 0;
      //   for (unsigned int j = 0 ; j < 7; ++j)
      //   {
      //     eef_force_[i] += jacobian_(i,j) * jnt_eff_[j + 6];
      //     if (i == 0)
      //     {
      //       joint_norm += pow(jnt_eff_[j + 6], 2);
      //     }
	    //   }
	    //   force_total_norm += pow(eef_force_[i], 2);
      //   force_diff += pow(base_eef_force_[i] - eef_force_[i], 2);
      // }

      ROS_INFO("Shoulder pan joint force: %f", jnt_eff_[6]);

      if (jnt_eff_[6] > max_force) {
        ROS_INFO("Shoulder pan joint effort exceeded 22 threshold");
	      break;
      }


      // DEBUG

      // if (force_diff > max_force) {
	    //   if (force_total_norm > base_force_total_norm) {
	    //   	ROS_INFO("Force exceeded!");
	    //   	break;
	    //   }
	    //   else if (force_diff > 900.0) {
      //     ROS_INFO("Force exceeded second threshold");
      //     break;
	    //   }
      // }

      // ROS_INFO("Checking for success...");
      gripper_transform_end_msg = tf_buffer.lookupTransform("base_link", "gripper_link", ros::Time(0), ros::Duration(1.0)); // get updated gripper_link location
      gripper_pos_end = gripper_transform_end_msg.transform.translation; // extract only the position

      // Calculate euclidian distance between gripper_pos_start and gripper_pos_end
      travel_dist = sqrt(pow(gripper_pos_end.x - gripper_pos_start.x, 2) + pow(gripper_pos_end.y - gripper_pos_start.y, 2) + pow(gripper_pos_end.z - gripper_pos_start.z, 2));

      if (travel_dist > insert_tol)
      {
        ROS_INFO("Insertion succeeded!");
        success = true;

        cmd.twist.linear.x = 0;
        cmd.twist.linear.y = 0;
        cmd.twist.linear.z = 0;
        cart_twist_cmd_publisher.publish(cmd);
        ros::Duration(0.5).sleep();

        break;
      }

      // Check for preempt
      if (schunk_insert_server.isPreemptRequested())
      {
        schunk_insert_server.setPreempted(result);
        return;
      }
      controller_rate.sleep();
    }

    ROS_INFO("Moved %f distance in.", travel_dist);

    //debug
   // travel_dist = 0.0;

    if (success)
    {
      // ROS_INFO("Insertion succeeded!");
      k = num_trial_max; // end attempts if successful


      // wait for 2 seconds and check for drift backwards
      ros::Duration(2.0).sleep();

      geometry_msgs::TransformStamped gripper_transform_end_msg_check = tf_buffer.lookupTransform("base_link", "gripper_link", ros::Time(0), ros::Duration(1.0)); // get updated gripper_link location
      geometry_msgs::Vector3 gripper_pos_end_check = gripper_transform_end_msg_check.transform.translation; // extract only the position

      // Calculate euclidian distance between gripper_pos_end and gripper_pos_end_check
      travel_dist = sqrt(pow(gripper_pos_end.x - gripper_pos_end_check.x, 2) + pow(gripper_pos_end.y - gripper_pos_end_check.y, 2) + pow(gripper_pos_end.z - gripper_pos_end_check.z, 2));

      // std::cout<<gripper_pos_end<<std::endl;
      // std::cout<<gripper_pos_end_check<<std::endl;
      // std::cout<<travel_dist<<std::endl;
      // if drift greater than 1 cm
      if(travel_dist > 0.01) {
        ROS_ERROR("Insertion failed... drift of %f detected. Recovery required.", travel_dist);
        success = false;
      } else {
        ROS_INFO("Inserted successfully! ");
      }
    }
    else
    {
      ROS_INFO("Insertion Failed!");

      cmd.twist.linear.x = 0;
      cmd.twist.linear.y = 0;
      cmd.twist.linear.z = 0;
      cart_twist_cmd_publisher.publish(cmd);

      ros::Duration(0.5).sleep();

      // reset the arm to the starting point
      ROS_INFO("resetting arm to original starting point...");
      arm_control_client.sendGoal(jnt_goal);
      arm_control_client.waitForResult();
      //      auto arm_controller_status = arm_control_client.getState();
      //      auto arm_controller_result = arm_control_client.getResult();
      //      ROS_INFO_STREAM("tests" << arm_controller_result->error_code << " - " << arm_controller_result->error_string);
      //      ROS_INFO("arm controller reported a status of %d...", arm_controller_status.state_);

      ros::Duration(0.5).sleep();

      // If last attempt failed, go to recovery...
      if (k == num_trial_max - 1)
      {
        ROS_INFO("All attempts failed...");
        ROS_INFO("Move to recovery.");
      }


      // // Use linear controller to reset to start position
      // else if (k == 0)
      // {
      //   ROS_INFO("Moving back to initial start position with linear controller for second attempt...");

      //   linear_goal.point.x = gripper_pos_start.x;
      //   linear_goal.point.y = gripper_pos_start.y;
      //   linear_goal.point.z = gripper_pos_start.z;
      //   linear_goal.hold_final_pose = true;

      //   ROS_INFO("Moving gripper_link to (x,y,z in base_link): (%f, %f, %f)", gripper_pos_start.x, gripper_pos_start.y, gripper_pos_start.z);

      //   linear_move_client.sendGoal(linear_goal);
      //   linear_move_client.waitForResult();

      //   // Just for debug info; can be removed once tested and verified.
      //   geometry_msgs::TransformStamped gripper_transform_end_msg_2 = tf_buffer.lookupTransform("base_link", "gripper_link",
      //       ros::Time(0), ros::Duration(1.0)); // get updated object_frame location
      //   gripper_pos_reset = gripper_transform_end_msg_2.transform.translation; // extract only the position
      //   ROS_INFO("Moved gripper_link to (x,y,z in base_link): (%f, %f, %f)", gripper_pos_reset.x, gripper_pos_reset.y, gripper_pos_reset.z);

      // }

      else if (k > 0)
      {
        // verify that the object is still in the gripper before we continue to a new pose
        fetch_driver_msgs::GripperStateConstPtr gripper_state =
          ros::topic::waitForMessage<fetch_driver_msgs::GripperState>("/gripper_state", n);
        if (!gripper_state
            || (gripper_state->joints[0].position <= gripper_closed_value && gripper_state->joints[0].effort > 0))
        {
          ROS_INFO("Detected empty gripper. Aborting.");
          schunk_insert_server.setAborted(result);
          return;
        }

        ROS_INFO("Moving to a new starting point...");
        double search_theta = search_delta_theta * (k - 1);
        ROS_INFO("Searching at %f", search_theta);
        double search_y = cos(search_theta) * search_dist;
        double search_z = -sin(search_theta) * search_dist;

        // object_to_base_transform_msg
        object_linear_move_goal = tf2::Vector3(0 + object_gripper_offset.x, search_y + object_gripper_offset.y, search_z + object_gripper_offset.z);
        base_linear_move_goal = object_to_base_tf * object_linear_move_goal;
        linear_goal.point.x = base_linear_move_goal.x();
        linear_goal.point.y = base_linear_move_goal.y();
        linear_goal.point.z = base_linear_move_goal.z();
        linear_goal.hold_final_pose = linear_hold_pos;

        ROS_INFO("Moving to (x,y,z in object_frame): (%f, %f, %f)", object_linear_move_goal.x(), object_linear_move_goal.y(), object_linear_move_goal.z());
        ROS_INFO("Moving gripper_link to (x,y,z in base_link): (%f, %f, %f)", base_linear_move_goal.x(), base_linear_move_goal.y(), base_linear_move_goal.z());

        linear_move_client.sendGoal(linear_goal);
        linear_move_client.waitForResult();

        geometry_msgs::TransformStamped gripper_transform_end_msg_2 = tf_buffer.lookupTransform("base_link", "object_frame",
            ros::Time(0), ros::Duration(1.0)); // get updated object_frame location
        gripper_pos_end = gripper_transform_end_msg_2.transform.translation; // extract only the position

        ROS_INFO("Moved gripper_link to (x,y,z in base_link): (%f, %f, %f)", gripper_pos_end.x, gripper_pos_end.y, gripper_pos_end.z);
      }
    }
  }

  // Set success
  ROS_INFO("Insertion action complete!");
  if (success) {
    result.success = true;
    schunk_insert_server.setSucceeded(result);
  } else {
    schunk_insert_server.setAborted(result);
  }

}



void SchunkInsertionController::executePullback(const manipulation_actions::SchunkPullbackGoalConstPtr& goal) {

  manipulation_actions::SchunkPullbackResult result;

  if (jnt_goal.trajectory.points[0].positions[0] == 0.0 || isnan(jnt_goal.trajectory.points[0].positions[0])){
    ROS_INFO("ERROR: Joint trajectory not preset...");
    result.success = false;
    schunk_pullback_server.setAborted(result);
  } else {
    ROS_INFO("Opening gripper...");
    gripper_goal.command.position = 0.1;
    gripper_goal.command.max_effort = 200;
    gripper_control_client.sendGoal(gripper_goal);
    gripper_control_client.waitForResult();

    ROS_INFO("Resetting arm to original starting point...");
    arm_control_client.sendGoal(jnt_goal);
    arm_control_client.waitForResult();

    std_srvs::Empty detach_objects_srv;
    if (CollisionSceneClient.call(detach_objects_srv)) {
      result.success = true;
      schunk_pullback_server.setSucceeded(result);
    } else {
      result.success = false;
      schunk_pullback_server.setAborted(result);
    }
  }

}


// TODO Remove
// void SchunkInsertionController::updateJacobian()
// {
//   // Extract joint position
//   boost::mutex::scoped_lock lock(joint_states_mutex);
//   jnt_pos_ = joint_states.position;

//   // Update Jacobian
//   kinematic_state->setJointGroupPositions(joint_model_group, jnt_pos_);

//   // const moveit::core::LinkModel* link_ = kinematic_state->getLinkModel("gripper_link");

//   // if(!kinematic_state->getJacobian(joint_model_group, link_, Eigen::Vector3d(0.0, 0.0, 0.0), jacobian_)) {
// 	//   std::cout << "Jacobian not calculated..." << std::endl;
//   // } else {
// 	//   std::cout << "Jacobian working fine: " << jacobian_ << std::endl;
//   // }
//   jacobian_ = kinematic_state->getJacobian(joint_model_group);
// }

void SchunkInsertionController::updateJointEffort()
{
  boost::mutex::scoped_lock lock(joint_states_mutex);
  jnt_eff_ = joint_states.effort;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "schunk_insertion");

  SchunkInsertionController lc;

  ros::spin();

  return EXIT_SUCCESS;
}
