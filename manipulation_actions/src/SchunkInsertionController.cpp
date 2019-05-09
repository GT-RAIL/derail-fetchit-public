#include <manipulation_actions/SchunkInsertionController.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>

using std::max;

SchunkInsertionController::SchunkInsertionController():
    pnh("~"),
    tf_listener(tf_buffer),
    schunk_insert_server(pnh, "schunk_insert", boost::bind(&SchunkInsertionController::executeInsertion, this, _1), false),
    arm_control_client("arm_controller/follow_joint_trajectory"),
    loader("robot_description")
{

  // Setup parameters. TODO: Velocity should be 0.05
  pnh.param<int>("command_rate", command_rate, 50); // identify the ideal rate to run the controller
  pnh.param<double>("max_force", max_force, 0.1); // identify the ideal threshold for detecting collision
  pnh.param<double>("insert_duration", insert_duration, 4); // find out the ideal duration
  pnh.param<double>("insert_tol", insert_tol, 0.1); // identify the ideal tolerance for detection insertion
  pnh.param<double>("max_reset_vel", max_reset_vel, 0.05); // identify the ideal maximum reset velocity
  pnh.param<int>("num_trial_max", num_trial_max, 10); // identify the ideal num of trails
  pnh.param<double>("reposition_duration", reposition_duration, 0.5); // find out the ideal duration
  pnh.param<double>("reset_duration", reset_duration, insert_duration); // find out the ideal duration

  jnt_goal.trajectory.joint_names.push_back("shoulder_pan_joint");
  jnt_goal.trajectory.joint_names.push_back("shoulder_lift_joint");
  jnt_goal.trajectory.joint_names.push_back("upperarm_roll_joint");
  jnt_goal.trajectory.joint_names.push_back("elbow_flex_joint");
  jnt_goal.trajectory.joint_names.push_back("forearm_roll_joint");
  jnt_goal.trajectory.joint_names.push_back("wrist_flex_joint");
  jnt_goal.trajectory.joint_names.push_back("wrist_roll_joint");
  jnt_goal.trajectory.points.resize(1);

  // initialize vectors
  eef_force_.emplace_back(0);
  eef_force_.emplace_back(0);
  eef_force_.emplace_back(0);

  // joint_states subscriber to get feedback on effort
  joint_states_subscriber = n.subscribe("joint_states", 1, &SchunkInsertionController::jointStatesCallback, this);

  // cart_twist publisher to send command to the controller
  cart_twist_cmd_publisher = n.advertise<geometry_msgs::TwistStamped>("/arm_controller/cartesian_twist/command", 1);

  // Setup MoveIt
  kinematic_model = loader.getModel();
  robot_state::RobotStatePtr temp_kinematic_state(new robot_state::RobotState(kinematic_model));
  joint_model_group = kinematic_model->getJointModelGroup("arm");
  kinematic_state = temp_kinematic_state;
  
  ROS_INFO("Completed MoveIt setup");
  

  // Start controller
  schunk_insert_server.start();
  ROS_INFO("schunk_insert_server started");
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

  // setup random seed for repeated attempts
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator (seed);
  std::uniform_real_distribution<double> distribution (0.0,max_reset_vel);

  // setup command messages
  geometry_msgs::TwistStamped cmd;
  cmd.header.frame_id = "end_effector_frame";
  geometry_msgs::TwistStamped reset_cmd;
  reset_cmd.header.frame_id = "end_effector_frame";

  // get the transform for large gear
  ROS_INFO("Transforming command to end-effector frame");
  geometry_msgs::TransformStamped object_transform_msg = tf_buffer.lookupTransform("object_frame", "gripper_link", ros::Time(0), ros::Duration(1.0));

  tf2::Transform object_tf;
  tf2::fromMsg(object_transform_msg.transform,object_tf);

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

  for (unsigned int k =0 ; k < num_trial_max ; ++k)
  {
    ros::Time end_time = ros::Time::now() + ros::Duration(insert_duration);

    geometry_msgs::TransformStamped eef_transform_start_msg = tf_buffer.lookupTransform("base_link", "gripper_link",
                                                                                        ros::Time(0), ros::Duration(1.0));
    eef_pos_start = eef_transform_start_msg.transform.translation; // extract only the position

    ROS_INFO("Initial eef position saved!");

    ROS_INFO("Attempt #%u of %d", k+1, num_trial_max);

    // Set initial eef_force to zero
    eef_force_[0] = 0;
    eef_force_[1] = 0;
    eef_force_[2] = 0;

    while (ros::Time::now() < end_time)
    {
      if (!((fabs(eef_force_[0]) < max_force && fabs(eef_force_[1]) < max_force && fabs(eef_force_[2]) < max_force)))
      {
        ROS_INFO("Force feedback exceeded!");
        break;
      }

      // Publish the command
      cart_twist_cmd_publisher.publish(cmd);

      // Compute interaction forces
      updateJointEffort(); // This updates jnt_eff_

      updateJacobian(); // This updates jacobian_

      float joint_norm = 0;
      for (unsigned int i = 0 ; i < 3 ; ++i)
      {
        eef_force_[i] = 0;
        for (unsigned int j = 0 ; j < 6; ++j)
	      {
          eef_force_[i] += jacobian_(i,j) * jnt_eff_[j];
          if (i == 0)
          {
            joint_norm += pow(jnt_eff_[j], 2);
          }
	      }
      }
      ROS_INFO("Force: %f, %f, %f, %f\n", eef_force_[0], eef_force_[1], eef_force_[2], joint_norm);

      // Check for preempt
      if (schunk_insert_server.isPreemptRequested())
      {
        schunk_insert_server.setPreempted(result);
        return;
      }
      controller_rate.sleep();
    }

    // Check for success
    ROS_INFO("Checking for success...");
    geometry_msgs::TransformStamped eef_transform_msg_ = tf_buffer.lookupTransform("base_link", "gripper_link",
        ros::Time(0), ros::Duration(1.0)); // update eef position
    eef_pos_ = eef_transform_msg_.transform.translation; // extract only the position

    if (fabs(eef_pos_.y - eef_pos_start.y) > insert_tol)
    {
      ROS_INFO("Insertion succeeded!");
      success = true;
      k = num_trial_max; // end attempts if successful
    }
    else
    {
      ROS_INFO("Insertion Failed!");

      // TODO: Remove this if
      if (k < num_trial_max)
      {
        // ros::Duration(1).sleep();

        // reset the arm to the starting point
        ROS_INFO("resetting arm to original starting point...");
        arm_control_client.sendGoal(jnt_goal);
        arm_control_client.waitForResult();
  //      auto arm_controller_status = arm_control_client.getState();
  //      auto arm_controller_result = arm_control_client.getResult();
  //      ROS_INFO_STREAM("tests" << arm_controller_result->error_code << " - " << arm_controller_result->error_string);
  //      ROS_INFO("arm controller reported a status of %d...", arm_controller_status.state_);

        ros::Duration(2).sleep();

        // generate a random cartesian velocity to move to new position and try again
        ROS_INFO("Moving to a new starting point...");
        reset_cmd.twist.linear.x = distribution(generator);
        reset_cmd.twist.linear.y = distribution(generator);
        reset_cmd.twist.linear.z = distribution(generator);

        // apply the random velocity
        ros::Time reset_end_time = ros::Time::now() + ros::Duration(reposition_duration);
        while (ros::Time::now() < reset_end_time)
        {
          cart_twist_cmd_publisher.publish(reset_cmd);

          // Check for preempt
          if (schunk_insert_server.isPreemptRequested())
          {
            schunk_insert_server.setPreempted(result);
            return;
          }
        }
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

void SchunkInsertionController::updateJacobian()
{
  // Extract joint position
  boost::mutex::scoped_lock lock(joint_states_mutex);
  jnt_pos_ = joint_states.position;

  // Update Jacobian
  kinematic_state->setJointGroupPositions(joint_model_group, jnt_pos_);
  jacobian_ = kinematic_state->getJacobian(joint_model_group);
}

void SchunkInsertionController::updateJointEffort()
{
  boost::mutex::scoped_lock lock(joint_states_mutex);
  jnt_eff_ = joint_states.effort;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "schunk_insert");

  SchunkInsertionController lc;

  ros::spin();

  return EXIT_SUCCESS;
}
