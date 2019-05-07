#include <manipulation_actions/LinearController.h>

using std::max;

LinearController::LinearController() :
    pnh("~"),
    tf_listener(tf_buffer),
    linear_move_server(pnh, "linear_move", boost::bind(&LinearController::executeLinearMove, this, _1), false),
    arm_control_client("arm_controller/follow_joint_trajectory")
{
  pnh.param("max_linear_vel", max_vel, 0.3);
  pnh.param("goal_tolerance", goal_tolerance, 0.002);
  pnh.param("abort_threshold", abort_threshold, 0.08);
  abort_threshold = pow(abort_threshold, 2);
  kp = 5;

  hold_goal.trajectory.joint_names.push_back("shoulder_pan_joint");
  hold_goal.trajectory.joint_names.push_back("shoulder_lift_joint");
  hold_goal.trajectory.joint_names.push_back("upperarm_roll_joint");
  hold_goal.trajectory.joint_names.push_back("elbow_flex_joint");
  hold_goal.trajectory.joint_names.push_back("forearm_roll_joint");
  hold_goal.trajectory.joint_names.push_back("wrist_flex_joint");
  hold_goal.trajectory.joint_names.push_back("wrist_roll_joint");
  hold_goal.trajectory.points.resize(1);

  joint_states_subscriber = n.subscribe("joint_states", 1, &LinearController::jointStatesCallback, this);
  arm_cartesian_cmd_publisher = n.advertise<geometry_msgs::TwistStamped>("/arm_controller/cartesian_twist/command", 1);

  linear_move_server.start();
}

void LinearController::jointStatesCallback(const sensor_msgs::JointState &msg)
{
  boost::mutex::scoped_lock lock(joint_states_mutex);
  joint_states = msg;
}

void LinearController::executeLinearMove(const manipulation_actions::LinearMoveGoalConstPtr &goal)
{
  manipulation_actions::LinearMoveResult result;

  // get initial pose to calculate expected duration
  geometry_msgs::TransformStamped gripper_tf = tf_buffer.lookupTransform("base_link", "gripper_link", ros::Time(0),
      ros::Duration(1.0));
  double x_err = goal->point.x - gripper_tf.transform.translation.x;
  double y_err = goal->point.y - gripper_tf.transform.translation.y;
  double z_err = goal->point.z - gripper_tf.transform.translation.z;

  geometry_msgs::TwistStamped cmd;
  cmd.header.frame_id = "base_link";

  ros::Rate controller_rate(100);
  double move_duration = max(max(fabs(x_err), fabs(y_err)), fabs(z_err)) / max_vel;
  move_duration *= 1.5; // give the controller some extra time in case things don't go perfectly smoothly
  ros::Time end_time = ros::Time::now() + ros::Duration(move_duration);

  while (ros::Time::now() < end_time || (fabs(x_err) < goal_tolerance && fabs(y_err) < goal_tolerance
    && fabs(z_err) < goal_tolerance))
  {
    gripper_tf = tf_buffer.lookupTransform("base_link", "gripper_link", ros::Time(0),
                                           ros::Duration(0.05));
    x_err = goal->point.x - gripper_tf.transform.translation.x;
    y_err = goal->point.y - gripper_tf.transform.translation.y;
    z_err = goal->point.z - gripper_tf.transform.translation.z;

    double dx = kp*x_err;
    double dy = kp*y_err;
    double dz = kp*z_err;

    double max_cmd = max(max(fabs(dx), fabs(dy)), fabs(dz));
    if (max_cmd > max_vel)
    {
      double scale = max_vel/max_cmd;
      dx *= scale;
      dy *= scale;
      dz *= scale;
    }

    cmd.twist.linear.x = dx;
    cmd.twist.linear.y = dy;
    cmd.twist.linear.z = dz;

    arm_cartesian_cmd_publisher.publish(cmd);

//    ros::spinOnce();
    controller_rate.sleep();
  }

  cmd.twist.linear.x = 0;
  cmd.twist.linear.y = 0;
  cmd.twist.linear.z = 0;
  arm_cartesian_cmd_publisher.publish(cmd);

  if (goal->hold_final_pose)
  {
    boost::mutex::scoped_lock lock(joint_states_mutex);

    // publish a non-trajectory to FollowJointTrajectory to disable gravity comp
    hold_goal.trajectory.points[0].positions.clear();
    for (size_t i = 6; i < 6 + hold_goal.trajectory.joint_names.size(); i ++)
    {
      hold_goal.trajectory.points[0].positions.push_back(joint_states.position[i]);
    }

    arm_control_client.sendGoal(hold_goal);
    arm_control_client.waitForResult(ros::Duration(0.5));
  }

  ros::Duration(0.1).sleep();  // let the arm settle
  gripper_tf = tf_buffer.lookupTransform("base_link", "gripper_link", ros::Time(0),
                                         ros::Duration(0.05));
  x_err = goal->point.x - gripper_tf.transform.translation.x;
  y_err = goal->point.y - gripper_tf.transform.translation.y;
  z_err = goal->point.z - gripper_tf.transform.translation.z;

  result.error.x = x_err;
  result.error.y = y_err;
  result.error.z = z_err;

  ROS_INFO("Final linear translation error: (%f, %f, %f)", x_err, y_err, z_err);
  if (pow(x_err, 2) + pow(y_err, 2) + pow(z_err, 2) > abort_threshold)
  {
    ROS_INFO("Did not pass execution threshold, aborting linear move action server.");
    linear_move_server.setAborted(result);
    return;
  }

  linear_move_server.setSucceeded(result);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "linear_controller");

  LinearController lc;

  ros::spin();

  return EXIT_SUCCESS;
}
