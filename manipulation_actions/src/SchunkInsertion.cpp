#include <manipulation_actions/SchunkInsertion.h>

using std::max;

SchunkInsertionController::SchunkInsertionController():
    pnh("~"),
    tf_listener(tf_buffer),
    schunk_insert_server(pnh, "schunk_insert", boost::bind(&SchunkInsertionController::executeInsertion, this, _1), false),
{
  max_force = 1; //TODO: identify the threshold for detecting collision

  // jointstates subcriber to get feedback on effort
  // joint_states_subscriber = n.subscribe("joint_states", 1, &SchunkInsertionController::jointStatesCallback, this);

  // cart_twist publisher to send command to the controller
  cart_twist_cmd_publisher = n.advertise<geometry_msgs::TwistStamped>("/arm_controller/cartesian_twist/command", 1);

  // setup kinematics
  setupKDL();

  schunk_insert_server.start();
}

// void SchunkInsertionController::jointStatesCallback(const sensor_msgs::JointState &msg)
// {
//   boost::mutex::scoped_lock lock(joint_states_mutex);
//   joint_states = msg;
// }

void SchunkInsertionController::executeInsertion(const geometry_msgs::TwistStamped &goal)
{
  // manipulation_actions::SchunkInsertResult result;
  tf::vector3 eef_twist_goal;
  std::vector<double> eef_force_{0.,0.,0.}; // initialize eef force to zero
  geometry_msgs::vector3 object_twist_goal_msg;
  tf::vector3 object_twist_goal;

  // setup command message
  geometry_msgs::TwistStamped cmd;
  cmd.header.frame_id = "end_effector_frame";

  // get the transform for large gear
  geometry_msgs::TransformStamped object_transform_msg = tf_buffer.lookupTransform("object_frame", "gripper_link", ros::Time(0), ros::Duration(1.0));

  object_tf = tf_buffer.transformMsgToTF(object_transform_msg.transform);

  // Convert desired velocity from object frame to end effector frame
  object_twist_goal_msg.x = goal->twist.linear.x;
  object_twist_goal_msg.y = goal->twist.linear.y;
  object_twist_goal_msg.z = goal->twist.linear.z;

  object_twist_goal = tf_buffer.vector3MsgToTF(object_twist_goal_msg);

  eef_twist_goal = object_tf * object_twist_goal;

  cmd.twist.linear.x = eef_twist_goal.x;
  cmd.twist.linear.y = eef_twist_goal.y;
  cmd.twist.linear.z = eef_twist_goal.z;

  ros::Rate controller_rate(30); // TODO: find out the ideal rate
  double move_duration = 3; // TODO: find out the ideal duration
  ros::Time end_time = ros::Time::now() + ros::Duration(move_duration);

  while (ros::Time::now() < end_time || (fabs(eef_force_(1)) < max_force && fabs(eef_force_(2)) < max_force && fabs(eef_force_(3)) < max_force))
  {
    // Publish the command
    cart_twist_cmd_publisher.publish(cmd);

    // Compute interaction forces
    updateJoints(); // This updates jnt_pos_

    updateEffort(); // This updates jnt_eff_

    jac_solver_->JntToJac(jnt_pos_, jacobian_); // Get jacobian

    for (unsigned int i = 0 ; i < 3 ; ++i)
    {
      eef_force_(i) = 0;
      for (unsigned int j = 0 ; j < kdl_chain_.getNrOfJoints() ; ++j)
        eef_force_(i) += jacobian_(i,j) * jnt_eff_(j);
    }

    ros::spinOnce();
    controller_rate.sleep();
  }

  schunk_insert_server.setSucceeded(result);
}

void SchunkInsertionController::updateJoints()
{
  for (size_t i = 0; i < joints_.size(); ++i)
    jnt_pos_(i) = joints_[i]->getPosition();
}

void SchunkInsertionController::updateEffort()
{
  for (size_t i = 0; i < joints_.size(); ++i)
    jnt_eff_(i) = joints_[i]->getEffort();
}

void SchunkInsertionController:setupKDL()
{
  // Initialize KDL structures
  std::string tip_link;
  pnh.param<std::string>("root_name", root_link_, "torso_lift_link");
  pnh.param<std::string>("tip_name", tip_link, "gripper_link");

  // Load URDF
  urdf::Model model;
  if (!model.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse URDF");
    return -1;
  }

  // Load the tree
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
  {
    ROS_ERROR("Could not construct tree from URDF");
    return -1;
  }

  // Populate the chain
  if(!kdl_tree.getChain(root_link_, tip_link, kdl_chain_))
  {
    ROS_ERROR("Could not construct chain from URDF");
    return -1;
  }

  jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
  jnt_pos_.resize(kdl_chain_.getNrOfJoints());
  jnt_eff_.resize(kdl_chain_.getNrOfJoints());
  jnt_eff_cmd.resize(kdl_chain_.getNrOfJoints());
  jacobian_.resize(kdl_chain_.getNrOfJoints());

  // Init joint handles
  joints_.clear();
  for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
    if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
      joints_.push_back(manager_->getJointHandle(kdl_chain_.getSegment(i).getJoint().getName()));

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "schunk_insert");

  SchunkInsertion lc;

  ros::spin();

  return EXIT_SUCCESS;
}
