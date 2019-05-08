#include <manipulation_actions/SchunkInsertionController.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

using std::max;

SchunkInsertionController::SchunkInsertionController():
    pnh("~"),
    tf_listener(tf_buffer),
    schunk_insert_server(pnh, "schunk_insert", boost::bind(&SchunkInsertionController::executeInsertion, this, _1), false),
    arm_control_client("arm_controller/follow_joint_trajectory"),
    controller_manager_(new robot_controllers::ControllerManager)
{
  max_force = 1; //TODO: identify the ideal threshold for detecting collision
  insert_duration = 3; // TODO: find out the ideal duration
  insert_tol = 0.04; //TODO: identify the ideal tolerance for detection insertion
  max_reset_vel = 0.01; // TODO: identify the ideal maximum reset velocity
  num_trail_max = 5; //TODO: identify the ideal num of trails
  reset_duration = 0.5; // TODO: find out the ideal duration

  jnt_goal.trajectory.joint_names.push_back("shoulder_pan_joint");
  jnt_goal.trajectory.joint_names.push_back("shoulder_lift_joint");
  jnt_goal.trajectory.joint_names.push_back("upperarm_roll_joint");
  jnt_goal.trajectory.joint_names.push_back("elbow_flex_joint");
  jnt_goal.trajectory.joint_names.push_back("forearm_roll_joint");
  jnt_goal.trajectory.joint_names.push_back("wrist_flex_joint");
  jnt_goal.trajectory.joint_names.push_back("wrist_roll_joint");
  jnt_goal.trajectory.points.resize(1);

  // jointstates subcriber to get feedback on effort
  // joint_states_subscriber = n.subscribe("joint_states", 1, &SchunkInsertionController::jointStatesCallback, this);

  // cart_twist publisher to send command to the controller
  cart_twist_cmd_publisher = n.advertise<geometry_msgs::TwistStamped>("/arm_controller/cartesian_twist/command", 1);

  // Setup kinematics
  setupKDL();
  std::cout << "Completed KDL setup" << std::endl;

  // Start controller
  schunk_insert_server.start();
  std::cout << "schunk_insert_server started" << std::endl;
}

// void SchunkInsertionController::jointStatesCallback(const sensor_msgs::JointState &msg)
// {
//   boost::mutex::scoped_lock lock(joint_states_mutex);
//   joint_states = msg;
// }

void SchunkInsertionController::executeInsertion(const manipulation_actions::SchunkInsertGoalConstPtr& goal)
{
  std::cout << "Setting things up for insertion.." << std::endl;
  manipulation_actions::SchunkInsertResult result;

  KDL::Vector cart_pos_;
  KDL::Frame cart_pose_;
  KDL::Vector eef_pos_start;
  KDL::Frame eef_pose_start;

  tf2::Vector3 eef_twist_goal;
  std::vector<double> eef_force_{0.,0.,0.}; // initialize eef force to zero
  geometry_msgs::Vector3 object_twist_goal_msg;
  tf2::Vector3 object_twist_goal;

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
  std::cout << "Transforming command to end-effector frame" << std::endl;
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

  ros::Rate controller_rate(30); // TODO: find out the ideal rate
  ros::Time end_time = ros::Time::now() + ros::Duration(insert_duration);

  // Save initial configuration and eef position
  std::cout << "Saving initial configuration..." << std::endl;
  updateJoints();
  std::cout << "Initial configuration saved!" << std::endl;

  jnt_pos_start = jnt_pos_;
  if (fksolver_->JntToCart(jnt_pos_, eef_pose_start) < 0)
  {
    ROS_ERROR_THROTTLE(1.0, "FKsolver solver failed");
  }
  eef_pos_start = eef_pose_start.p; // extract only the position


  // Start insertion attempts
  bool success = false;

  std::cout << "Starting insertion attempts" << std::endl;

  for (unsigned int k =0 ; k < num_trail_max ; ++k)
  {
    std::cout << "Attempt #" << k << std::endl;

    while (ros::Time::now() < end_time || (fabs(eef_force_[0]) < max_force && fabs(eef_force_[0]) < max_force && fabs(eef_force_[2]) < max_force))
    {
      // Publish the command
      cart_twist_cmd_publisher.publish(cmd);

      // Compute interaction forces
      updateJoints(); // This updates jnt_pos_

      updateEffort(); // This updates jnt_eff_

      jac_solver_->JntToJac(jnt_pos_, jacobian_); // Get jacobian

      for (unsigned int i = 0 ; i < 3 ; ++i)
      {
        eef_force_[i] = 0;
        for (unsigned int j = 0 ; j < kdl_chain_.getNrOfJoints() ; ++j)
          eef_force_[i] += jacobian_(i,j) * jnt_eff_(j);
      }

      ros::spinOnce();
      controller_rate.sleep();
    }

    // compute eef position
    if (fksolver_->JntToCart(jnt_pos_, cart_pose_) < 0)
    {
      ROS_ERROR_THROTTLE(1.0, "FKsolver solver failed");
    }
    cart_pos_ = cart_pose_.p; // extract only the positions

    // Check for success
    if (fabs(cart_pos_.y() - eef_pos_start.y()) > insert_tol)
    {
      success = true;
      k = num_trail_max; // end attempts if successful
    }
    else
    {
      // reset the arm to the starting point
      jnt_goal.trajectory.points[0].positions.clear();
      for (size_t i = 6; i < 6 + jnt_goal.trajectory.joint_names.size(); i ++)
      {
        jnt_goal.trajectory.points[0].positions.push_back(jnt_pos_start(i));
      }
      arm_control_client.sendGoal(jnt_goal);
      arm_control_client.waitForResult(ros::Duration(0.5));

      // generate a random cartesian velocity to move to new position and try again
      reset_cmd.twist.linear.x = distribution(generator);
      reset_cmd.twist.linear.y = distribution(generator);
      reset_cmd.twist.linear.z = distribution(generator);

      // apply the random velocity
      ros::Time reset_end_time = ros::Time::now() + ros::Duration(reset_duration);
      while (ros::Time::now() < reset_end_time)
      {
        cart_twist_cmd_publisher.publish(reset_cmd);
      }
    }

  }

  // Set success
  if (success == true) {
    schunk_insert_server.setSucceeded(result);
  } else {
    schunk_insert_server.setAborted(result);
  }

}

void SchunkInsertionController::updateJoints()
{
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    std::cout << "accessing joints_ position" << std::endl;
    joints_[i]->getPosition();
    std::cout << "accessing jnt_pos_" << std::endl;
    jnt_pos_(i);
    std::cout << "accessing successful" << std::endl;

    jnt_pos_(i) = joints_[i]->getPosition();
  }
}

void SchunkInsertionController::updateEffort()
{
  for (size_t i = 0; i < joints_.size(); ++i)
    jnt_eff_(i) = joints_[i]->getEffort();
}

void SchunkInsertionController::setupKDL()
{
  // define controller manager
//  robot_controllers::ControllerManager manager_;
//  robot_controllers::Controller::init(pnh, *controller_manager_);
  controller_manager_->init(n);
  twist_controller_.init(n, controller_manager_);

  // Initialize KDL structures
  std::string tip_link;
  pnh.param<std::string>("root_name", root_link_, "torso_lift_link");
  pnh.param<std::string>("tip_name", tip_link, "gripper_link");

  // Load URDF
  urdf::Model model;
  if (!model.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse URDF");
  }

  // Load the tree
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
  {
    ROS_ERROR("Could not construct tree from URDF");
  }

  // Populate the chain
  if(!kdl_tree.getChain(root_link_, tip_link, kdl_chain_))
  {
    ROS_ERROR("Could not construct chain from URDF");
  }

  jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
  jnt_pos_.resize(kdl_chain_.getNrOfJoints());
  jnt_eff_.resize(kdl_chain_.getNrOfJoints());
  jnt_eff_cmd.resize(kdl_chain_.getNrOfJoints());
  jacobian_.resize(kdl_chain_.getNrOfJoints());

  // Init joint handles
  joints_.clear();
  joints_.resize(kdl_chain_.getNrOfSegments());
  std::cout << "Starting joint init" << std::endl;
  for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
  {
    if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
    {
      std::cout << "loop iter: " << i << std::endl;
      joints_.push_back(robot_controllers::JointHandlePtr(
         controller_manager_->getJointHandle(kdl_chain_.getSegment(i).getJoint().getName())));
      controller_manager_->getJointHandle(kdl_chain_.getSegment(i).getJoint()
      .getName())->getPosition();
//      joints_[i]->getPosition();
      std::cout << "joint set" << std::endl;
    }
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "schunk_insert");

  SchunkInsertionController lc;

  ros::spin();

  return EXIT_SUCCESS;
}
