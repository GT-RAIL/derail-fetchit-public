#ifndef MANIPULATION_ACTIONS_SCHUNK_INSERTION_CONTROLLER_H
#define MANIPULATION_ACTIONS_SCHUNK_INSERTION_CONTROLLER_H

// C++
#include <fstream>
#include <iostream>
#include <random>
#include <chrono>
#include <algorithm>
#include <cmath>

// boost
#include <boost/thread/mutex.hpp>

//MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


// ROS
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/TwistStamped.h>
#include <manipulation_actions/SchunkInsertAction.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <robot_controllers_interface/controller.h>
#include <robot_controllers_interface/joint_handle.h>
#include <robot_controllers_interface/controller_manager.h>
#include <robot_controllers/cartesian_twist.h>


#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

// Linear Controller
#include <manipulation_actions/LinearController.h>
#include <manipulation_actions/LinearMoveAction.h>

#define PI 3.14159265

class SchunkInsertionController
{

public:
    SchunkInsertionController();

private:

    void executeInsertion(const manipulation_actions::SchunkInsertGoalConstPtr &goal);

    // helpers
    void jointStatesCallback(const sensor_msgs::JointState &msg);
    void updateJacobian();
    void updateJointEffort();

    ros::NodeHandle n, pnh;

    // topics
    ros::Publisher cart_twist_cmd_publisher;
    ros::Subscriber joint_states_subscriber;

    // actionlib
    actionlib::SimpleActionServer<manipulation_actions::SchunkInsertAction> schunk_insert_server;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
    actionlib::SimpleActionClient<manipulation_actions::LinearMoveAction> linear_move_client;

    // TF
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2::Vector3 eef_twist_goal;
    tf2::Vector3 object_twist_goal;
    tf2::Vector3 object_linear_move_goal;
    tf2::Vector3 base_linear_move_goal;

    // *_msgs
    sensor_msgs::JointState joint_states;
    control_msgs::FollowJointTrajectoryGoal jnt_goal;
    geometry_msgs::Vector3 eef_pos_;
    geometry_msgs::Vector3 gripper_pos_start;
    geometry_msgs::Vector3 gripper_pos_end;
    geometry_msgs::Vector3 gripper_pos_reset;
    geometry_msgs::Vector3 object_twist_goal_msg;
    manipulation_actions::LinearMoveGoal linear_goal;

    // std
    std::vector<double> jnt_eff_;
    std::vector<double> eef_force_;
    std::vector<double> base_eef_force_;
    std::vector<double> jnt_pos_;
    std::vector<double> jnt_pos_start;

    // MoveIt stuff
    robot_model_loader::RobotModelLoader loader;
    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;
    robot_state::JointModelGroup* joint_model_group;

    // Eigen
    Eigen::MatrixXd jacobian_;

    // boost
    boost::mutex joint_states_mutex;

    // parameters
    int command_rate;
    double insert_duration;
    double insert_tol;
    double max_force;
    double max_reset_vel;
    int num_trial_max;
    double reposition_duration;
    double reset_duration;
    double travel_dist;
    double search_delta_theta;
    double search_dist;

};

#endif // MANIPULATION_ACTIONS_SCHUNK_INSERTION_CONTROLLER_H
