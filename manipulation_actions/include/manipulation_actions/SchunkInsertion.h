#ifndef MANIPULATION_ACTIONS_SCHUNK_INSERTION_H
#define MANIPULATION_ACTIONS_SCHUNK_INSERTION_H

// C++
#include <fstream>
#include <iostream>
#include <random>
#include <chrono>

// boost
#include <boost/thread/mutex.hpp>

// KDL
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/TwistStamped.h>
#include <manipulation_actions/SchunkInsertAction.h>
#include <manipulation_actions/SchunkInsertResult.h>
#include <manipulation_actions/SchunkInsertGoal.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <robot_controllers_interface/controller.h>
#include <robot_controllers_interface/joint_handle.h>
#include <robot_controllers_interface/controller_manager.h>

using namespace robot_controllers

class SchunkInsertionController
{

public:
    SchunkInsertionController();

private:

    // void jointStatesCallback(const sensor_msgs::JointState &msg);

    void executeInsertion(const manipulation_actions::SchunkInsertGoal &goal);

    // helpers
    void setupKDL();
    void updateJoints();
    void updateEffort();

    ros::NodeHandle n, pnh;

    // topics
    ros::Publisher cart_twist_cmd_publisher;
    // ros::Subscriber joint_states_subscriber;

    // actionlib
    actionlib::SimpleActionServer<manipulation_actions::SchunkInsertAction> schunk_insert_server;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;

    // TF
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    // boost::mutex joint_states_mutex;

    // control_msgs::FollowJointTrajectoryGoal hold_goal;
    // sensor_msgs::JointState joint_states;

    double max_force;
    double insert_duration;
    double insert_tol;
    int num_trail_max;
    double reset_duration;
    double max_reset_vel;

    std::string root_link_;
    KDL::Chain kdl_chain_;
    boost::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    KDL::JntArray jnt_pos_;
    KDL::JntArray jnt_pos_start;
    KDL::JntArray jnt_eff_;
    KDL::JntArray jnt_eff_cmd;
    control_msgs::FollowJointTrajectoryGoal jnt_goal;
    KDL::Jacobian jacobian_;
    std::vector<JointHandlePtr> joints_;

    boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fksolver_;

};

#endif // MANIPULATION_ACTIONS_SCHUNK_INSERTION_H
