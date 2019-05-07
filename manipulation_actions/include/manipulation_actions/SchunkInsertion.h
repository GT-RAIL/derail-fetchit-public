#ifndef MANIPULATION_ACTIONS_SCHUNK_INSERTION_H
#define MANIPULATION_ACTIONS_SCHUNK_INSERTION_H

// C++
#include <fstream>
#include <iostream>

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
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class SchunkInsertion
{

public:
    SchunkInsertionController();

private:

    // void jointStatesCallback(const sensor_msgs::JointState &msg);

    void executeInsertion(const geometry_msgs::TwistStamped &goal);

    ros::NodeHandle n, pnh;

    // topics
    ros::Publisher cart_twist_cmd_publisher;
    // ros::Subscriber joint_states_subscriber;

    // actionlib
    actionlib::SimpleActionServer<manipulation_actions::SchunkInsertAction> schunk_insert_server;
    // actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;

    // TF
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    // boost::mutex joint_states_mutex;

    // control_msgs::FollowJointTrajectoryGoal hold_goal;
    // sensor_msgs::JointState joint_states;

    double max_force;
    double insert_tol;
    int num_trail_max;

    std::string root_link_;
    KDL::Chain kdl_chain_;
    boost::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    KDL::JntArray jnt_pos_;
    KDL::JntArray jnt_pos_start;
    KDL::JntArray jnt_eff_;
    KDL::JntArray jnt_eff_cmd;
    KDL::Jacobian jacobian_;
    std::vector<JointHandlePtr> joints_;
    KDL::Frame cart_pose_;

};

#endif // MANIPULATION_ACTIONS_SCHUNK_INSERTION_H
