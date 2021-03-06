#ifndef MANIPULATION_ACTIONS_LINEAR_CONTROLLER_H
#define MANIPULATION_ACTIONS_LINEAR_CONTROLLER_H

// C++
#include <fstream>
#include <iostream>

// boost
#include <boost/thread/mutex.hpp>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/TwistStamped.h>
#include <manipulation_actions/LinearMoveAction.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class LinearController
{

public:
    LinearController();

private:

    void executeLinearMove(const manipulation_actions::LinearMoveGoalConstPtr &goal);

    ros::NodeHandle n, pnh;

    // topics
    ros::Publisher arm_cartesian_cmd_publisher;

    // actionlib
    actionlib::SimpleActionServer<manipulation_actions::LinearMoveAction> linear_move_server;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;

    // TF
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    control_msgs::FollowJointTrajectoryGoal hold_goal;

    double max_vel;
    double goal_tolerance;
    double abort_threshold;

    double kp;
    double ip;
};

#endif // MANIPULATION_ACTIONS_LINEAR_CONTROLLER_H
