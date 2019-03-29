#ifndef MANIPULATION_ACTIONS_PLACER_H
#define MANIPULATION_ACTIONS_PLACER_H

// C++
#include <fstream>
#include <iostream>

// ROS
#include <actionlib/server/simple_action_server.h>
#include <manipulation_actions/StoreObjectAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class Placer
{

public:
    Placer();

    void publishTF();

private:

    void executeStore(const manipulation_actions::StoreObjectGoalConstPtr &goal);

    ros::NodeHandle n, pnh;

    // topics
    ros::Publisher object_place_pose_debug;
    ros::Publisher place_pose_bin_debug;
    ros::Publisher place_pose_base_debug;

    // actionlib
    actionlib::SimpleActionServer<manipulation_actions::StoreObjectAction> store_object_server;

    // MoveIt interfaces
    moveit::planning_interface::MoveGroupInterface *arm_group;
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;

    bool attach_arbitrary_object;

    // TF
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    bool debug;
};

#endif // MANIPULATION_ACTIONS_PLACER_H