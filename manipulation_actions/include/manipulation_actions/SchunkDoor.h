#ifndef MANIPULATION_ACTIONS_SCHUNK_DOOR_H
#define MANIPULATION_ACTIONS_SCHUNK_DOOR_H

// C++
#include <fstream>
#include <iostream>
#include <random>
#include <chrono>
#include <algorithm>
#include <cmath>

// boost
#include <boost/thread/mutex.hpp>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
//#include <robot_controllers_interface/controller.h>
//#include <robot_controllers_interface/joint_handle.h>
//#include <robot_controllers_interface/controller_manager.h>
//#include <robot_controllers/cartesian_twist.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

//MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Linear Controller
#include "manipulation_actions/LinearController.h"
#include "manipulation_actions/LinearMoveAction.h"

// Action
#include "manipulation_actions/SchunkDoorAction.h"
#include "rail_manipulation_msgs/ProcessSegmentedObjects.h"
#include "rail_manipulation_msgs/SegmentObjects.h"
#include "rail_manipulation_msgs/SegmentedObject.h"


#define PI 3.14159265

class SchunkDoor{

public:
    SchunkDoor();

private:

    void executeDoorAction (const manipulation_actions::SchunkDoorGoalConstPtr &goal);
    void planToPose(geometry_msgs::PoseStamped& pose, std::string& pose_frame, moveit::planning_interface::MoveGroupInterface::Plan& pose_plan);

    bool getGripperPreApproachPose(geometry_msgs::PoseStamped& pre_approach_gripper_pose_stamped);
    bool getGripperDoorClosePos(geometry_msgs::Point& door_closed_gripper_pos);
    bool getHandleInBase(geometry_msgs::TransformStamped& base_link_to_handle_tf);
    bool inTolerance(float value, float min, float max);

    ros::NodeHandle n, pnh;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;

    moveit::planning_interface::MoveGroupInterface* arm_group_;
    moveit::planning_interface::PlanningSceneInterface* planning_scene_interface_;

    actionlib::SimpleActionServer<manipulation_actions::SchunkDoorAction> schunk_door_server;
    actionlib::SimpleActionClient<manipulation_actions::LinearMoveAction> linear_move_client;


    std::string base_frame_ = "base_frame";
    std::string handle_frame_ = "handle_frame";
    std::string reference_frame_;

    float approach_angle;

    // segmentation related
    ros::ServiceClient seg_client_;
    std::string seg_frame_;
    float handle_width_min_,handle_width_max_;
    float handle_depth_min_,handle_depth_max_;
    float handle_height_min_,handle_height_max_;
    bool viz_;
};

#endif // MANIPULATION_ACTIONS_SCHUNK_DOOR_H
