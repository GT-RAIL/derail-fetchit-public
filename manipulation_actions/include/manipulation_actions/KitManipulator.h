#ifndef MANIPULATION_ACTIONS_KIT_MANIPULATOR_H
#define MANIPULATION_ACTIONS_KIT_MANIPULATOR_H

// C++
#include <fstream>
#include <iostream>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <geometry_msgs/TwistStamped.h>
#include <manipulation_actions/AttachArbitraryObject.h>
#include <manipulation_actions/KitManipAction.h>
#include <manipulation_actions/LinearMoveAction.h>
#include <manipulation_actions/ScoredPose.h>
#include <manipulation_actions/StoreObjectAction.h>
#include <manipulation_actions/ToggleGripperCollisions.h>
#include <moveit_msgs/GetCartesianPath.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>

class KitManipulator
{

public:
    KitManipulator();

    void publishTF();

private:

    void executeStore(const manipulation_actions::StoreObjectGoalConstPtr &goal);

    void executeKitPick(const manipulation_actions::KitManipGoalConstPtr &goal);

    void executeKitPlace(const manipulation_actions::KitManipGoalConstPtr &goal);

    void initPickPoses();

    bool toggleGripperCollisions(std::string object, bool allow_collisions);

    ros::NodeHandle n, pnh;

    // topics
    ros::Publisher object_place_pose_debug;
    ros::Publisher place_pose_bin_debug;
    ros::Publisher place_pose_base_debug;
    ros::Publisher arm_cartesian_cmd_publisher;

    // services
    ros::ServiceClient attach_arbitrary_object_client;
    ros::ServiceClient attach_closest_object_client;
    ros::ServiceClient detach_objects_client;
    ros::ServiceClient reattach_held_to_base_client;
    ros::ServiceClient toggle_gripper_collisions_client;

    // actionlib
    actionlib::SimpleActionServer<manipulation_actions::StoreObjectAction> store_object_server;
    actionlib::SimpleActionServer<manipulation_actions::KitManipAction> kit_pick_server;
    actionlib::SimpleActionServer<manipulation_actions::KitManipAction> kit_place_server;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_client;
    actionlib::SimpleActionClient<manipulation_actions::LinearMoveAction> linear_move_client;

    // MoveIt interfaces
    moveit::planning_interface::MoveGroupInterface *arm_group;
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;

    // preset poses
    std::vector<geometry_msgs::PoseStamped> kit_pick_poses;
    std::vector<sensor_msgs::JointState> kit_place_poses;
    size_t current_grasp_pose;

    bool attach_arbitrary_object;

    double low_place_height;
    double high_place_height;

    // TF
    tf2_ros::TransformBroadcaster tf_broadcaster;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    bool plan_mode;

    bool debug;
    bool pause_for_verification;
};

#endif // MANIPULATION_ACTIONS_KIT_MANIPULATOR_H
