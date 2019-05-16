#ifndef MANIPULATION_ACTIONS_SCHUNK_GEAR_GRASPER_H
#define MANIPULATION_ACTIONS_SCHUNK_GEAR_GRASPER_H

// C++
#include <fstream>
#include <iostream>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <geometry_msgs/TwistStamped.h>
#include <manipulation_actions/AttachSimpleGeometry.h>
#include <manipulation_actions/SchunkGraspAction.h>
#include <manipulation_actions/SchunkRetrieveAction.h>
#include <manipulation_actions/LinearMoveAction.h>
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

class SchunkGearGrasper
{

public:
    SchunkGearGrasper();

    void publishTF();

private:

    void executeSchunkGearGrasp(const manipulation_actions::SchunkGraspGoalConstPtr &goal);

    void executeSchunkGearRetrieve(const manipulation_actions::SchunkRetrieveGoalConstPtr &goal);

    //void debugAttachObject(const manipulation_actions::KitManipGoalConstPtr &goal);

    void initGraspPoses();

    bool planToPose(geometry_msgs::PoseStamped& goal_pose,
                    moveit::planning_interface::MoveGroupInterface::Plan& pose_plan);

    ros::NodeHandle n, pnh;

    // topics
    ros::Publisher approach_pose_debug;
    ros::Publisher grasp_pose_debug;
    ros::Publisher takeout_pose_debug;

    // services
    ros::ServiceClient attach_simple_geometry_client;

    // actionlib
    actionlib::SimpleActionServer<manipulation_actions::SchunkGraspAction> schunk_gear_grasp_server;
    actionlib::SimpleActionServer<manipulation_actions::SchunkRetrieveAction> schunk_gear_retrieve_server;
    //actionlib::SimpleActionServer<manipulation_actions::KitManipAction> debug_attach_object_server;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_client;
    actionlib::SimpleActionClient<manipulation_actions::LinearMoveAction> linear_move_client;

    // MoveIt interfaces
    moveit::planning_interface::MoveGroupInterface *arm_group;
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;

    // preset poses
    std::vector<geometry_msgs::PoseStamped> grasp_poses;
    size_t current_grasp_pose;

    double approach_offset_in_x;
    double retrieve_offset_in_x;
    int max_planning_attempts;

    // TF
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::TransformBroadcaster tf_broadcaster;

};

#endif // MANIPULATION_ACTIONS_SCHUNK_GEAR_GRASPER_H
