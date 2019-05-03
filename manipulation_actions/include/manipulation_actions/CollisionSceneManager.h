#ifndef MANIPULATION_ACTIONS_COLLISION_SCENE_MANAGER_H
#define MANIPULATION_ACTIONS_COLLISION_SCENE_MANAGER_H

// Boost
#include <boost/thread/mutex.hpp>

// C++
#include <fstream>
#include <iostream>

// ROS
#include <manipulation_actions/AttachArbitraryObject.h>
#include <manipulation_actions/AttachToBase.h>
#include <manipulation_actions/ToggleGripperCollisions.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PlanningScene.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_srvs/Empty.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class CollisionSceneManager
{

public:
    static constexpr float SCENE_OBJECT_DST_SQR_THRESHOLD = 0.04;

    CollisionSceneManager();

private:

    ros::NodeHandle n, pnh;

    // topics
    ros::Publisher planning_scene_publisher;
    ros::Subscriber objects_subscriber;

    // services
    ros::ServiceServer attach_closest_server;
    ros::ServiceServer detach_all_server;
    ros::ServiceServer attach_arbitrary_server;
    ros::ServiceServer attach_base_server;
    ros::ServiceServer detach_base_server;
    ros::ServiceServer reattach_held_to_base_server;
    ros::ServiceServer toggle_gripper_collisions_server;
    ros::ServiceClient planning_scene_client;

    // MoveIt interfaces
    moveit::planning_interface::MoveGroupInterface *arm_group;
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;

    boost::mutex objects_mutex;
    rail_manipulation_msgs::SegmentedObjectList object_list; //the last received object list
    std::vector<std::string> attached_objects;  // the names of objects (in the planning scene) attached to the gripper
    std::vector<std::string> base_attached_objects;  // the names of objects that are attached to the robot base
    std::vector<std::string> unattached_objects;  // the names of all unattached objects in the planning scene

    // TF
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    void objectsCallback(const rail_manipulation_msgs::SegmentedObjectList &msg);

    bool attachClosestObject(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    bool detachAllObjects(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    bool attachArbitraryObject(manipulation_actions::AttachArbitraryObject::Request &req,
        manipulation_actions::AttachArbitraryObject::Response &res);

    bool attachBase(manipulation_actions::AttachToBase::Request &req,
        manipulation_actions::AttachToBase::Response &res);

    bool detachBase(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    bool reattachHeldToBase(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    bool toggleGripperCollisions(manipulation_actions::ToggleGripperCollisions::Request &req,
        manipulation_actions::ToggleGripperCollisions::Response &res);

    moveit_msgs::CollisionObject collisionFromSegmentedObject(const rail_manipulation_msgs::SegmentedObject &msg,
        std::string suffix="");
};

#endif // MANIPULATION_ACTIONS_COLLISION_SCENE_MANAGER_H
