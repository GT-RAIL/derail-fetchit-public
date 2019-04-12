#ifndef MANIPULATION_ACTIONS_COLLISION_SCENE_MANAGER_H
#define MANIPULATION_ACTIONS_COLLISION_SCENE_MANAGER_H

// Boost
#include <boost/thread/mutex.hpp>

// C++
#include <fstream>
#include <iostream>

// ROS
#include <manipulation_actions/AttachArbitraryObject.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
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
    ros::Subscriber objects_subscriber;

    // services
    ros::ServiceServer attach_closest_server;
    ros::ServiceServer detach_all_server;
    ros::ServiceServer attach_arbitrary_server;

    // MoveIt interfaces
    moveit::planning_interface::MoveGroupInterface *arm_group;
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;

    boost::mutex objects_mutex;
    rail_manipulation_msgs::SegmentedObjectList object_list; //the last received object list
    std::vector<std::string> attached_objects;  // the names of the objects (in the planning scene) attached to the robot
    std::vector<std::string> unattached_objects;  // the names of all unattached objects in the planning scene

    // TF
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    void objectsCallback(const rail_manipulation_msgs::SegmentedObjectList &msg);

    bool attachClosestObject(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    bool detachAllObjects(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    bool attachArbitraryObject(manipulation_actions::AttachArbitraryObject::Request &req,
        manipulation_actions::AttachArbitraryObject::Response &res);
};

#endif // MANIPULATION_ACTIONS_COLLISION_SCENE_MANAGER_H