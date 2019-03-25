#ifndef IN_HAND_LOCALIZER_H
#define IN_HAND_LOCALIZER_H

// C++
#include <fstream>
#include <iostream>

// Boost
#include <boost/thread/mutex.hpp>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/PointHeadAction.h>
#include <eigen_conversions/eigen_msg.h>
#include <manipulation_actions/InHandLocalizeAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class InHandLocalizer
{

public:
    InHandLocalizer();

    void publishTF();

private:

    void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg);

    void executeLocalize(const manipulation_actions::InHandLocalizeGoalConstPtr &goal);

    bool extractObjectCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &object_cloud);

    bool moveToLocalizePose(double wrist_offset);

    ros::NodeHandle n, pnh;

    // topics
    ros::Subscriber cloud_subscriber;
    ros::Publisher palm_debug;
    ros::Publisher l_debug;
    ros::Publisher r_debug;
    ros::Publisher crop_debug;
    ros::Publisher object_cloud_debug;
    ros::Publisher object_pose_debug;

    // actionlib
    actionlib::SimpleActionServer<manipulation_actions::InHandLocalizeAction> in_hand_localization_server;
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> point_head_client;

    boost::mutex cloud_mutex;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    bool cloud_received;

    // MoveIt interfaces
    moveit::planning_interface::MoveGroupInterface *arm_group;
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;

    bool attach_arbitrary_object;

    // TF
    tf2_ros::TransformBroadcaster tf_broadcaster;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    geometry_msgs::TransformStamped wrist_object_tf;
    bool transform_set;

    // object pose (for debugging, this will also be added to the tf tree)
    geometry_msgs::PoseStamped object_pose;

    sensor_msgs::JointState localize_pose;
    geometry_msgs::Vector3 finger_dims;
    geometry_msgs::Vector3 palm_dims;
    double padding;
    int num_views;

    bool debug;
};

#endif // IN_HAND_LOCALIZER_H