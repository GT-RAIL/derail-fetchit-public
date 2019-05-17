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
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/PointHeadAction.h>
#include <eigen_conversions/eigen_msg.h>
#include <fetch_driver_msgs/GripperState.h>
#include <manipulation_actions/AttachToBase.h>
#include <manipulation_actions/InHandLocalizeAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_srvs/Empty.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class InHandLocalizer
{

public:
    InHandLocalizer();

    void publishTF();

private:

    void executeLocalize(const manipulation_actions::InHandLocalizeGoalConstPtr &goal);

    bool extractObjectCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &object_cloud);

    bool moveToLocalizePose(double wrist_offset, bool no_moveit=false);

    bool resetObjectFrame(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    void resetTransform();

    ros::NodeHandle n, pnh;

    // topics
    ros::Publisher palm_debug;
    ros::Publisher l_debug;
    ros::Publisher r_debug;
    ros::Publisher crop_debug;
    ros::Publisher object_cloud_debug;
    ros::Publisher object_pose_debug;

    ros::ServiceClient detach_objects_client;
    ros::ServiceClient attach_gripper_client;
    ros::ServiceServer reset_object_frame_server;

    // actionlib
    actionlib::SimpleActionServer<manipulation_actions::InHandLocalizeAction> in_hand_localization_server;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> point_head_client;

    boost::mutex transform_mutex;

    // MoveIt interfaces
    moveit::planning_interface::MoveGroupInterface *arm_group;
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;

    // non-MoveIt wrist spinning
    control_msgs::FollowJointTrajectoryGoal wrist_goal;

    std::string cloud_topic;

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

    double outlier_radius;
    double min_neighbors;
    double gear_angle_threshold;
    double gear_bad_grasp_threshold;

    bool debug;
};

#endif // IN_HAND_LOCALIZER_H
