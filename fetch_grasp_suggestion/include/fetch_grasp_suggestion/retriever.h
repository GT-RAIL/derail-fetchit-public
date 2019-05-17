#ifndef FETCH_GRASP_SUGGESTION_RETREIVER_H
#define FETCH_GRASP_SUGGESTION_RETREIVER_H

// C++
#include <iostream>

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <fetch_grasp_suggestion/common.h>
#include <fetch_grasp_suggestion/RetrieveGrasps.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <rail_manipulation_msgs/SegmentedObject.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "fetch_grasp_suggestion/ScoredPose.h"


class Retriever
{
public:
    Retriever();

    void publishTF();

private:
    // Callback functions
    bool retrieveGraspsCallback(fetch_grasp_suggestion::RetrieveGrasps::Request &req,
        fetch_grasp_suggestion::RetrieveGrasps::Response &res);

    // Helper functions

    // Sample grasps based on the object
    // returns true if large gear is in vertical orientation, false otherwise
    bool enumerateLargeGearGrasps(const rail_manipulation_msgs::SegmentedObject &object,
        geometry_msgs::PoseArray &grasps_out);

    void enumerateSmallGearGrasps(const rail_manipulation_msgs::SegmentedObject &object,
        geometry_msgs::PoseArray &grasps_out);

    // Checks for the grasps
    geometry_msgs::Pose adjustGraspDepth(geometry_msgs::Pose grasp_pose, double distance);
    bool isInCollision(geometry_msgs::PoseStamped grasp, sensor_msgs::PointCloud2 cloud, bool check_palm);
    bool isInCollision(geometry_msgs::PoseStamped grasp, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, bool check_palm);

    // Calculate the object pose. Copied from InHandLocalizer
    void calculateLargeGearPose(const rail_manipulation_msgs::SegmentedObject &object, geometry_msgs::PoseStamped &pose);

    // Attributes
    ros::NodeHandle n_, pn_;

    // services
    ros::ServiceServer retrieve_grasps_service_;

    // Debug
    ros::Publisher debug_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher pose2_pub_;
    bool debug_;

    // TODO: Remove when finished developing. Allows an easier service call in the CLI
//    ros::Subscriber segmentation_sub_;
//    rail_manipulation_msgs::SegmentedObjectList segmented_objects_;
//    void segmentCallback(const rail_manipulation_msgs::SegmentedObjectList &msg);

    // TF
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    geometry_msgs::TransformStamped grasp_calculation_tf_;

    // Point Cloud
    std::string cloud_topic_;

    // Params that can be set
    double min_grasp_depth_, max_grasp_depth_;
    std::string desired_grasp_frame_;
};
#endif //FETCH_GRASP_SUGGESTION_RETREIVER_H
