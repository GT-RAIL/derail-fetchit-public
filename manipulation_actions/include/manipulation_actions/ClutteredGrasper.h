#ifndef CLUTTERED_GRASPER_H
#define CLUTTERED_GRASPER_H

// C++
#include <fstream>
#include <iostream>

// Boost
#include <boost/thread/mutex.hpp>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <rail_grasp_calculation_msgs/RankGraspsAction.h>
#include <rail_grasp_calculation_msgs/SampleGraspsAction.h>
#include <rail_manipulation_msgs/GraspFeedback.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <rail_manipulation_msgs/SuggestGrasps.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class ClutteredGrasper
{

public:
    ClutteredGrasper();

private:

    void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg);

    void objectsCallback(const rail_manipulation_msgs::SegmentedObjectList &list);

    /**
     * @brief Sample grasp candidates using rail_grasp_calculation.
     *
     * This is the first of two required steps for grasp ranking. A large set of potential grasps are sampled from the
     * environment point cloud in the area containing the object, using AGILE. This results in a set of unclustered,
     * unranked grasp poses where a two-fingered gripper can potentially close on the object.
     *
     * @param object point cloud of only the object-of-interest
     * @param object_source_frame tf frame of the object point cloud
     * @param environment_source_frame tf frame of the full scene point cloud
     * @param grasps_out unordered list of sampled grasp candidates
     * @param cloud_out cropped environment point cloud required for the next grasp ranking step
     */
    void SampleGraspCandidates(sensor_msgs::PointCloud2 object, std::string object_source_frame,
        std::string environment_source_frame, geometry_msgs::PoseArray &grasps_out,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out);

    ros::NodeHandle n_, pnh_;

    // topics
    ros::Publisher grasps_publisher_;
    ros::Subscriber cloud_subscriber_;
    ros::Subscriber objects_subscriber_;
    ros::Subscriber grasp_feedback_subscriber_;

    // actionlib
    actionlib::SimpleActionClient<rail_grasp_calculation_msgs::SampleGraspsAction> sample_grasps_client_;

    tf::TransformListener tf_listener_;

    boost::mutex cloud_mutex_;  /// mutex for full scene point cloud
    boost::mutex stored_grasp_mutex_;  /// mutex for suggested grasp list (service workflow)
    boost::mutex object_list_mutex_;  /// mutex for segmented object list (actionlib workflow)

    rail_manipulation_msgs::SegmentedObjectList object_list_;  /// stored segmented objects list (for actionlib workflow)

    sensor_msgs::PointCloud2 stored_object_cloud_;  /// most recent object-of-interest point cloud (for service workflow)
    sensor_msgs::PointCloud2 stored_scene_cloud_;  /// most recent scene point cloud (for unsegmented point cloud mode)
    std::vector<int> selected_grasps_;  /// previously selected grasps (as indices), to prevent conflicting training data
    std::vector<double> object_features_;  /// object features calculated for training instances, stored to save time
    double min_grasp_depth_, max_grasp_depth_;  /// bounds on grasp depth search

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;  /// stored full scene point cloud

    bool cloud_received_;  /// true once first point cloud is received

    geometry_msgs::Vector3 box_dims;
    double box_error_threshold;
};

#endif // CLUTTERED_GRASPER_H