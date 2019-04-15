#ifndef FETCH_GRASP_SUGGESTION_POINT_CLOUD_MANIPULATION_H
#define FETCH_GRASP_SUGGESTION_POINT_CLOUD_MANIPULATION_H

// C++
#include <string>
#include <vector>

// ROS
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

class PointCloudManipulation
{

public:

  /**
   * @brief Transform a point cloud to a different frame.
   * @param cloud_in point cloud to be transformed
   * @param cloud_out point cloud object to return the transformed point cloud
   * @param frame coordinate frame to transform the point cloud to
   * @param tf_listener a tf listener object to perform the transform
   */
  static void transformPointCloud(const sensor_msgs::PointCloud2 &cloud_in, sensor_msgs::PointCloud2 &cloud_out,
      std::string frame, tf::TransformListener &tf_listener);

  /**
   * @brief Transform a point cloud to a different frame.
   * @param cloud_in point cloud to be transformed
   * @param cloud_out point cloud object to return the transformed point cloud
   * @param frame coordinate frame to transform the point cloud to
   * @param tf_listener a tf listener object to perform the transform
   */
  static void transformPointCloud(const sensor_msgs::PointCloud2 &cloud_in,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out, std::string frame, tf::TransformListener &tf_listener);

  /**
   * @brief Transform a point cloud to a different frame.
   * @param cloud_in point cloud to be transformed
   * @param cloud_out point cloud object to return the transformed point cloud
   * @param frame coordinate frame to transform the point cloud to
   * @param tf_listener a tf listener object to perform the transform
   */
  static void transformPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
      sensor_msgs::PointCloud2 &cloud_out, std::string frame, tf::TransformListener &tf_listener);

  /**
   * @brief Transform a point cloud to a different frame.
   * @param cloud_in point cloud to be transformed
   * @param cloud_out point cloud object to return the transformed point cloud
   * @param frame coordinate frame to transform the point cloud to
   * @param tf_listener a tf listener object to perform the transform
   */
  static void transformPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out, std::string frame, tf::TransformListener &tf_listener);

  /**
   * @brief Convert a point cloud from sensor_msgs::PointCloud2 point cloud to a pcl PointXYZRGB point cloud
   * @param cloud_in point cloud to be converted
   * @param cloud_out object to return the converted point cloud
   */
  static void fromSensorMsgs(const sensor_msgs::PointCloud2 &cloud_in,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);

  /**
   * @brief Convert a pcl PointXYZRGB point cloud to a sensor_msgs::PointCloud2 point cloud
   * @param cloud_in point cloud to be converted
   * @param cloud_out object to return the converted point cloud
   */
  static void toSensorMsgs(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
      sensor_msgs::PointCloud2 &cloud_out);
};

#endif  // FETCH_GRASP_SUGGESTION_POINT_CLOUD_MANIPULATION_H
