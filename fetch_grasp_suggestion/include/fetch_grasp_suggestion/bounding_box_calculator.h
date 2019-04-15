#ifndef FETCH_GRASP_SUGGESTION_BOUNDING_BOX_H
#define FETCH_GRASP_SUGGESTION_BOUNDING_BOX_H

// ROS
#include <fetch_grasp_suggestion/BoundingBox.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief Static functions to compute point cloud bounding boxes for pick-and-place.
 */
class BoundingBoxCalculator
{

public:

  /**
   * @brief Fit a z-axis-aligned bounding box to x-y plane principal direction of point cloud.
   * @param cloud point cloud to bound
   * @return computed bounding box
   */
  static fetch_grasp_suggestion::BoundingBox computeBoundingBox(sensor_msgs::PointCloud2 cloud);

  /**
   * @brief Fit a z-axis-aligned bounding box to x-y plane principal direction of point cloud.
   * @param cloud point cloud to bound
   * @return computed bounding box
   */
  static fetch_grasp_suggestion::BoundingBox computeBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

  /**
   * @brief Fit a z-axis-aligned bounding box to x-y plane principal direction of point cloud.
   * @param cloud point cloud to bound
   * @return computed bounding box
   */
  static fetch_grasp_suggestion::BoundingBox computeBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif  // FETCH_GRASP_SUGGESTION_BOUNDING_BOX_H
