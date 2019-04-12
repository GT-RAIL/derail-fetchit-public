#ifndef FETCH_GRASP_SUGGESTION_COMMON_H
#define FETCH_GRASP_SUGGESTION_COMMON_H

// C++
#include <cmath>
#include <string>
#include <sstream>
#include <vector>

// Eigen
#include <Eigen/Dense>

// ROS
#include <fetch_grasp_suggestion/bounding_box_calculator.h>
#include <geometry_msgs/Point.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

class Common
{

public:

  /**
   * @brief Combine two grasp feature vectors into pairwise feature vector (as a csv entry) with object context.
   * @param object_features contextual feature vector calculated from the object-of-interest
   * @param hi feature vector of first grasp's heuristics
   * @param hj feature vector of second grasp's heuristics
   * @param positive true for a positive example (hi should be ranked above hj), false for a negative example
   * @return csv line for the pairwise feature vector
   */
  static std::string createTrainingInstance(std::vector<double> object_features, std::vector<double> hi,
      std::vector<double> hj, bool positive);

  /**
   * @brief Combine two grasps into a single c++ feature vector with object context.
   * @param object_features contextual feature vector calculated from the object-of-interest
   * @param hi feature vector of first grasp's heuristics
   * @param hj feature vector of second grasp's heuristics
   * @return pairwise feature vector
   */
  static std::vector<double> createTrainingVector(std::vector<double> object_features, std::vector<double> hi,
      std::vector<double> hj);

  /**
   * @brief Convert an rgb colorspace vector into a CIELAB colorspace vector.
   * @param colorRGB vector in rgb colorspace to be converted
   * @return vector in CIELAB colorspace
   */
  static Eigen::Vector3f RGB2Lab(const Eigen::Vector3f& colorRGB);

  /**
   * @brief Calculate features from an object point cloud.
   * @param cloud object point cloud
   * @return feature vector describing the object
   */
  static std::vector<double> calculateObjectFeatures(const sensor_msgs::PointCloud2 &cloud);

  /**
   * @brief Calculate features from an object point cloud.
   * @param cloud object point cloud
   * @return feature vector describing the object
   */
  static std::vector<double> calculateObjectFeatures(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

  /**
   * @brief Calculate features in a local region of an unsegmented point cloud.
   * @param cloud object point cloud
   * @param point center point of local region
   * @return feature vector describing the local region
   */
  static std::vector<double> calculateLocalFeatures(const sensor_msgs::PointCloud2 &cloud, geometry_msgs::Point point);
};

#endif  // FETCH_GRASP_SUGGESTION_COMMON_H
