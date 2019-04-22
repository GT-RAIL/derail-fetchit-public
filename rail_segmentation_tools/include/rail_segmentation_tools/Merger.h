#ifndef RAIL_SEGMENTATION_TOOLS_MERGER_H_
#define RAIL_SEGMENTATION_TOOLS_MERGER_H_

// ROS
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <rail_manipulation_msgs/ProcessSegmentedObjects.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <rail_manipulation_msgs/SegmentObjects.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/MarkerArray.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>

// C++ Standard Library
#include <fstream>
#include <string>

class Merger
{
public:
  static constexpr double DEFAULT_MERGE_DST = 0.05;
  static constexpr double DEFAULT_COLOR_DELTA = 10;

  Merger();

private:
  bool mergeCallback(rail_manipulation_msgs::ProcessSegmentedObjects::Request &req,
      rail_manipulation_msgs::ProcessSegmentedObjects::Response &res);

  double merge_dst;
  double color_delta;

  bool republish_segmented_objects;

  ros::NodeHandle n, pn;
  ros::Publisher segmented_objects_pub;
  ros::Publisher markers_pub;
  ros::ServiceServer merge_srv;
  ros::ServiceClient calculate_featuers_client;
};

#endif
