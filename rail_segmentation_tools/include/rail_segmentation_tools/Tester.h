#ifndef RAIL_SEGMENTATION_TOOLS_TESTER_H_
#define RAIL_SEGMENTATION_TOOLS_TESTER_H_

// ROS
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <rail_manipulation_msgs/ProcessSegmentedObjects.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <rail_manipulation_msgs/SegmentObjects.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

// C++ Standard Library
#include <fstream>
#include <string>

class Tester
{
public:
  Tester();

  void testAll();

private:
  double merge_dst;
  double l_delta;

  ros::NodeHandle n;
  ros::ServiceClient segment_client;
  ros::ServiceClient merge_client;
};

#endif
