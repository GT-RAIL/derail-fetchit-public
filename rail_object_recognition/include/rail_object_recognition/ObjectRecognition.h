#include<ros/ros.h>
#include<pcl/conversions.h>
#include<pcl/features/esf.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/common.h>
#include<iostream>
#include<string>
#include<sensor_msgs/PointCloud2.h>
#include "rail_object_recognition/PartsQuery.h"
#include "rail_object_recognition/Descriptor.h"
#include "rail_object_recognition/ExtractPointCloud.h"

class ObjectRecognition{
	public:
		ObjectRecognition(ros::NodeHandle* nodehandle);
	private:
		ros::NodeHandle nh_;
		ros::ServiceServer object_recognition_service_;
		ros::ServiceClient parts_classifier_client_;	

		void initializeServices();
		bool serviceCallback(rail_object_recognition::ExtractPointCloud::Request &req, rail_object_recognition::ExtractPointCloud::Response &res);
};
