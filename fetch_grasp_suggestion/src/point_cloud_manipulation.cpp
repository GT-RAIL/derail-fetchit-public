#include <fetch_grasp_suggestion/point_cloud_manipulation.h>

using std::string;

void PointCloudManipulation::transformPointCloud(const sensor_msgs::PointCloud2 &cloud_in,
    sensor_msgs::PointCloud2 &cloud_out, std::string frame, tf::TransformListener &tf_listener)
{
  pcl_ros::transformPointCloud(frame, cloud_in, cloud_out, tf_listener);
  cloud_out.header.frame_id = frame;
}

void PointCloudManipulation::transformPointCloud(const sensor_msgs::PointCloud2 &cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out, std::string frame, tf::TransformListener &tf_listener)
{
  sensor_msgs::PointCloud2 temp_cloud_out;
  transformPointCloud(cloud_in, temp_cloud_out, frame, tf_listener);
  fromSensorMsgs(temp_cloud_out, cloud_out);
}

void PointCloudManipulation::transformPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
    sensor_msgs::PointCloud2 &cloud_out, std::string frame, tf::TransformListener &tf_listener)
{
  sensor_msgs::PointCloud2 temp_cloud_in;
  toSensorMsgs(cloud_in, temp_cloud_in);
  transformPointCloud(temp_cloud_in, cloud_out, frame, tf_listener);
}

void PointCloudManipulation::transformPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out, std::string frame, tf::TransformListener &tf_listener)
{
  sensor_msgs::PointCloud2 temp_cloud_in, temp_cloud_out;
  toSensorMsgs(cloud_in, temp_cloud_in);
  transformPointCloud(temp_cloud_in, temp_cloud_out, frame, tf_listener);
  fromSensorMsgs(temp_cloud_out, cloud_out);
}

void PointCloudManipulation::fromSensorMsgs(const sensor_msgs::PointCloud2 &cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
  pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(cloud_in, *temp_cloud);
  pcl::fromPCLPointCloud2(*temp_cloud, *cloud_out);
}

void PointCloudManipulation::toSensorMsgs(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
    sensor_msgs::PointCloud2 &cloud_out)
{
  pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(*cloud_in, *temp_cloud);
  pcl_conversions::fromPCL(*temp_cloud, cloud_out);
}
