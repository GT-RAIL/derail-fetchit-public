#include <manipulation_actions/ClutteredGrasper.h>

using std::ios;
using std::string;
using std::stringstream;
using std::vector;

ClutteredGrasper::ClutteredGrasper() :
    pnh_("~"),
    cloud_(new pcl::PointCloud<pcl::PointXYZRGB>),
    sample_grasps_client_("/rail_agile/sample_grasps")
{
  string segmentation_topic, cloud_topic;
  pnh_.param<string>("segmentation_topic", segmentation_topic, "rail_segmentation/segmented_objects");
  pnh_.param<string>("cloud_topic", cloud_topic, "head_camera/depth_registered/points");
  pnh_.param<double>("box_dim_x", box_dims.x, 0.17);
  pnh_.param<double>("box_dim_y", box_dims.y, 0.17);
  pnh_.param<double>("box_error_threshold", box_error_threshold, 0.05);

  cloud_received_ = false;

  cloud_subscriber_ = n_.subscribe(cloud_topic, 1, &ClutteredGrasper::cloudCallback, this);
  objects_subscriber_ = n_.subscribe(segmentation_topic, 1, &ClutteredGrasper::objectsCallback, this);
}

void ClutteredGrasper::cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg)
{
  boost::mutex::scoped_lock lock(cloud_mutex_);

  *cloud_ = *msg;

  cloud_received_ = true;
}

void ClutteredGrasper::SampleGraspCandidates(sensor_msgs::PointCloud2 object, string object_source_frame,
    string environment_source_frame, geometry_msgs::PoseArray &grasps_out,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out)
{
  //get a pcl version of the object point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(object, *temp_cloud);
  pcl::fromPCLPointCloud2(*temp_cloud, *object_cloud);

  //transform object cloud to camera frame to get new crop box dimensions
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (object_cloud->header.frame_id != environment_source_frame)
  {
    pcl_ros::transformPointCloud(environment_source_frame, ros::Time(0), *object_cloud, object_source_frame,
                                 *transformed_cloud, tf_listener_);
    transformed_cloud->header.frame_id = environment_source_frame;
  }
  else
  {
    *transformed_cloud = *object_cloud;
  }

  //calculate workspace bounds in new coordinate frame
  pcl::PointXYZRGB min_workspace_point, max_workspace_point;
  pcl::getMinMax3D(*transformed_cloud, min_workspace_point, max_workspace_point);

  //crop cloud based on specified object
  //set padding to a negative number to not calculate grasps on the box edge
  double cloud_padding = -0.02;
  pcl::CropBox<pcl::PointXYZRGB> crop_box;
  Eigen::Vector4f min_point, max_point;
  min_point[0] = static_cast<float>(min_workspace_point.x - cloud_padding);
  min_point[1] = static_cast<float>(min_workspace_point.y - cloud_padding);
  min_point[2] = static_cast<float>(min_workspace_point.z - cloud_padding);
  max_point[0] = static_cast<float>(max_workspace_point.x + cloud_padding);
  max_point[1] = static_cast<float>(max_workspace_point.y + cloud_padding);
  max_point[2] = static_cast<float>(max_workspace_point.z + cloud_padding);
  crop_box.setMin(min_point);
  crop_box.setMax(max_point);
  crop_box.setInputCloud(cloud_);
  crop_box.filter(*cloud_out);

  rail_grasp_calculation_msgs::SampleGraspsGoal sample_goal;
  pcl::toPCLPointCloud2(*cloud_out, *temp_cloud);
  pcl_conversions::fromPCL(*temp_cloud, sample_goal.cloud);

  sample_goal.workspace.mode = rail_grasp_calculation_msgs::Workspace::WORKSPACE_VOLUME;
  sample_goal.workspace.x_min = min_point[0];
  sample_goal.workspace.y_min = min_point[1];
  sample_goal.workspace.z_min = min_point[2];
  sample_goal.workspace.x_max = max_point[0];
  sample_goal.workspace.y_max = max_point[1];
  sample_goal.workspace.z_max = max_point[2];

  sample_grasps_client_.sendGoal(sample_goal);
  sample_grasps_client_.waitForResult(ros::Duration(10.0));
  grasps_out = sample_grasps_client_.getResult()->graspList;
}

void ClutteredGrasper::objectsCallback(const rail_manipulation_msgs::SegmentedObjectList &list)
{
  boost::mutex::scoped_lock lock(object_list_mutex_);

  object_list_ = list;

  // for testing, find the screw box and execute grasp calculation
  size_t box_index = object_list_.objects.size();
  double min_error = std::numeric_limits<double>::max();
  for (size_t i = 0; i < object_list_.objects.size(); i ++)
  {
    //TODO: check on rail_segmentation's bounding box calculation ordering of (x,y,z) dims; also this should be
    // documented somewhere!
    double obj_x = object_list_.objects[i].bounding_volume.dimensions.y;
    double obj_y = object_list_.objects[i].bounding_volume.dimensions.z;
    if (obj_y > obj_x)
    {
      double temp = obj_x;
      obj_x = obj_y;
      obj_y = temp;
    }
    double error = fabs(obj_x - box_dims.x) + fabs(obj_y - box_dims.y);
    std::cout << "Object " << i << ": " << object_list_.objects[i].bounding_volume.dimensions.x << ", " <<
    object_list_.objects[i].bounding_volume.dimensions.y << ", " << object_list_.objects[i].bounding_volume.dimensions.z << std::endl;
    if (error <= box_error_threshold && error < min_error)
    {
      min_error = error;
      box_index = i;
    }
  }

  if (box_index < object_list_.objects.size())
  {
    ROS_INFO("Screw box found at index %lu", box_index);
  }
  else
  {
    ROS_INFO("Screw box not found among segmented objects.");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cluttered_grasper");

  ClutteredGrasper cg;

  ros::spin();

  return EXIT_SUCCESS;
}