#include <fetch_grasp_suggestion/retriever.h>

using std::ios;
using std::string;
using std::stringstream;
using std::vector;


Retriever::Retriever() :
  pn_("~"),
  cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
{
  string cloud_topic;
  pn_.param<string>("cloud_topic", cloud_topic, "head_camera/depth_registered/points");
  pn_.param<string>("desired_grasp_frame", desired_grasp_frame_, "base_link");
  pn_.param<double>("min_grasp_depth", min_grasp_depth_, -0.03);
  pn_.param<double>("max_grasp_depth", max_grasp_depth_, 0.03);

  cloud_received_ = false;

  debug_pub_ = pn_.advertise<geometry_msgs::PoseArray>("debug_poses", 10);
  segmentation_sub_ = n_.subscribe("/rail_segmentation/segmented_objects", 1, &Retriever::segmentCallback, this);
  cloud_subscriber_ = n_.subscribe(cloud_topic, 1, &Retriever::cloudCallback, this);
  retrieve_grasps_service_ = pn_.advertiseService("retrieve_grasps", &Retriever::retrieveGraspsCallback, this);
}


bool Retriever::retrieveGraspsCallback(fetch_grasp_suggestion::RetrieveGrasps::Request &req,
    fetch_grasp_suggestion::RetrieveGrasps::Response &res)
{
  // Check if we have all the data that we need
  if (!cloud_received_)
  {
    ROS_WARN("No point cloud data received!");
    return false;
  }

  rail_manipulation_msgs::SegmentedObject object = segmented_objects_.objects[req.object_idx];

  // Check the type of object that we're sampling grasps for and sample there. If this is an unrecognized
  // object type then error out. For the sampling process, we use the position of the centroid and then subtract
  // half of the 'Z' dimension
  if (req.type.object == manipulation_actions::ChallengeObject::LARGE_GEAR)
  {
//    enumerateLargeGearGrasps(req.object, res.grasp_list);
    enumerateLargeGearGrasps(object, res.grasp_list);
  }
  else if (req.type.object == manipulation_actions::ChallengeObject::SMALL_GEAR)
  {
//    enumerateSmallGearGrasps(req.object, res.grasp_list);
    enumerateSmallGearGrasps(object, res.grasp_list);
  }
  else
  {
    ROS_WARN_STREAM("Cannot retrieve grasps on object of type " << req.type.object);
    return false;
  }

  debug_pub_.publish(res.grasp_list);

  // Done. Return the grasps
  return true;
}

void Retriever::enumerateLargeGearGrasps(const rail_manipulation_msgs::SegmentedObject &object,
    geometry_msgs::PoseArray &grasps_out)
{
  // First transform to base_link if the object's bounding box is not already in base_link
  geometry_msgs::PoseStamped center_pose = object.bounding_volume.pose;
  center_pose.pose.position.z -= (object.bounding_volume.dimensions.z / 2);
  if (center_pose.header.frame_id != desired_grasp_frame_)
  {
    // I really hope this doesn't throw an error...
    tf_listener_.transformPose(desired_grasp_frame_, ros::Time(0), center_pose, desired_grasp_frame_,
        center_pose);
    center_pose.header.frame_id = desired_grasp_frame_;
  }
  grasps_out.header.frame_id = desired_grasp_frame_;

  // Now enumerate all the grasps
  double pitch_angle_increment = M_PI_2 / 6;  // 15 degrees
  double yaw_angle_increment = M_PI_2 / 6;    // 15 degrees
  for (int i = 0; i < 4; i++)
  {
    double p = 0 + (i * pitch_angle_increment);
    for (int j = 0; j < 3; j++)
    {
      double y = M_PI / 6 + (j * yaw_angle_increment);

      // Add the positive pose
      geometry_msgs::Pose grasp = center_pose.pose;
      tf2::Quaternion pitch, yaw, center;
      yaw.setRPY(0, M_PI, y);
      pitch.setRPY(M_PI_2, p, 0);
      tf2::fromMsg(center_pose.pose.orientation, center);
      tf2::Quaternion rotation = center * yaw * pitch;
      rotation.normalize();
      grasp.orientation = tf2::toMsg(rotation);
      grasps_out.poses.emplace_back(grasp);

      // Add the negative pose, if one exists
      if (p != 0)
      {
        pitch.setRPY(M_PI_2, -p, 0);
        rotation = center * yaw * pitch;
        rotation.normalize();
        grasp.orientation = tf2::toMsg(rotation);
        grasps_out.poses.emplace_back(grasp);
      }
    }
  }
}

void Retriever::enumerateSmallGearGrasps(const rail_manipulation_msgs::SegmentedObject &object,
    geometry_msgs::PoseArray &grasps_out)
{

}

void Retriever::cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg)
{
  boost::mutex::scoped_lock lock(cloud_mutex_);
  *cloud_ = *msg;
  cloud_received_ = true;
}

void Retriever::segmentCallback(const rail_manipulation_msgs::SegmentedObjectList &msg)
{
  segmented_objects_ = msg;
}


// Helper functions
geometry_msgs::Pose Retriever::adjustGraspDepth(geometry_msgs::Pose grasp_pose, double distance)
{
  geometry_msgs::Pose result;
  result.orientation.x = grasp_pose.orientation.x;
  result.orientation.y = grasp_pose.orientation.y;
  result.orientation.z = grasp_pose.orientation.z;
  result.orientation.w = grasp_pose.orientation.w;

  geometry_msgs::Transform transform;
  Eigen::Affine3d transform_matrix;
  transform.rotation = grasp_pose.orientation;
  transform.translation.x = grasp_pose.position.x;
  transform.translation.y = grasp_pose.position.y;
  transform.translation.z = grasp_pose.position.z;
  tf::transformMsgToEigen(transform, transform_matrix);

  Eigen::Vector3d transform_point, transformed_point;
  transform_point[0] = distance;
  transform_point[1] = 0;
  transform_point[2] = 0;
  transformed_point = transform_matrix*transform_point;
  tf::pointEigenToMsg(transformed_point, result.position);

  return result;
}

bool Retriever::isInCollision(geometry_msgs::PoseStamped grasp, sensor_msgs::PointCloud2 cloud, bool check_palm)
{
  // convert to pcl point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(cloud, *temp_cloud);
  pcl::fromPCLPointCloud2(*temp_cloud, *cloud_pcl);

  return isInCollision(grasp, cloud_pcl, check_palm);
}

bool Retriever::isInCollision(geometry_msgs::PoseStamped grasp, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    bool check_palm)
{
  //TODO(enhancement): finger tip size, shape, and position are all hardcoded for Fetch
  pcl::CropBox<pcl::PointXYZRGB> crop_box;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr collision_points(new pcl::PointCloud<pcl::PointXYZRGB>);
  Eigen::Vector4f min_point, max_point;
  crop_box.setInputCloud(cloud);

  geometry_msgs::PoseStamped check_grasp = grasp;
  if (grasp.header.frame_id != cloud->header.frame_id)
  {
    // transform grasp pose to point cloud frame
    tf_listener_.transformPose(cloud->header.frame_id, ros::Time(0), grasp, cloud->header.frame_id, check_grasp);
    check_grasp.header.frame_id = cloud->header.frame_id;
  }

  geometry_msgs::Transform transform;
  Eigen::Affine3d transform_matrix;
  transform.rotation = check_grasp.pose.orientation;
  transform.translation.x = check_grasp.pose.position.x;
  transform.translation.y = check_grasp.pose.position.y;
  transform.translation.z = check_grasp.pose.position.z;
  tf::transformMsgToEigen(transform, transform_matrix);

  Eigen::Vector3d transform_point;
  Eigen::Vector3d transformed_point;
  tf::Quaternion rotation_tf;
  double r, p, y;
  Eigen::Vector3f rotation;
  geometry_msgs::PoseStamped test_pose;
  visualization_msgs::Marker test_marker;

  // left finger
  min_point[0] = -0.029f;
  max_point[0] = 0.029f;
  min_point[1] = 0;
  max_point[1] = 0.014f;
  min_point[2] = -0.013f;
  max_point[2] = 0.013f;
  crop_box.setMin(min_point);
  crop_box.setMax(max_point);

  transform_point[0] = 0;
  transform_point[1] = -0.065f;
  transform_point[2] = 0;
  transformed_point = transform_matrix*transform_point;
  crop_box.setTranslation(transformed_point.cast<float>());

  tf::quaternionMsgToTF(check_grasp.pose.orientation, rotation_tf);
  tf::Matrix3x3(rotation_tf).getRPY(r, p, y);
  rotation[0] = static_cast<float>(r);
  rotation[1] = static_cast<float>(p);
  rotation[2] = static_cast<float>(y);
  crop_box.setRotation(rotation);

  crop_box.filter(*collision_points);
  if (!collision_points->points.empty())
    return true;

  // right finger
  min_point[1] = -0.014f;
  max_point[1] = 0;
  crop_box.setMin(min_point);
  crop_box.setMax(max_point);

  transform_point[1] = 0.065f;
  transformed_point = transform_matrix*transform_point;
  crop_box.setTranslation(transformed_point.cast<float>());

  crop_box.filter(*collision_points);
  if (!collision_points->points.empty())
    return true;

  if (check_palm)
  {
    min_point[0] = 0;
    max_point[0] = 0.137f;
    min_point[1] = -0.059f;
    max_point[1] = 0.059f;
    min_point[2] = -0.035f;
    max_point[2] = 0.035f;
    crop_box.setMin(min_point);
    crop_box.setMax(max_point);

    transform_point[0] = -0.166;
    transform_point[1] = 0;
    transform_point[2] = 0;
    transformed_point = transform_matrix*transform_point;
    crop_box.setTranslation(transformed_point.cast<float>());

    crop_box.filter(*collision_points);
    if (!collision_points->points.empty())
      return true;
  }

  return false;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "retriever");

  Retriever r;

  ros::spin();

  return EXIT_SUCCESS;
}
