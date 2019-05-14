#include <fetch_grasp_suggestion/retriever.h>

using std::ios;
using std::string;
using std::stringstream;
using std::vector;


Retriever::Retriever() :
  pn_("~"),
  tf_listener_(tf_buffer_)
{
  pn_.param<string>("cloud_topic", cloud_topic_, "head_camera/depth_registered/points");
  pn_.param<string>("desired_grasp_frame", desired_grasp_frame_, "base_link");
  pn_.param<double>("min_grasp_depth", min_grasp_depth_, -0.03);
  pn_.param<double>("max_grasp_depth", max_grasp_depth_, 0.03);
  pn_.param<bool>("debug", debug_, true);

  debug_pub_ = pn_.advertise<geometry_msgs::PoseArray>("debug_poses", 10);
  retrieve_grasps_service_ = pn_.advertiseService("retrieve_grasps", &Retriever::retrieveGraspsCallback, this);

  // TODO: Remove when finished developing. Allows an easier service call in the CLI
//  segmentation_sub_ = n_.subscribe("/rail_segmentation/segmented_objects", 1, &Retriever::segmentCallback, this);

  // default trivial initial transform at the gripper link to keep tf happy
  grasp_calculation_tf_.header.frame_id = desired_grasp_frame_;
  grasp_calculation_tf_.child_frame_id = "grasp_calculation_frame";
  grasp_calculation_tf_.transform.rotation.w = 1.0;
  grasp_calculation_tf_.header.stamp = ros::Time::now();
}

void Retriever::publishTF()
{
  grasp_calculation_tf_.header.stamp = ros::Time::now();
  tf_broadcaster_.sendTransform(grasp_calculation_tf_);
}

bool Retriever::retrieveGraspsCallback(fetch_grasp_suggestion::RetrieveGrasps::Request &req,
    fetch_grasp_suggestion::RetrieveGrasps::Response &res)
{
//  rail_manipulation_msgs::SegmentedObject object = segmented_objects_.objects[req.object_idx];

  // Check the type of object that we're sampling grasps for and sample there. If this is an unrecognized
  // object type then error out
  if (req.type.object == manipulation_actions::ChallengeObject::LARGE_GEAR)
  {
    enumerateLargeGearGrasps(req.object, res.grasp_list);
//    enumerateLargeGearGrasps(object, res.grasp_list);
  }
  else if (req.type.object == manipulation_actions::ChallengeObject::SMALL_GEAR)
  {
    enumerateSmallGearGrasps(req.object, res.grasp_list);
//    enumerateSmallGearGrasps(object, res.grasp_list);
  }
  else
  {
    ROS_WARN("Cannot retrieve grasps on object of type: %d", req.type.object);
    return false;
  }
  ROS_INFO("Enumerated %lu grasps", res.grasp_list.poses.size());

  // Prune out all those grasps that would lead to a collision. Also, figure out a grasp depth
  if (!res.grasp_list.poses.empty())
  {
    for (int i = res.grasp_list.poses.size() - 1; i >= 0; i--)
    {
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header = res.grasp_list.header;
      pose_stamped.pose = res.grasp_list.poses[i];
      if (isInCollision(pose_stamped, req.object.point_cloud, false))
//      if (isInCollision(pose_stamped, object.point_cloud, false))
      {
        res.grasp_list.poses.erase(res.grasp_list.poses.begin() + i);
      }
    }
  }
  ROS_INFO("%lu grasps remain after collision checking", res.grasp_list.poses.size());

  // get the current point cloud (for collision checking)
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc_msg = ros::topic::waitForMessage< pcl::PointCloud<pcl::PointXYZRGB> >
      (cloud_topic_, n_, ros::Duration(10.0));
  if (pc_msg == NULL)
  {
    ROS_INFO("No point cloud received for segmentation.");
    return false;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);
  *pc = *pc_msg;

  // Then calculate the grasp depth
  for (int i = 0; i < res.grasp_list.poses.size(); i++)
  {
    double depth_lower_bound = min_grasp_depth_;
    double depth_upper_bound = max_grasp_depth_;
    double current_depth = max_grasp_depth_;
    geometry_msgs::PoseStamped test_pose;
    test_pose.header = res.grasp_list.header;
    test_pose.pose = res.grasp_list.poses[i];

    // check the max grasp depth first
    test_pose.pose = adjustGraspDepth(res.grasp_list.poses[i], current_depth);
    if (isInCollision(test_pose, pc, true))
    {
      geometry_msgs::Pose adjusted_pose;
      //binary search for the pose
      for (int j = 0; j < 5; j++)
      {
        current_depth = (depth_lower_bound + depth_upper_bound) / 2.0;
        adjusted_pose = adjustGraspDepth(res.grasp_list.poses[i], current_depth);
        test_pose.pose.position = adjusted_pose.position;
        if (isInCollision(test_pose, pc, true))
        {
          depth_upper_bound = current_depth;
        }
        else
        {
          depth_lower_bound = current_depth;
        }
      }
    }

    // Store the pose depth
    res.grasp_list.poses[i].position = test_pose.pose.position;
  }

  if (debug_)
  {
    debug_pub_.publish(res.grasp_list);
  }

  // Done. Return the grasps
  return true;
}

void Retriever::enumerateLargeGearGrasps(const rail_manipulation_msgs::SegmentedObject &object,
    geometry_msgs::PoseArray &grasps_out)
{
  // First get the pose of the gear
  geometry_msgs::PoseStamped center_pose = object.bounding_volume.pose;
  calculateLargeGearPose(object, center_pose);

  if (center_pose.header.frame_id != desired_grasp_frame_)
  {
    geometry_msgs::PoseStamped old_center = center_pose;
    geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(desired_grasp_frame_,
        center_pose.header.frame_id, ros::Time(0));
    tf2::doTransform(old_center, center_pose, transform);
  }

  // output the tf frame
  grasp_calculation_tf_.transform.translation.x = center_pose.pose.position.x;
  grasp_calculation_tf_.transform.translation.y = center_pose.pose.position.y;
  grasp_calculation_tf_.transform.translation.z = center_pose.pose.position.z;
  grasp_calculation_tf_.transform.rotation = center_pose.pose.orientation;

  // Also set the output of the grasps
  grasps_out.header.frame_id = desired_grasp_frame_;

  // Set the center pose to the center of the base of the gear
  geometry_msgs::PoseStamped base_center;
  base_center.header.frame_id = grasp_calculation_tf_.child_frame_id;
  base_center.pose.position.x = -fmax(object.bounding_volume.dimensions.z, fmax(object.bounding_volume.dimensions.x,
      object.bounding_volume.dimensions.y)) / 2.0 + .02;
  base_center.pose.orientation.w = 1;
  tf2::doTransform(base_center, center_pose, grasp_calculation_tf_);

  // Now enumerate all the grasps
  double yaw_angle_increment = M_PI / 6;  // 30 degrees
  double pitch_angle_increment = M_PI / 12;   // 15 degrees
  for (int i = 0; i < 7; i++)
  {
    double y = 0 + (i * yaw_angle_increment);  // start at 0
    for (int j = 0; j < 2; j++)
    {
      double p = 0 + (j * pitch_angle_increment);  // start at 0

      // Add the positive pose
      geometry_msgs::Pose grasp = center_pose.pose;
      tf2::Quaternion pitch, yaw, center, rotation;
      tf2::fromMsg(center_pose.pose.orientation, center);
      rotation.setRPY(0, M_PI_2, 0);
      rotation = center * rotation;

      yaw.setRPY(0, 0, y);
      rotation = rotation * yaw;

      pitch.setRPY(0, p, 0);
      rotation = rotation * pitch;
      rotation.normalize();

      grasp.orientation = tf2::toMsg(rotation);
      grasps_out.poses.emplace_back(grasp);

      // Add the negative pose, if one exists
      if (y != 0 || p != 0)
      {
        if (y != 0)
        {
          rotation.setRPY(0, M_PI_2, 0);
          yaw.setRPY(0, 0, -y);
          rotation = center * rotation * yaw * pitch;
          rotation.normalize();
          grasp.orientation = tf2::toMsg(rotation);
          grasps_out.poses.emplace_back(grasp);
        }
        if (p != 0)
        {
          rotation.setRPY(0, M_PI_2, 0);
          pitch.setRPY(0, -p, 0);
          rotation = center * rotation * yaw * pitch;
          rotation.normalize();
          grasp.orientation = tf2::toMsg(rotation);
          grasps_out.poses.emplace_back(grasp);
        }
        if (y != 0 && p != 0)
        {
          rotation.setRPY(0, M_PI_2, 0);
          yaw.setRPY(0, 0, y);
          rotation = center * rotation * yaw * pitch;
          rotation.normalize();
          grasp.orientation = tf2::toMsg(rotation);
          grasps_out.poses.emplace_back(grasp);
        }
      }
    }
  }

  vector<ScoredPose> sorted_poses;
  // rank grasps according to orientation
  if (object.bounding_volume.dimensions.x > .075)
  {
    // vertical case
    ROS_INFO("Ranking grasps for VERTICAL large gear");
    for (size_t i = 0; i < grasps_out.poses.size(); i ++)
    {
      geometry_msgs::PoseStamped candidate;
      candidate.header.frame_id = grasps_out.header.frame_id;
      candidate.pose.position = grasps_out.poses[i].position;
      candidate.pose.orientation = grasps_out.poses[i].orientation;
      // rank by distance of approach pose from robot, filter out grasps with upward angles
      tf2::Quaternion q;
      tf2::fromMsg(candidate.pose.orientation, q);

      // scoring with respect to "downward pointing"
      tf2::Vector3 gravity_vector(0, 0, -1);
      tf2::Matrix3x3 rotation_mat(q);
      tf2::Vector3 x_vector(1, 0, 0);
      tf2::Vector3 pose_x_vector = rotation_mat * x_vector;
      double filter_score = acos(pose_x_vector.dot(gravity_vector));

      if (filter_score < (M_PI_2 - M_PI/24.0))  // only take poses that are pointing at a downward angle
      {
        // scoring with respect to yaw angle
        double score = acos(pose_x_vector.dot(x_vector));
        sorted_poses.emplace_back(ScoredPose(candidate, score));
      }
    }
  }
  else
  {
    ROS_INFO("Ranking grasps for HORIZONTAL large gear");
    // horizontal case
    for (size_t i = 0; i < grasps_out.poses.size(); i ++)
    {
      geometry_msgs::PoseStamped candidate;
      candidate.header.frame_id = grasps_out.header.frame_id;
      candidate.pose.position = grasps_out.poses[i].position;
      candidate.pose.orientation = grasps_out.poses[i].orientation;
      // rank by distance of approach pose from robot, filter out grasps with upward angles
      tf2::Quaternion q;
      tf2::fromMsg(candidate.pose.orientation, q);

      // scoring with respect to "downward pointing"
      tf2::Vector3 gravity_vector(0, 0, -1);
      tf2::Matrix3x3 rotation_mat(q);
      tf2::Vector3 x_vector(1, 0, 0);
      tf2::Vector3 pose_x_vector = rotation_mat * x_vector;
      double score = acos(pose_x_vector.dot(gravity_vector));

      if (score <= M_PI/6.0)  // only take poses that are pointing at a steep downward angle
      {
        sorted_poses.emplace_back(ScoredPose(candidate, score));
      }
    }
  }

  // sort poses (low scores are better)
  sort(sorted_poses.begin(), sorted_poses.end());
  grasps_out.poses.resize(sorted_poses.size());
  for (size_t i = 0; i < sorted_poses.size(); i ++)
  {
    grasps_out.poses[i].position = sorted_poses[i].pose.pose.position;
    grasps_out.poses[i].orientation = sorted_poses[i].pose.pose.orientation;
  }
}

void Retriever::enumerateSmallGearGrasps(const rail_manipulation_msgs::SegmentedObject &object,
    geometry_msgs::PoseArray &grasps_out)
{
  // First transform to base_link if the object's bounding box is not already in base_link
  geometry_msgs::PoseStamped center_pose = object.bounding_volume.pose;
  if (center_pose.header.frame_id != desired_grasp_frame_)
  {
    geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(desired_grasp_frame_,
                                                                           center_pose.header.frame_id, ros::Time(0));
    tf2::doTransform(object.bounding_volume.pose, center_pose, transform);
    center_pose.header.frame_id = desired_grasp_frame_;
    center_pose.header.frame_id = desired_grasp_frame_;
  }
  grasps_out.header.frame_id = desired_grasp_frame_;

  // Now enumerate all the grasps
  double roll_angle_increment = M_PI_2 / 2;   // 45 degrees
  double yaw_angle_increment = M_PI_2 / 6;  // 15 degrees
  for (int i = 0; i < 3; i++)
  {
    double y = i * yaw_angle_increment;

    for (int j = 0; j < 3; j++)
    {
      double r = 0 + (j * roll_angle_increment);

      // Add the positive pose
      geometry_msgs::Pose grasp = center_pose.pose;
      tf2::Quaternion roll, center;

      roll.setRPY(r, 0, M_PI + y);
      tf2::fromMsg(center_pose.pose.orientation, center);
      tf2::Quaternion rotation = center * roll;
      rotation.normalize();
      grasp.orientation = tf2::toMsg(rotation);
      grasps_out.poses.emplace_back(grasp);

      if (y != 0)
      {
        roll.setRPY(r, 0, M_PI - y);
        rotation = center * roll;
        rotation.normalize();
        grasp.orientation = tf2::toMsg(rotation);
        grasps_out.poses.emplace_back(grasp);
      }

      // Add the negative pose, if one exists
      if (r != 0)
      {
        roll.setRPY(-r, 0, M_PI + y);
        rotation = center * roll;
        rotation.normalize();
        grasp.orientation = tf2::toMsg(rotation);
        grasps_out.poses.emplace_back(grasp);

        if (y != 0)
        {
          roll.setRPY(-r, 0, M_PI - y);
          rotation = center * roll;
          rotation.normalize();
          grasp.orientation = tf2::toMsg(rotation);
          grasps_out.poses.emplace_back(grasp);
        }
      }
    }
  }
}

// TODO: Remove when finished developing. Allows an easier service call in the CLI
//void Retriever::segmentCallback(const rail_manipulation_msgs::SegmentedObjectList &msg)
//{
//  segmented_objects_ = msg;
//}


// Helper functions
void Retriever::calculateLargeGearPose(const rail_manipulation_msgs::SegmentedObject &object,
    geometry_msgs::PoseStamped &pose)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(object.point_cloud, *object_cloud);

  // calculate principle axes on cluster
  // compute principal direction
  Eigen::Matrix3f covariance;
  Eigen::Vector4f centroid;
  centroid[0] = object.centroid.x;
  centroid[1] = object.centroid.y;
  centroid[2] = object.centroid.z;
  pcl::computeCovarianceMatrixNormalized(*object_cloud, centroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eig_dx = eigen_solver.eigenvectors();
  eig_dx.col(2) = eig_dx.col(0).cross(eig_dx.col(1));

  // move the points to that reference frame
  Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
  p2w.block(0, 0, 3, 3) = eig_dx.transpose();
  p2w.block(0, 3, 3, 1) = -1.f * (p2w.block(0, 0, 3, 3) * centroid.head(3));
  pcl::PointCloud<pcl::PointXYZRGB> c_points;
  pcl::transformPointCloud(*object_cloud, c_points, p2w);

  // calculate transform
  pcl::PointXYZRGB min_pt, max_pt;
  pcl::getMinMax3D(c_points, min_pt, max_pt);
  const Eigen::Vector3f mean_diag = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());
  const Eigen::Quaternionf qfinal(eig_dx);
  const Eigen::Vector3f tfinal = eig_dx * mean_diag + centroid.head(3);

  tf::Vector3 tfinal_tf(tfinal[0], tfinal[1], tfinal[2]);
  tf::Quaternion qfinal_tf(qfinal.x(), qfinal.y(), qfinal.z(), qfinal.w());
  tf::Quaternion adjustment;
  adjustment.setRPY(0, -M_PI_2, 0);
  qfinal_tf *= adjustment;

  // set the transform
  geometry_msgs::Vector3 position_stub;
  tf::vector3TFToMsg(tfinal_tf, position_stub);
  pose.pose.position.x = position_stub.x;
  pose.pose.position.y = position_stub.y;
  pose.pose.position.z = position_stub.z;
  tf::quaternionTFToMsg(qfinal_tf, pose.pose.orientation);

  // Then check the number of points at each end in order to calculate a consistent X pose
  tf::Transform tf_transform;
  tf_transform.setRotation(qfinal_tf);
  tf_transform.setOrigin(tfinal_tf);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl_ros::transformPointCloud(*object_cloud, *cloud_transformed, tf_transform.inverse());

  pcl::PointXYZRGB min_dim, max_dim;
  pcl::getMinMax3D(*cloud_transformed, min_dim, max_dim);

  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud(cloud_transformed);

  pcl::PointXYZRGB base_point;
  pcl::PointXYZRGB tip_point;
  base_point.x = min_dim.x;
  base_point.y = 0;
  base_point.z = 0;
  tip_point.x = max_dim.x;
  tip_point.y = 0;
  tip_point.z = 0;

  // figure out which side of the x-axis has more points
  vector<int> indices;
  vector<float> sqr_dsts;
  kdtree.radiusSearch(base_point, 0.035, indices, sqr_dsts);
  size_t base_points = sqr_dsts.size();
  indices.clear();
  sqr_dsts.clear();
  kdtree.radiusSearch(tip_point, 0.035, indices, sqr_dsts);
  size_t tip_points = sqr_dsts.size();
  ROS_INFO("Tip points: %lu; base points: %lu", tip_points, base_points);

  if (tip_points > base_points)
  {
    ROS_INFO("Flipping transform.");
    // flip transform
    tf2::Quaternion tf_q(pose.pose.orientation.x, pose.pose.orientation.y,
                         pose.pose.orientation.z, pose.pose.orientation.w);
    tf2::Quaternion flip;
    flip.setRPY(0, 0, M_PI);
    tf_q = tf_q * flip;

    pose.pose.orientation.x = tf_q.x();
    pose.pose.orientation.y = tf_q.y();
    pose.pose.orientation.z = tf_q.z();
    pose.pose.orientation.w = tf_q.w();
  }
}

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
    geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(cloud->header.frame_id,
                                                                           grasp.header.frame_id, ros::Time(0));
    tf2::doTransform(grasp, check_grasp, transform);
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

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    r.publishTF();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}
