#include <manipulation_actions/InHandLocalizer.h>

using std::ios;
using std::string;
using std::stringstream;
using std::vector;

InHandLocalizer::InHandLocalizer() :
    pnh("~"),
    cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
    tf_listener(tf_buffer),
    in_hand_localization_server(pnh, "localize", boost::bind(&InHandLocalizer::executeLocalize, this, _1), false),
    point_head_client("head_controller/point_head")
{
  string cloud_topic;
  pnh.param<string>("cloud_topic", cloud_topic, "head_camera/depth_registered/points");
  pnh.param<int>("num_views", num_views, 4);
  pnh.param<double>("finger_dims_x", finger_dims.x, 0.058);
  pnh.param<double>("finger_dims_y", finger_dims.y, 0.014);
  pnh.param<double>("finger_dims_z", finger_dims.z, 0.026);
  pnh.param<double>("palm_dims_x", palm_dims.x, 0.137);
  pnh.param<double>("palm_dims_y", palm_dims.y, 0.13);
  pnh.param<double>("palm_dims_z", palm_dims.z, 0.08);
  pnh.param<double>("padding", padding, 0.005);
  pnh.param<double>("outlier_radius", outlier_radius, 0.005);
  pnh.param<double>("min_neighbors", min_neighbors, 50);
  pnh.param<bool>("add_object", attach_arbitrary_object, false);
  pnh.param<bool>("debug", debug, true);

  //TODO: make this a param
  localize_pose.name.push_back("shoulder_pan_joint");
  localize_pose.name.push_back("shoulder_lift_joint");
  localize_pose.name.push_back("upperarm_roll_joint");
  localize_pose.name.push_back("elbow_flex_joint");
  localize_pose.name.push_back("forearm_roll_joint");
  localize_pose.name.push_back("wrist_flex_joint");
  localize_pose.name.push_back("wrist_roll_joint");

  // Halloween pose
//  localize_pose.position.push_back(1.47);
//  localize_pose.position.push_back(0.88);
//  localize_pose.position.push_back(1.80);
//  localize_pose.position.push_back(-1.67);
//  localize_pose.position.push_back(0.91);
//  localize_pose.position.push_back(-1.31);
//  localize_pose.position.push_back(-1.53);

  // Initial test pose
//  localize_pose.position.push_back(1.58);
//  localize_pose.position.push_back(0.60);
//  localize_pose.position.push_back(1.57);
//  localize_pose.position.push_back(-1.87);
//  localize_pose.position.push_back(0.63);
//  localize_pose.position.push_back(-1.38);
//  localize_pose.position.push_back(1.41);

  // Angled test pose
  localize_pose.position.push_back(1.13);
  localize_pose.position.push_back(0.61);
  localize_pose.position.push_back(1.53);
  localize_pose.position.push_back(-1.10);
  localize_pose.position.push_back(0.22);
  localize_pose.position.push_back(-1.86);
  localize_pose.position.push_back(-0.66);

  cloud_received = false;

  arm_group = new moveit::planning_interface::MoveGroupInterface("arm");
  arm_group->startStateMonitor();

  planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();

  cloud_subscriber = n.subscribe(cloud_topic, 1, &InHandLocalizer::cloudCallback, this);

  transform_set = false;

  if (debug)
  {
    palm_debug = pnh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("palm_debug", 1);
    l_debug = pnh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("l_debug", 1);
    r_debug = pnh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("r_debug", 1);
    crop_debug = pnh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("crop_debug", 1);
    object_cloud_debug = pnh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("object_cloud_debug", 1);
    object_pose_debug = pnh.advertise<geometry_msgs::PoseStamped>("object_pose_debug", 1);
  }

  in_hand_localization_server.start();
}

void InHandLocalizer::publishTF()
{
  if (transform_set)
  {
    wrist_object_tf.header.stamp = ros::Time::now();
    tf_broadcaster.sendTransform(wrist_object_tf);
    if (debug)
    {
      object_pose_debug.publish(object_pose);
    }
  }
}

void InHandLocalizer::cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg)
{
  boost::mutex::scoped_lock lock(cloud_mutex);

  *cloud = *msg;

  cloud_received = true;
}

void InHandLocalizer::executeLocalize(const manipulation_actions::InHandLocalizeGoalConstPtr &goal)
{
  manipulation_actions::InHandLocalizeResult result;

  if (attach_arbitrary_object)
  {
    // add an arbitrary object to planning scene for testing (typically this would be done at grasp time)
    vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);
    collision_objects[0].header.frame_id = "gripper_link";
    collision_objects[0].id = "arbitrary_gripper_object";
    shape_msgs::SolidPrimitive shape;
    shape.type = shape_msgs::SolidPrimitive::SPHERE;
    shape.dimensions.resize(1);
    shape.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = 0.15;
    collision_objects[0].primitives.push_back(shape);
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    collision_objects[0].primitive_poses.push_back(pose);
    planning_scene_interface->addCollisionObjects(collision_objects);

    ros::Duration(0.5).sleep();

    vector<string> touch_links;
    touch_links.emplace_back("r_gripper_finger_link");
    touch_links.emplace_back("l_gripper_finger_link");
    touch_links.emplace_back("gripper_link");
    touch_links.emplace_back("wrist_roll_link");
    touch_links.emplace_back("wrist_flex_link");
    arm_group->attachObject("arbitrary_gripper_object", "gripper_link", touch_links);
  }

  if (!moveToLocalizePose(0))
  {
    ROS_INFO("Failed to move to localize pose, aborting in hand localization.");
    in_hand_localization_server.setAborted(result);
    if (attach_arbitrary_object)
    {
      arm_group->detachObject("arbitrary_gripper_object");
      vector<string> obj_ids;
      obj_ids.push_back("arbitrary_gripper_object");
      planning_scene_interface->removeCollisionObjects(obj_ids);
    }
    return;
  }

  ROS_INFO("Localize pose reached.");
  ROS_INFO("Pointing head at gripper...");
  // lookup transform and adjust point back a little bit in the base_link frame because the robot should look down more
  geometry_msgs::TransformStamped head_point = tf_buffer.lookupTransform("base_link", "gripper_link",
                                                                       ros::Time(0), ros::Duration(1.0));
  control_msgs::PointHeadGoal head_goal;
  head_goal.target.header.frame_id = "base_link";
  head_goal.target.point.x = head_point.transform.translation.x - 0.05;
  head_goal.target.point.y = head_point.transform.translation.y;
  head_goal.target.point.z = head_point.transform.translation.z;
  point_head_client.sendGoal(head_goal);
  point_head_client.waitForResult(ros::Duration(5.0));
  ROS_INFO("Head angle set.");

  // extract an object point cloud in the wrist frame
  // TODO: run this for multiple views, merge point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (!extractObjectCloud(object_cloud))
  {
    in_hand_localization_server.setAborted(result);
    if (attach_arbitrary_object)
    {
      arm_group->detachObject("arbitrary_gripper_object");
      vector<string> obj_ids;
      obj_ids.push_back("arbitrary_gripper_object");
      planning_scene_interface->removeCollisionObjects(obj_ids);
    }
    return;
  }
  ROS_INFO("Initial object point cloud extracted.");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_view_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  double view_difference = 2*M_PI/num_views;
  for (unsigned int i = 1; i < num_views; i ++)
  {
    ROS_INFO("Capturing next view...");
    if (!moveToLocalizePose(i*view_difference))
    {
      ROS_INFO("Failed to move to localize pose, using as many views as we have collected so far...");
      break;
    }

    if (!extractObjectCloud(new_view_cloud))
    {
      ROS_INFO("Point cloud not responding, skipping this view...");
      continue;
    }
    *object_cloud += *new_view_cloud;
    ROS_INFO("View %d captured.", i);
  }

  // filter cluster to reduce noise
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outlier_remover;
  outlier_remover.setInputCloud(object_cloud);
  outlier_remover.setRadiusSearch(outlier_radius);
  outlier_remover.setMinNeighborsInRadius(min_neighbors);
  outlier_remover.filter(*object_cloud_filtered);
  outlier_remover.setInputCloud(object_cloud_filtered);
  outlier_remover.setMinNeighborsInRadius(min_neighbors/2.0);
  outlier_remover.filter(*object_cloud);

  if (debug)
  {
    object_cloud_debug.publish(object_cloud);
  }

  // calculate principle axes on cluster
  // compute principal direction
  Eigen::Matrix3f covariance;
  Eigen::Vector4f centroid;
  // use center instead of the centroid to not bias closer towards the sensor
//  pcl::PointXYZRGB min_original, max_original;
//  pcl::getMinMax3D(*object_cloud, min_original, max_original);
  pcl::PointXYZRGB centroid_point;
  pcl::computeCentroid(*object_cloud, centroid_point);
//  centroid[0] = fabs(max_original.x - min_original.x);
//  centroid[1] = fabs(max_original.y - min_original.y);
//  centroid[2] = fabs(max_original.z - min_original.z);
  centroid[0] = centroid_point.x;
  centroid[1] = centroid_point.y;
  centroid[2] = centroid_point.z;
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

  wrist_object_tf.header.frame_id = object_cloud->header.frame_id;
  wrist_object_tf.child_frame_id = "object_frame";
  wrist_object_tf.header.stamp = ros::Time::now();
  tf::vector3TFToMsg(tfinal_tf, wrist_object_tf.transform.translation);
  tf::quaternionTFToMsg(qfinal_tf, wrist_object_tf.transform.rotation);

  transform_set = true;

  if (debug)
  {
    object_pose.header.frame_id = object_cloud->header.frame_id;
    object_pose.pose.position.x = wrist_object_tf.transform.translation.x;
    object_pose.pose.position.y = wrist_object_tf.transform.translation.y;
    object_pose.pose.position.z = wrist_object_tf.transform.translation.z;
    object_pose.pose.orientation = wrist_object_tf.transform.rotation;
    object_pose_debug.publish(object_pose);
  }

  if (attach_arbitrary_object)
  {
    arm_group->detachObject("arbitrary_gripper_object");
    vector<string> obj_ids;
    obj_ids.push_back("arbitrary_gripper_object");
    planning_scene_interface->removeCollisionObjects(obj_ids);
  }

  in_hand_localization_server.setSucceeded(result);
}

bool InHandLocalizer::extractObjectCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &object_cloud)
{
  cloud_received = false;
  // TODO: (optimization) lower this sleep duration
  ros::Duration(1.0).sleep();  // wait for point cloud to catch up
  ros::Time start_time = ros::Time::now();
  ros::Rate cloud_wait_rate(100);
  while (ros::Time::now() - start_time < ros::Duration(5.0))
  {
    if (cloud_received)
    {
      break;
    }
    ros::spinOnce();
    cloud_wait_rate.sleep();
  }

  if (!cloud_received)
  {
    ROS_INFO("Point cloud is not updating, aborting in hand localization.");
    return false;
  }

  {
    boost::mutex::scoped_lock lock(cloud_mutex);

    // transform point cloud to wrist link
    geometry_msgs::TransformStamped to_wrist = tf_buffer.lookupTransform("wrist_roll_link", cloud->header.frame_id,
                                                                           ros::Time(0), ros::Duration(1.0));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
//    tf2::doTransform(cloud, cloud_transformed, to_wrist);
    tf::StampedTransform to_wrist_tf;
    tf::transformStampedMsgToTF(to_wrist, to_wrist_tf);
    pcl_ros::transformPointCloud(*cloud, *cloud_transformed, to_wrist_tf);
    cloud_transformed->header.frame_id = "wrist_roll_link";
    std::cout << cloud_transformed->header.frame_id << std::endl;

    // transforms we need: cloud frame to gripper_link, cloud frame to l_gripper_finger_link, cloud frame to
    // r_gripper_finger_link
    geometry_msgs::TransformStamped to_gripper = tf_buffer.lookupTransform(cloud_transformed->header.frame_id, "gripper_link",
                                                                           ros::Time(0), ros::Duration(1.0));
    geometry_msgs::TransformStamped to_l_finger = tf_buffer.lookupTransform(cloud_transformed->header.frame_id,
                                                                            "l_gripper_finger_link", ros::Time(0), ros::Duration(1.0));
    geometry_msgs::TransformStamped to_r_finger = tf_buffer.lookupTransform(cloud_transformed->header.frame_id,
                                                                            "r_gripper_finger_link", ros::Time(0), ros::Duration(1.0));

    // transform datastructures the various APIs will need
    Eigen::Vector3d translation;
    tf::Quaternion rotation_tf;
    double r, p, y;
    Eigen::Vector3f rotation;

    // create the crop box that we'll use to reduce the point cloud and remove the finger and palm points
    pcl::CropBox<pcl::PointXYZRGB> crop_box;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr remove_points(new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Vector4f min_point, max_point;
    crop_box.setInputCloud(cloud_transformed);

    // crop to smaller workspace
    double crop_dim = finger_dims.x/2.0 + palm_dims.x;
    min_point[0] = -crop_dim + 0.03;  // add some padding based on tests on the real robot
    min_point[1] = min_point[0];
    min_point[2] = min_point[0];
    max_point[0] = crop_dim;
    max_point[1] = max_point[0];
    max_point[2] = max_point[0];
    crop_box.setMin(min_point);
    crop_box.setMax(max_point);

    translation[0] = to_gripper.transform.translation.x;
    translation[1] = to_gripper.transform.translation.y;
    translation[2] = to_gripper.transform.translation.z;
    crop_box.setTranslation(translation.cast<float>());

    tf::quaternionMsgToTF(to_gripper.transform.rotation, rotation_tf);
    tf::Matrix3x3(rotation_tf).getRPY(r, p, y);
    rotation[0] = static_cast<float>(r);
    rotation[1] = static_cast<float>(p);
    rotation[2] = static_cast<float>(y);
    crop_box.setRotation(rotation);
    crop_box.filter(*object_cloud);

    if (debug)
    {
      crop_debug.publish(object_cloud);
    }

    crop_box.setInputCloud(object_cloud);
    crop_box.setNegative(true);

    // crop palm points
    max_point[0] = -finger_dims.x/2.0 + padding;
    min_point[0] = -finger_dims.x/2.0 - palm_dims.x - padding;
    max_point[1] = palm_dims.y/2.0 + padding;
    min_point[1] = -max_point[1];
    max_point[2] = palm_dims.z/2.0 + padding;
    min_point[2] = -max_point[2];
    crop_box.setMin(min_point);
    crop_box.setMax(max_point);

    translation[0] = to_gripper.transform.translation.x;
    translation[1] = to_gripper.transform.translation.y;
    translation[2] = to_gripper.transform.translation.z;
    crop_box.setTranslation(translation.cast<float>());

    tf::quaternionMsgToTF(to_gripper.transform.rotation, rotation_tf);
    tf::Matrix3x3(rotation_tf).getRPY(r, p, y);
    rotation[0] = static_cast<float>(r);
    rotation[1] = static_cast<float>(p);
    rotation[2] = static_cast<float>(y);
    crop_box.setRotation(rotation);

    if (debug)
    {
      crop_box.setNegative(false);
      crop_box.filter(*remove_points);
      palm_debug.publish(remove_points);
      crop_box.setNegative(true);
    }

    crop_box.filter(*object_cloud);

    // crop left finger points
    max_point[0] = finger_dims.x + padding;
    min_point[0] = -max_point[0];
    max_point[1] = finger_dims.y + padding;
    min_point[1] = -padding;
    max_point[2] = finger_dims.z/2.0 + padding;
    min_point[2] = -max_point[2];
    crop_box.setMin(min_point);
    crop_box.setMax(max_point);

    translation[0] = to_l_finger.transform.translation.x;
    translation[1] = to_l_finger.transform.translation.y;
    translation[2] = to_l_finger.transform.translation.z;
    crop_box.setTranslation(translation.cast<float>());

    tf::quaternionMsgToTF(to_l_finger.transform.rotation, rotation_tf);
    tf::Matrix3x3(rotation_tf).getRPY(r, p, y);
    rotation[0] = static_cast<float>(r);
    rotation[1] = static_cast<float>(p);
    rotation[2] = static_cast<float>(y);
    crop_box.setRotation(rotation);

    if (debug)
    {
      crop_box.setNegative(false);
      crop_box.filter(*remove_points);
      l_debug.publish(remove_points);
      crop_box.setNegative(true);
    }

    crop_box.filter(*object_cloud);

    // crop right finger points
    max_point[0] = finger_dims.x + padding;
    min_point[0] = -max_point[0];
    max_point[1] = padding;
    min_point[1] = -finger_dims.y - padding;
    max_point[2] = finger_dims.z/2.0 + padding;
    min_point[2] = -max_point[2];
    crop_box.setMin(min_point);
    crop_box.setMax(max_point);

    translation[0] = to_r_finger.transform.translation.x;
    translation[1] = to_r_finger.transform.translation.y;
    translation[2] = to_r_finger.transform.translation.z;
    crop_box.setTranslation(translation.cast<float>());

    tf::quaternionMsgToTF(to_r_finger.transform.rotation, rotation_tf);
    tf::Matrix3x3(rotation_tf).getRPY(r, p, y);
    rotation[0] = static_cast<float>(r);
    rotation[1] = static_cast<float>(p);
    rotation[2] = static_cast<float>(y);
    crop_box.setRotation(rotation);

    if (debug)
    {
      crop_box.setNegative(false);
      crop_box.filter(*remove_points);
      r_debug.publish(remove_points);
      crop_box.setNegative(true);
    }

    crop_box.filter(*object_cloud);
  }

  return true;
}

bool InHandLocalizer::moveToLocalizePose(double wrist_offset)
{
  ROS_INFO("Moving to localize pose...");
  arm_group->setPlannerId("arm[RRTConnectkConfigDefault]");
  arm_group->setPlanningTime(5.0);
  arm_group->setStartStateToCurrentState();
  localize_pose.position[localize_pose.position.size() - 1] += wrist_offset;
  arm_group->setJointValueTarget(localize_pose);

  moveit::planning_interface::MoveItErrorCode move_result = arm_group->move();
  if (move_result != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    localize_pose.position[localize_pose.position.size() - 1] -= wrist_offset;
    return false;
  }
  localize_pose.position[localize_pose.position.size() - 1] -= wrist_offset;

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "in_hand_localizer");

  InHandLocalizer ihl;

  ros::Rate loop_rate(1000);

  while (ros::ok())
  {
    ihl.publishTF();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}
