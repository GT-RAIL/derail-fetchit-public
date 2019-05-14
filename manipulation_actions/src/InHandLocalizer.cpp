#include <manipulation_actions/InHandLocalizer.h>

using std::ios;
using std::string;
using std::stringstream;
using std::vector;

InHandLocalizer::InHandLocalizer() :
    pnh("~"),
    tf_listener(tf_buffer),
    in_hand_localization_server(pnh, "localize", boost::bind(&InHandLocalizer::executeLocalize, this, _1), false),
    point_head_client("head_controller/point_head"),
    arm_control_client("arm_controller/follow_joint_trajectory")
{
  pnh.param<string>("cloud_topic", cloud_topic, "head_camera/depth_registered/points");
  pnh.param<int>("num_views", num_views, 3);
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

  wrist_goal.trajectory.joint_names.push_back("shoulder_pan_joint");
  wrist_goal.trajectory.joint_names.push_back("shoulder_lift_joint");
  wrist_goal.trajectory.joint_names.push_back("upperarm_roll_joint");
  wrist_goal.trajectory.joint_names.push_back("elbow_flex_joint");
  wrist_goal.trajectory.joint_names.push_back("forearm_roll_joint");
  wrist_goal.trajectory.joint_names.push_back("wrist_flex_joint");
  wrist_goal.trajectory.joint_names.push_back("wrist_roll_joint");
  wrist_goal.trajectory.points.resize(1);

  arm_group = new moveit::planning_interface::MoveGroupInterface("arm");
  arm_group->startStateMonitor();

  planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();

  attach_gripper_client =
      n.serviceClient<manipulation_actions::AttachToBase>("collision_scene_manager/attach_to_gripper");
  detach_objects_client = n.serviceClient<std_srvs::Empty>("collision_scene_manager/detach_objects");

  reset_object_frame_server = pnh.advertiseService("reset_object_frame", &InHandLocalizer::resetObjectFrame, this);;

  transform_set = false;

  // default initial transform at gripper_link, to keep tf happy
  resetTransform();

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

bool InHandLocalizer::resetObjectFrame(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  resetTransform();
  return true;
}

void InHandLocalizer::resetTransform()
{
  boost::mutex::scoped_lock(transform_mutex);
  wrist_object_tf.header.frame_id = "gripper_link";
  wrist_object_tf.child_frame_id = "object_frame";
  wrist_object_tf.transform.translation.x = 0.0;
  wrist_object_tf.transform.translation.y = 0.0;
  wrist_object_tf.transform.translation.z = 0.0;
  wrist_object_tf.transform.rotation.x = 0.0;
  wrist_object_tf.transform.rotation.y = 0.0;
  wrist_object_tf.transform.rotation.z = 0.0;
  wrist_object_tf.transform.rotation.w = 1.0;
  wrist_object_tf.header.stamp = ros::Time::now();
}

void InHandLocalizer::publishTF()
{
  boost::mutex::scoped_lock(transform_mutex);
  wrist_object_tf.header.stamp = ros::Time::now();
  tf_broadcaster.sendTransform(wrist_object_tf);
  if (debug && transform_set)
  {
    object_pose_debug.publish(object_pose);
  }
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
  head_goal.target.point.x = head_point.transform.translation.x - 0.2;
  head_goal.target.point.y = head_point.transform.translation.y;
  head_goal.target.point.z = head_point.transform.translation.z;
  point_head_client.sendGoal(head_goal);
  point_head_client.waitForResult(ros::Duration(5.0));
  ROS_INFO("Head angle set.");

  // wait for point cloud to catch up
  ros::Duration(1.0).sleep();

  // extract an object point cloud in the wrist frame
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
  double views = num_views;
  if (goal->correct_object_direction)
  {
    views = 4;
  }
  double view_difference = 2*M_PI/views;
  for (unsigned int i = 1; i < views; i ++)
  {
    ROS_INFO("Capturing next view...");
    if (!moveToLocalizePose(i*view_difference, true))
    {
      ROS_INFO("Failed to move to localize pose, using as many views as we have collected so far...");
      break;
    }

    // wait for point cloud to catch up
    ros::Duration(1.0).sleep();

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
  pcl::PointXYZRGB centroid_point;
  pcl::computeCentroid(*object_cloud, centroid_point);
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
  tf::Quaternion adjustment;
  adjustment.setRPY(0, -M_PI_2, 0);
  qfinal_tf *= adjustment;

  {
    boost::mutex::scoped_lock(transform_mutex);
    wrist_object_tf.header.frame_id = object_cloud->header.frame_id;
    wrist_object_tf.child_frame_id = "object_frame";
    wrist_object_tf.header.stamp = ros::Time::now();
    tf::vector3TFToMsg(tfinal_tf, wrist_object_tf.transform.translation);
    tf::quaternionTFToMsg(qfinal_tf, wrist_object_tf.transform.rotation);

    transform_set = true;
  }

  // calculate what we need for bounding box info and optoinal pose direction correction
  tf::Transform tf_transform;
  tf_transform.setRotation(qfinal_tf);
  tf_transform.setOrigin(tfinal_tf);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl_ros::transformPointCloud(*object_cloud, *cloud_transformed, tf_transform.inverse());
  cloud_transformed->header.frame_id = "object_frame";

  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud(cloud_transformed);

  pcl::PointXYZRGB min_dim, max_dim;
  pcl::getMinMax3D(*cloud_transformed, min_dim, max_dim);

  // TODO: sanity checks given our known object poses (point cloud noise can mess this up)
  double xdim = max_dim.x - min_dim.x;
  double ydim = max_dim.y - min_dim.y;
  double zdim = max_dim.z - min_dim.z;

  // check 1: very large objects in one dimension
  if (std::max(std::max(xdim, ydim), zdim) > 0.19)
  {
    ROS_INFO("In-hand localization extracted an object that's unusually long; aborting for retry.");
    in_hand_localization_server.setAborted(result);
    return;
  }

  // check 2: the "large square" or "large box" case
  vector<double> dims;
  dims.push_back(xdim);
  dims.push_back(ydim);
  dims.push_back(zdim);
  std::sort(dims.begin(), dims.end());
  if (dims[2] > .08 && dims[2] - dims[1] < .05)
  {
    ROS_INFO("In-hand localization extracted an object that's unusually large and square; aborting for retry.");
    in_hand_localization_server.setAborted(result);
    return;
  }

  if (goal->correct_object_direction)
  {
    // goal: set the x direction to point away from the larger part of the object

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
      {
        boost::mutex::scoped_lock(transform_mutex);
        ROS_INFO("Flipping transform.");
        // flip transform
        tf2::Quaternion tf_q(wrist_object_tf.transform.rotation.x, wrist_object_tf.transform.rotation.y,
                             wrist_object_tf.transform.rotation.z, wrist_object_tf.transform.rotation.w);
        tf2::Quaternion flip;
        flip.setRPY(0, 0, M_PI);
        tf_q = tf_q * flip;

        wrist_object_tf.transform.rotation.x = tf_q.x();
        wrist_object_tf.transform.rotation.y = tf_q.y();
        wrist_object_tf.transform.rotation.z = tf_q.z();
        wrist_object_tf.transform.rotation.w = tf_q.w();
      }
    }
  }

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
  else
  {
    // attach new localized object as collision object to the gripper

    std_srvs::Empty detach;
    detach_objects_client.call(detach);

    manipulation_actions::AttachToBase attach_srv;
    attach_srv.request.segmented_object.recognized = false;

    pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*object_cloud, *temp_cloud);

    pcl_conversions::fromPCL(*temp_cloud, attach_srv.request.segmented_object.point_cloud);

    attach_srv.request.segmented_object.bounding_volume.dimensions.x = max_dim.x - min_dim.x;
    attach_srv.request.segmented_object.bounding_volume.dimensions.y = max_dim.y - min_dim.y;
    attach_srv.request.segmented_object.bounding_volume.dimensions.z = max_dim.z - min_dim.z;

    geometry_msgs::PoseStamped bb_pose_cloud;
    bb_pose_cloud.header.frame_id = wrist_object_tf.header.frame_id;
    bb_pose_cloud.pose.position.x = wrist_object_tf.transform.translation.x;
    bb_pose_cloud.pose.position.y = wrist_object_tf.transform.translation.y;
    bb_pose_cloud.pose.position.z = wrist_object_tf.transform.translation.z;
    bb_pose_cloud.pose.orientation = wrist_object_tf.transform.rotation;

    // set pose for bounding box
    attach_srv.request.segmented_object.bounding_volume.pose = bb_pose_cloud;

    if (!attach_gripper_client.call(attach_srv))
    {
      ROS_INFO("Couldn't call collision scene manager client to update the gripper's attached object!");
    }
  }

  result.object_transform = wrist_object_tf;

  in_hand_localization_server.setSucceeded(result);
}

bool InHandLocalizer::extractObjectCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &object_cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc_msg = ros::topic::waitForMessage< pcl::PointCloud<pcl::PointXYZRGB> >
      (cloud_topic, n, ros::Duration(10.0));
  if (pc_msg == NULL)
  {
    ROS_INFO("No point cloud received for segmentation.");
    return false;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);
  *pc = *pc_msg;

  // transform point cloud to wrist link
  geometry_msgs::TransformStamped to_wrist = tf_buffer.lookupTransform("wrist_roll_link", pc->header.frame_id,
                                                                         ros::Time(0), ros::Duration(1.0));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
//    tf2::doTransform(cloud, cloud_transformed, to_wrist);
  tf::StampedTransform to_wrist_tf;
  tf::transformStampedMsgToTF(to_wrist, to_wrist_tf);
  pcl_ros::transformPointCloud(*pc, *cloud_transformed, to_wrist_tf);
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

  return true;
}

bool InHandLocalizer::moveToLocalizePose(double wrist_offset, bool no_moveit)
{
  ROS_INFO("Moving to localize pose...");
  sensor_msgs::JointStateConstPtr js_msg;
  sensor_msgs::JointState joint_state;
  while (joint_state.position.size() <= 3)
  {
    js_msg = ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states", n);
    if (js_msg != NULL)
    {
      joint_state = *js_msg;
    }
  }

  if (no_moveit)
  {
    // publish a non-trajectory to FollowJointTrajectory to disable gravity comp
    wrist_goal.trajectory.points[0].positions.clear();
    for (size_t i = 6; i < 6 + wrist_goal.trajectory.joint_names.size(); i ++)
    {
      wrist_goal.trajectory.points[0].positions.push_back(joint_state.position[i]);
    }
    wrist_goal.trajectory.points[0].positions[wrist_goal.trajectory.points[0].positions.size() - 1] =
        localize_pose.position[localize_pose.position.size() - 1] + wrist_offset;
    wrist_goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

    arm_control_client.sendGoal(wrist_goal);
    arm_control_client.waitForResult();
    return true;
  }
  else
  {
    bool at_goal = true;
    for (size_t i = 6; i < 6 + localize_pose.position.size(); i++)
    {
      double dst = fabs(joint_state.position[i] - localize_pose.position[i]);
      dst = std::min(dst, fabs(2 * M_PI - dst));
      if (dst > M_PI / 24)
      {
        at_goal = false;
        break;
      }
    }
    if (at_goal)
    {
      return true;
    }

    arm_group->setPlannerId("arm[RRTConnectkConfigDefault]");
    arm_group->setPlanningTime(1.5);
    arm_group->setStartStateToCurrentState();
    localize_pose.position[localize_pose.position.size() - 1] += wrist_offset;
    arm_group->setJointValueTarget(localize_pose);

    moveit::planning_interface::MoveItErrorCode move_result = arm_group->move();
    if (move_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      // one retry for execution/perception errors
      if (move_result.val == moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE
          || move_result.val == moveit_msgs::MoveItErrorCodes::CONTROL_FAILED
          || move_result.val == moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA
          || move_result.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT
          || move_result.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
      {
        while (joint_state.position.size() <= 3)
        {
          js_msg = ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states", n);
          if (js_msg != NULL)
          {
            joint_state = *js_msg;
          }
        }

        bool at_goal = true;
        for (size_t i = 6; i < 6 + localize_pose.position.size(); i++)
        {
          double dst = fabs(joint_state.position[i] - localize_pose.position[i]);
          dst = std::min(dst, fabs(2 * M_PI - dst));
          if (dst > M_PI / 24)
          {
            at_goal = false;
            break;
          }
        }
        if (at_goal)
        {
          return true;
        }

        arm_group->setStartStateToCurrentState();
        arm_group->setJointValueTarget(localize_pose);
        move_result = arm_group->move();
        if (move_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
          localize_pose.position[localize_pose.position.size() - 1] -= wrist_offset;
          return false;
        }
        else
        {
          localize_pose.position[localize_pose.position.size() - 1] -= wrist_offset;
          return true;
        }
      }
      else
      {
        localize_pose.position[localize_pose.position.size() - 1] -= wrist_offset;
        return false;
      }
    }
    localize_pose.position[localize_pose.position.size() - 1] -= wrist_offset;
  }

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "in_hand_localizer");

  InHandLocalizer ihl;

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ihl.publishTF();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}
