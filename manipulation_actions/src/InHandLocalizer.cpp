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
  pnh.param<double>("finger_dims_x", finger_dims.x, 0.058);
  pnh.param<double>("finger_dims_y", finger_dims.y, 0.014);
  pnh.param<double>("finger_dims_z", finger_dims.z, 0.026);
  pnh.param<double>("palm_dims_x", palm_dims.x, 0.137);
  pnh.param<double>("palm_dims_y", palm_dims.y, 0.118);
  pnh.param<double>("palm_dims_z", palm_dims.z, 0.08);
  pnh.param<double>("padding", padding, 0.005);
  pnh.param<bool>("debug", debug, true);

  //TODO: make this a param
  localize_pose.name.push_back("shoulder_pan_joint");
  localize_pose.name.push_back("shoulder_lift_joint");
  localize_pose.name.push_back("upperarm_roll_joint");
  localize_pose.name.push_back("elbow_flex_joint");
  localize_pose.name.push_back("forearm_roll_joint");
  localize_pose.name.push_back("wrist_flex_joint");
  localize_pose.name.push_back("wrist_roll_joint");
  localize_pose.position.push_back(1.47);
  localize_pose.position.push_back(0.88);
  localize_pose.position.push_back(1.80);
  localize_pose.position.push_back(-1.67);
  localize_pose.position.push_back(0.91);
  localize_pose.position.push_back(-1.31);
  localize_pose.position.push_back(-1.53);  // for a second verify pose, set this to 0.11

  cloud_received = false;

  arm_group = new moveit::planning_interface::MoveGroupInterface("arm");
  arm_group->startStateMonitor();

  cloud_subscriber = n.subscribe(cloud_topic, 1, &InHandLocalizer::cloudCallback, this);

  if (debug)
  {
    palm_debug = pnh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("palm_debug", 1);
    l_debug = pnh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("l_debug", 1);
    r_debug = pnh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("r_debug", 1);
    crop_debug = pnh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("crop_debug", 1);
    object_cloud_debug = pnh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("object_cloud_debug", 1);
  }

  in_hand_localization_server.start();
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

  ROS_INFO("Moving to localize pose...");
  arm_group->setPlannerId("arm[RRTConnectkConfigDefault]");
  arm_group->setPlanningTime(5.0);
  arm_group->setStartStateToCurrentState();
  arm_group->setJointValueTarget(localize_pose);

  moveit::planning_interface::MoveItErrorCode move_result = arm_group->move();
  if (move_result != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("Failed to move to localize pose, aborting in hand localization.");
    in_hand_localization_server.setAborted(result);
    return;
  }

  ROS_INFO("Localize pose reached.");
  ROS_INFO("Pointing head at gripper...");
  control_msgs::PointHeadGoal head_goal;
  head_goal.target.header.frame_id = "gripper_link";
  point_head_client.sendGoal(head_goal);
  point_head_client.waitForResult(ros::Duration(5.0));

  // extract an object point cloud in the wrist frame
  // TODO: run this for multiple views, merge point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (!extractObjectCloud(object_cloud))
  {
    in_hand_localization_server.setAborted(result);
    return;
  }

  // TODO: calculate principle axes on cluster
  

  in_hand_localization_server.setSucceeded(result);
}

bool InHandLocalizer::extractObjectCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &object_cloud)
{
  cloud_received = false;
  ros::Duration(0.5).sleep();  // wait for point cloud to catch up
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

    // TODO: transform point cloud to wrist link
    geometry_msgs::TransformStamped to_wrist = tf_buffer.lookupTransform(cloud->header.frame_id, "wrist_link",
                                                                           ros::Time(0), ros::Duration(1.0));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_transformed->header.frame_id = "wrist_roll_link";
//    tf2::doTransform(cloud, cloud_transformed, to_wrist);
    tf::StampedTransform to_wrist_tf;
    tf::transformStampedMsgToTF(to_wrist, to_wrist_tf);
    pcl_ros::transformPointCloud(*cloud, *cloud_transformed, to_wrist_tf);

    // transforms we need: cloud frame to gripper_link, cloud frame to l_gripper_finger_link, cloud frame to
    // r_gripper_finger_link
    geometry_msgs::TransformStamped to_gripper = tf_buffer.lookupTransform(cloud->header.frame_id, "gripper_link",
                                                                           ros::Time(0), ros::Duration(1.0));
    geometry_msgs::TransformStamped to_l_finger = tf_buffer.lookupTransform(cloud->header.frame_id,
                                                                            "l_gripper_finger_link", ros::Time(0), ros::Duration(1.0));
    geometry_msgs::TransformStamped to_r_finger = tf_buffer.lookupTransform(cloud->header.frame_id,
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
    min_point[0] = -crop_dim;
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
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "in_hand_localizer");

  InHandLocalizer ihl;

  ros::spin();

  return EXIT_SUCCESS;
}