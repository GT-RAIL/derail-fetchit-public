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
  pnh.param<double>("finger_dims_x", finger_dims.x, 0.17);
  pnh.param<double>("finger_dims_y", finger_dims.y, 0.17);
  pnh.param<double>("finger_dims_z", finger_dims.z, 0.17);
  pnh.param<double>("palm_dims_x", palm_dims.x, 0.17);
  pnh.param<double>("palm_dims_y", palm_dims.y, 0.17);
  pnh.param<double>("palm_dims_z", palm_dims.z, 0.17);

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
    in_hand_localization_server.setAborted(result);
    return;
  }

  // TODO: point cloud filtering
  {
    boost::mutex::scoped_lock lock(cloud_mutex);

  }

  // TODO: calculate principle axes on cluster

  in_hand_localization_server.setSucceeded(result);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "in_hand_localizer");

  InHandLocalizer ihl;

  ros::spin();

  return EXIT_SUCCESS;
}