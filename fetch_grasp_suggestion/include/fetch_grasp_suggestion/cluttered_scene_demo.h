#ifndef FETCH_GRASP_SUGGESTION_CLUTTERED_SCENE_DEMO_H
#define FETCH_GRASP_SUGGESTION_CLUTTERED_SCENE_DEMO_H

// Boost
#include <boost/thread/mutex.hpp>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <fetch_grasp_suggestion/ExecuteGraspAction.h>
#include <fetch_grasp_suggestion/PairwiseRank.h>
#include <fetch_grasp_suggestion/PresetMoveAction.h>
#include <fetch_grasp_suggestion/RankedGraspList.h>
#include <fetch_grasp_suggestion/SuggestGrasps.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// Grasp Suggestion
#include <fetch_grasp_suggestion/point_cloud_manipulation.h>

/**
 * @brief Calculate and execute a sequence of pairwise classified grasps to clear objects in a cluttered scene
 */
class ClutteredSceneDemo
{

public:

  /**
   * @brief Initialize ROS messages, services, and actions, required for clearing a cluttered scene
   */
  ClutteredSceneDemo();

private:

  /**
   * @brief Callback to begin demo execution.
   *
   * Once called, the demo will run to completion, or until the user manually stops execution by following the
   * prompts presented in the command line.
   *
   * @param msg Empty message
   */
  void runCallback(const std_msgs::Empty &msg);

  /**
   * @brief Receive the point cloud covering the area to be cleared in the demo.
   * @param cloud Current scene point cloud
   */
  void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);

  ros::NodeHandle n_, pnh_;

  // topics
  ros::Publisher debug_publisher_;
  ros::Publisher cloud_publisher_;
  ros::Subscriber run_subscriber_;
  ros::Subscriber cloud_subscriber_;

  // services
  ros::ServiceClient suggest_grasps_client_;
  ros::ServiceClient rank_grasps_client_;
  ros::ServiceClient drop_object_client_;

  // actionlib
  actionlib::SimpleActionClient<fetch_grasp_suggestion::ExecuteGraspAction> execute_grasp_client_;
  actionlib::SimpleActionClient<fetch_grasp_suggestion::PresetMoveAction> prepare_robot_client_;
  actionlib::SimpleActionClient<fetch_grasp_suggestion::PresetMoveAction> drop_position_client_;
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_client_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

  tf::TransformListener tf_listener_;

  boost::mutex cloud_mutex_;

  bool debug_;
};

#endif  // FETCH_GRASP_SUGGESTION_CLUTTERED_SCENE_DEMO_H
