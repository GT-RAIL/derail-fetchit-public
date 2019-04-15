#ifndef FETCH_GRASP_SUGGESTION_SUGGESTER_H
#define FETCH_GRASP_SUGGESTION_SUGGESTER_H

// C++
#include <fstream>
#include <iostream>

// Boost
#include <boost/thread/mutex.hpp>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <fetch_grasp_suggestion/AddObject.h>
#include <fetch_grasp_suggestion/common.h>
#include <fetch_grasp_suggestion/ClassifyAll.h>
#include <fetch_grasp_suggestion/SuggestGraspsAction.h>
#include <rail_manipulation_msgs/PairwiseRank.h>


#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <rail_grasp_calculation_msgs/RankGraspsAction.h>
#include <rail_grasp_calculation_msgs/SampleGraspsAction.h>
#include <rail_manipulation_msgs/GraspFeedback.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <rail_manipulation_msgs/SuggestGrasps.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/**
 * @brief ROS node for grasp suggestion.
 *
 * Grasp suggestion with two workflows:
 * (1) ROS service to calculate a list of grasps given a point cloud
 * (2) Actionlib server to calculate grasps on objects from a continuously-listened-to segmentation topic
 *
 * Also included is grasp feedback logging which generates positive and negative ordered pair grasp feature vectors
 * output to a .csv file.
 */
class Suggester
{

public:
  /**
   * @brief Initialize ROS message, services, and actions required for grasp suggestion.
   */
  Suggester();

private:

  /**
   * @brief Calculate grasp suggestions for an object from the segmented objects list.
   * @param goal index of object-of-interest in segmented objects list
   */
  void getGraspSuggestions(const fetch_grasp_suggestion::SuggestGraspsGoalConstPtr &goal);

  /**
   * @brief Generate and log pairwise grasp ranking training instances.
   * @param grasp_feedback indices of considered and selected grasps
   */
  void graspFeedbackCallback(const rail_manipulation_msgs::GraspFeedback &grasp_feedback);

  /**
   * @brief Store the full scene point cloud.
   * @param msg incoming point cloud data
   */
  void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg);

  /**
   * @brief Store the list of segmented objects.
   * @param list incoming object data
   */
  void objectsCallback(const rail_manipulation_msgs::SegmentedObjectList &list);

  /**
   * @brief Calculate grasp suggestions given a segmented point cloud.
   * @param req input point cloud
   * @param res output ordered list of grasps
   * @return true on service call success
   */
  bool suggestGraspsCallback(rail_manipulation_msgs::SuggestGrasps::Request &req,
      rail_manipulation_msgs::SuggestGrasps::Response &res);

  /**
   * @brief Calculate grasp suggestions given an unsegmented point cloud.
   * @param req input point cloud
   * @param res output ordered list of grasps
   * @return true on service call success
   */
  bool suggestGraspsSceneCallback(rail_manipulation_msgs::SuggestGrasps::Request &req,
      rail_manipulation_msgs::SuggestGrasps::Response &res);

  /**
   * @brief Calculate grasp suggestions given a point cloud using Agile only.
   *
   * This is included only for baseline testing, and is not intended to be used for any normal use of this
   * package!
   *
   * @param req input point cloud
   * @param res output list of grasps
   * @return true on service call success
   */
  bool suggestGraspsAgileCallback(rail_manipulation_msgs::SuggestGrasps::Request &req,
      rail_manipulation_msgs::SuggestGrasps::Response &res);

    /**
     * @brief Calculate grasp suggestions given a point cloud using antipodal sampling only.
     *
     * This is included only for baseline testing, and is not intended to be used for any normal use of this
     * package!
     *
     * @param req input point cloud
     * @param res output list of grasps
     * @return true on service call success
     */
  bool suggestGraspsRandomCallback(rail_manipulation_msgs::SuggestGrasps::Request &req,
      rail_manipulation_msgs::SuggestGrasps::Response &res);

  /**
   * @brief Rank stored grasp list according to pairwise classifier.
   * @param req empty request
   * @param res pairwise-ranked grasp list
   * @return true on service call success
   */
  bool pairwiseRankCallback(rail_manipulation_msgs::PairwiseRank::Request &req,
      rail_manipulation_msgs::PairwiseRank::Response &res);

  /**
   * @brief Rank stored grasp list according to pairwise classifier, for use with unsegmented point clouds.
   * @param req empty request
   * @param res pairwise-ranked grasp list
   * @return true on service call success
   */
  bool pairwiseRankSceneCallback(rail_manipulation_msgs::PairwiseRank::Request &req,
      rail_manipulation_msgs::PairwiseRank::Response &res);

  /**
   * @brief Sample grasp candidates using rail_grasp_calculation.
   *
   * This is the first of two required steps for grasp ranking. A large set of potential grasps are sampled from the
   * environment point cloud in the area containing the object, using AGILE. This results in a set of unclustered,
   * unranked grasp poses where a two-fingered gripper can potentially close on the object.
   *
   * @param object point cloud of only the object-of-interest
   * @param object_source_frame tf frame of the object point cloud
   * @param environment_source_frame tf frame of the full scene point cloud
   * @param grasps_out unordered list of sampled grasp candidates
   * @param cloud_out cropped environment point cloud required for the next grasp ranking step
   */
  void SampleGraspCandidates(sensor_msgs::PointCloud2 object, std::string object_source_frame,
      std::string environment_source_frame, geometry_msgs::PoseArray &grasps_out,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out);

  /**
   * @brief Sample grasp candidates using rail_grasp_calculation.
   *
   * This is the first of two required steps for grasp ranking. A large set of potential grasps are sampled from the
   * environment point cloud in the area containing the object, using AGILE. This results in a set of unclustered,
   * unranked grasp poses where a two-fingered gripper can potentially close on the object.
   *
   * @param object point cloud of only the object-of-interest
   * @param object_source_frame tf frame of the object point cloud
   * @param environment_source_frame tf frame of the full scene point cloud
   * @param grasps_out unordered list of sampled grasp candidates
   * @param cloud_out cropped environment point cloud required for the next grasp ranking step
   * @param agile_only flag for using the full agile pipeline (set to true for agile baseline testing)
   */
  void SampleGraspCandidates(sensor_msgs::PointCloud2 object, std::string object_source_frame,
      std::string environment_source_frame, geometry_msgs::PoseArray &grasps_out,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, bool agile_only);

  /**
   * @brief Sample grasp candidates from an unsegmented point cloud using rail_grasp_calculation.
   * @param cloud point cloud of the area over which to calculate grasps
   * @param grasps_out unordered list of sampled grasp candidates
   */
  void SampleGraspCandidatesScene(sensor_msgs::PointCloud2 cloud, geometry_msgs::PoseArray &grasps_out);

  /**
   * @brief Cluster and rank grasps according to a set of heuristics.
   *
   * This is the second of two required steps for grasp ranking. The ranked list is computed by first clustering the
   * input grasps by similar position and orientation, followed by calculating a linear combination of a set of grasp
   * heuristics that describe the relationship of the grasp approach angle with the object surface and environment and
   * the position of the grasp with respect to the object.
   *
   * @param cloud_in environment point cloud cropped from the full scene around the object-of-interest
   * @param objectCloud point cloud of only the object-of-interest
   * @param grasps unordered grasp list
   * @param object_source_frame tf frame of the object point cloud
   * @param ranked_grasps resulting list of ranked grasps, with their associated heuristic information
   */
  void rankCandidates(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, sensor_msgs::PointCloud2 object_cloud,
      geometry_msgs::PoseArray &grasps, std::string object_source_frame,
      fetch_grasp_suggestion::RankedGraspList &ranked_grasps);

  /**
   * @brief Cluster and rank grasps according to a set of heuristics for an unsegmented point cloud.
   * @param cloud unsegmented point cloud covering the area over which grasps were calculated
   * @param grasps unordered grasp list
   * @param ranked_grasps resulting list of ranked grasps, with their associated heuristic information
   */
  void rankCandidatesScene(sensor_msgs::PointCloud2 cloud, geometry_msgs::PoseArray &grasps,
      fetch_grasp_suggestion::RankedGraspList &ranked_grasps);

  /**
   * @brief Move a pose along the local x direction of the pose, effectively adjusting the grasp depth
   * @param grasp_pose grasp pose to be adjusted
   * @param distance amount by which to adjust the pose, in meters
   * @return new grasp pose translated by the specified amount in the x direction of the pose
   */
  static geometry_msgs::Pose adjustGraspDepth(geometry_msgs::Pose grasp_pose, double distance);

  /**
   * @brief Check if the fingers of a potential grasp are in collision with a given point cloud
   * @param grasp pose of grasp candidate
   * @param cloud point cloud of interest for collision checking
   * @param check_palm also check palm collision if true
   * @return true if grasp is in collision with point cloud
   */
  bool isInCollision(geometry_msgs::PoseStamped grasp, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, bool check_palm);

  /**
   * @brief Check if the fingers of a potential grasp are in collision with a given point cloud
   * @param grasp pose of grasp candidate
   * @param cloud point cloud of interest for collision checking
   * @param check_palm also check palm collision if true
   * @return true if grasp is in collision with point cloud
   */
  bool isInCollision(geometry_msgs::PoseStamped grasp, sensor_msgs::PointCloud2 cloud, bool check_palm);

  ros::NodeHandle n_, pnh_;

  // topics
  ros::Publisher grasps_publisher_;
  ros::Subscriber cloud_subscriber_;
  ros::Subscriber objects_subscriber_;
  ros::Subscriber grasp_feedback_subscriber_;

  // services
  ros::ServiceClient clear_objects_client_;
  ros::ServiceClient add_object_client_;
  ros::ServiceClient classify_all_client_;
  ros::ServiceServer suggest_grasps_service_;
  ros::ServiceServer suggest_grasps_baseline_service_;
  ros::ServiceServer suggest_grasps_random_service_;
  ros::ServiceServer suggest_grasps_scene_service_;
  ros::ServiceServer pairwise_rank_service_;
  ros::ServiceServer pairwise_rank_scene_service_;

  // actionlib
  actionlib::SimpleActionClient<rail_grasp_calculation_msgs::SampleGraspsAction> sample_grasps_client_;
  actionlib::SimpleActionClient<rail_grasp_calculation_msgs::SampleGraspsAction> sample_grasps_baseline_client_;
  actionlib::SimpleActionClient<rail_grasp_calculation_msgs::RankGraspsAction> rank_grasps_object_client_;
  actionlib::SimpleActionClient<rail_grasp_calculation_msgs::RankGraspsAction> rank_grasps_scene_client_;
  actionlib::SimpleActionServer<fetch_grasp_suggestion::SuggestGraspsAction> suggest_grasps_server_;

  tf::TransformListener tf_listener_;

  boost::mutex cloud_mutex_;  /// mutex for full scene point cloud
  boost::mutex stored_grasp_mutex_;  /// mutex for suggested grasp list (service workflow)
  boost::mutex object_list_mutex_;  /// mutex for segmented object list (actionlib workflow)

  rail_manipulation_msgs::SegmentedObjectList object_list_;  /// stored segmented objects list (for actionlib workflow)

  sensor_msgs::PointCloud2 stored_object_cloud_;  /// most recent object-of-interest point cloud (for service workflow)
  sensor_msgs::PointCloud2 stored_scene_cloud_;  /// most recent scene point cloud (for unsegmented point cloud mode)
  fetch_grasp_suggestion::RankedGraspList stored_grasp_list_;  /// most recent grasp list (for service workflow)
  std::vector<int> selected_grasps_;  /// previously selected grasps (as indices), to prevent conflicting training data
  std::vector<double> object_features_;  /// object features calculated for training instances, stored to save time
  double min_grasp_depth_, max_grasp_depth_;  /// bounds on grasp depth search

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;  /// stored full scene point cloud

  bool cloud_received_;  /// true once first point cloud is received

  std::string filename_;  /// output file for saving training instances
  std::ofstream file_;
};

#endif  // FETCH_GRASP_SUGGESTION_SUGGESTER_H
