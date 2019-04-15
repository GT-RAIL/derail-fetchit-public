#ifndef FETCH_GRASP_SUGGESTION_TEST_GRASP_SUGGESTION_H
#define FETCH_GRASP_SUGGESTION_TEST_GRASP_SUGGESTION_H

// Boost
#include <boost/thread/mutex.hpp>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <fetch_grasp_suggestion/ExecuteGraspAction.h>
#include <fetch_grasp_suggestion/PairwiseRank.h>
#include <fetch_grasp_suggestion/RankedGraspList.h>
#include <fetch_grasp_suggestion/SuggestGrasps.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

/**
 * @brief Calculate and execute pairwise classified grasps for a segmented object in one autonomous step
 */
class Tester
{

public:

  /**
   * @brief Initialize ROS messages, services, and actions, required for grasp suggestion testing
   */
  Tester();

private:

  /**
   * @brief Perform a single grasp test for a given object, with grasps ranked by the pairwise ranking approach
   * @param msg index of the object to attempt to grasp
   */
  void testCallback(const std_msgs::Int32 &msg);

  /**
   * @brief Perform a single grasp test for a given object, with grasps calculated directly from AGILE
   * @param msg index of the object to attempt to grasp
   */
  void testAgileCallback(const std_msgs::Int32 &msg);

  /**
   * @brief Perform a single grasp test for a given object, with grasps ranked by a linear combination of heuristics
   * @param msg
   */
  void testHeuristicCallback(const std_msgs::Int32 &msg);

  /**
   * @brief Perform a single grasp test for a given object, with randomly selected antipodal grasps
   * @param msg
   */
  void testRandomCallback(const std_msgs::Int32 &msg);

  /**
   * @brief Update segmented objects list.
   * @param list new segmented objects
   */
  void objectsCallback(const rail_manipulation_msgs::SegmentedObjectList &list);

  ros::NodeHandle n_, pnh_;

  // topics
  ros::Publisher debug_publisher_;
  ros::Subscriber test_subscriber_;
  ros::Subscriber test_agile_subscriber_;
  ros::Subscriber test_heuristic_subscriber_;
  ros::Subscriber test_random_subscriber_;
  ros::Subscriber objects_subscriber_;

  // services
  ros::ServiceClient suggest_grasps_client_;
  ros::ServiceClient suggest_grasps_baseline_client_;
  ros::ServiceClient suggest_grasps_random_client_;
  ros::ServiceClient rank_grasps_client_;

  // actionlib
  actionlib::SimpleActionClient<fetch_grasp_suggestion::ExecuteGraspAction> execute_grasp_client_;

  boost::mutex object_list_mutex_;

  rail_manipulation_msgs::SegmentedObjectList object_list_;

  bool debug_;
};

#endif  // FETCH_GRASP_SUGGESTION_TEST_GRASP_SUGGESTION_H
