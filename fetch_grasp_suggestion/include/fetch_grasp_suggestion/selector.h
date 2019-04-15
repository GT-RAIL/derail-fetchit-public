#ifndef FETCH_GRASP_SUGGESTION_SELECTOR_H
#define FETCH_GRASP_SUGGESTION_SELECTOR_H

// C++
#include <fstream>
#include <iostream>

// Boost
#include <boost/thread/mutex.hpp>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <fetch_grasp_suggestion/common.h>
#include <fetch_grasp_suggestion/CycleGrasps.h>
#include <fetch_grasp_suggestion/ExecuteGraspAction.h>
#include <fetch_grasp_suggestion/ExecuteSelectedGraspAction.h>
#include <fetch_grasp_suggestion/RankedGraspList.h>
#include <interactive_markers/interactive_marker_server.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <ros/ros.h>

/**
 * @brief Select and execute grasps from grasp suggestion via an interactive marker server
 */
class Selector
{

public:

  /**
   * @brief Initialize ROS messages, services, actions, and interactive marker server required for grasp selection
   */
  Selector();

private:

  /**
   * @brief Call the executor for the currently selected grasp.
   * @param goal empty goal
   */
  void executeSelectedGrasp(const fetch_grasp_suggestion::ExecuteSelectedGraspGoalConstPtr &goal);

  /**
   * @brief Update the suggested grasp list.
   * @param grasp_list new grasp list
   */
  void graspsCallback(const fetch_grasp_suggestion::RankedGraspList &grasp_list);

  /**
   * @brief Update segmented objects list.
   * @param list new segmented objects
   */
  void objectsCallback(const rail_manipulation_msgs::SegmentedObjectList &list);

  /**
   * Change the currently selected grasp.
   * @param req option to cycle forward or backward
   * @param res new grasp index
   * @return true on service call success
   */
  bool cycleGraspsCallback(fetch_grasp_suggestion::CycleGrasps::Request &req,
      fetch_grasp_suggestion::CycleGrasps::Response &res);

  /**
   * @brief Cycle forward one grasp.
   */
  void cycleGraspsForward();

  /**
   * @brief Cycle backward one grasp.
   */
  void cycleGraspsBackward();

  /**
   * @brief Update the displayed grasp marker to show the currently selected grasp.
   */
  void updateMarker();

  /**
   * @brief Clear out stored grasp data.
   */
  void resetGraspData();

  /**
   * @brief Create and save the full set of pairwise training instance from a single grasp selection.
   * @param selected_index index of the grasp selected
   */
  void logTrainingInstances(int selected_index);

  /**
   * @brief Create an interactive marker that looks like the Fetch's gripper.
   * @param pose the pose of the gripper
   * @return gripper interactive marker at the given pose
   */
  static visualization_msgs::InteractiveMarker makeGripperMarker(geometry_msgs::PoseStamped pose);

  /**
   * @brief Create a marker that represents a component of the Fetch's gripper
   * @param x x offset of mesh for gripper component
   * @param y y offset of mesh for gripper component
   * @param z z offset of mesh for gripper component
   * @param rx x quaternion component of mesh for gripper component
   * @param ry y quaternion component of mesh for gripper component
   * @param rz z quaternion component of mesh for gripper component
   * @param rw w quaternion component of mesh for gripper component
   * @param mesh_location file path to mesh
   * @return marker for the given gripper component
   */
  static visualization_msgs::Marker createGripperMeshMarker(double x, double y, double z,
      double rx, double ry, double rz, double rw, std::string mesh_location);

  ros::NodeHandle n_, pnh_;

  // topics
  ros::Subscriber grasps_subscriber_;
  ros::Subscriber objects_subscriber_;

  // services
  ros::ServiceServer cycle_grasps_server_;

  // actionlib
  actionlib::SimpleActionClient<fetch_grasp_suggestion::ExecuteGraspAction> execute_grasp_client_;
  actionlib::SimpleActionServer<fetch_grasp_suggestion::ExecuteSelectedGraspAction> execute_selected_grasp_server_;

  boost::mutex grasp_list_mutex_;

  // interactive markers
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server_;
  visualization_msgs::InteractiveMarker grasp_selector_marker_;
  visualization_msgs::Marker grasp_marker_;
  visualization_msgs::InteractiveMarkerControl grasp_marker_control_;

  rail_manipulation_msgs::SegmentedObjectList object_list_;
  fetch_grasp_suggestion::RankedGraspList grasp_list_;

  int grasp_index_;
  int max_index_seen_;  /// the greatest index looked at by the user, used for generating training instances
  std::vector<int> selected_grasps_;  /// running list of grasps selected for execution from the current list
  std::vector<double> object_features_;  /// calculated features for object that grasps relate to

  std::string filename_;  /// output file for saving training instances
  std::ofstream file_;
};

#endif  // FETCH_GRASP_SUGGESTION_SELECTOR_H
