#ifndef FETCH_GRASP_SUGGESTION_EXECUTOR_H
#define FETCH_GRASP_SUGGESTION_EXECUTOR_H

// Boost
#include <boost/thread/mutex.hpp>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <control_msgs/GripperCommandAction.h>
#include <eigen_conversions/eigen_msg.h>
#include <fetch_grasp_suggestion/AddObject.h>
#include <fetch_grasp_suggestion/bounding_box_calculator.h>
#include <fetch_grasp_suggestion/ExecuteGraspAction.h>
#include <fetch_grasp_suggestion/PresetMoveAction.h>
#include <fetch_grasp_suggestion/PresetJointsMoveAction.h>
#include <moveit_msgs/GetCartesianPath.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/**
 * @brief Grasp execution, integrating with MoveIt!, in a single action call.
 */
class Executor
{

public:
  /**
   * @brief Initialize ROS messages, services, and actions needed for grasp execution.
   */
  Executor();

private:

  /**
   * @brief Execute a full grasping motion (move to approach, move to grasp pose, close gripper, lift object).
   * @param goal grasp pose to be executed
   */
  void executeGrasp(const fetch_grasp_suggestion::ExecuteGraspGoalConstPtr &goal);

  /**
   * @brief Move the arm to a preset out-of-the-way pose ready for grasping.
   * @param goal empty goal
   */
  void prepareRobot(const fetch_grasp_suggestion::PresetMoveGoalConstPtr &goal);

  /**
   * @brief Move the arm to the side at a preset pose for dropping objects.
   * @param goal empty goal
   */
  void dropPosition(const fetch_grasp_suggestion::PresetMoveGoalConstPtr &goal);

  /**
   * @brief Move the arm to a tuck position in preparation for navigation.
   * @param goal empty goal
   */
  void presetPosition(const fetch_grasp_suggestion::PresetJointsMoveGoalConstPtr &goal);

  /**
   * @brief Add a collision object to the MoveIt! planning scene.
   * @param req object to be added
   * @param res empty response
   * @return true on service call success
   */
  bool addObject(fetch_grasp_suggestion::AddObject::Request &req, fetch_grasp_suggestion::AddObject::Response &res);

  /**
   * @brief Detach all objects in the attached_objects_list from the gripper (service callback).
   * @param req empty request
   * @param res empty response
   * @return true on service call success
   */
  bool detachObjectsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /**
   * @brief Detach all objects in the attached_objects_ list from the gripper.
   */
  void detachObjects();

  /**
   * @brief Clear all added objects from the MoveIt! planning scene (service callback).
   * @param req empty request
   * @param res empty response
   * @return true on service call success
   */
  bool clearObjects(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /**
   * @brief Clear all added objects from the MoveIt! planning scene.
   */
  void clearAll();

  /**
   * @brief Open gripper, detach object, and reset collision objects.
   * @param req empty request
   * @param res empty response
   * @return true on service call success
   */
  bool dropObjectCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /**
   * @brief Turn on collisions between an object and the robot's fingers.
   * @param object object that needs collisions turned on again.
   */
  void enableGripperCollision(std::string object);

  ros::NodeHandle n_, pnh_;

  //messages
  ros::Publisher planning_scene_publisher_;
  ros::Publisher test1_;
  ros::Publisher test2_;

  //services
  ros::ServiceServer add_object_server_;
  ros::ServiceServer detach_objects_server_;
  ros::ServiceServer clear_objects_server_;
  ros::ServiceServer drop_object_server_;
  ros::ServiceClient compute_cartesian_path_client_;
  ros::ServiceClient planning_scene_client_;

  //actionlib
  actionlib::SimpleActionServer<fetch_grasp_suggestion::ExecuteGraspAction> execute_grasp_server_;
  actionlib::SimpleActionServer<fetch_grasp_suggestion::PresetMoveAction> prepare_robot_server_;
  actionlib::SimpleActionServer<fetch_grasp_suggestion::PresetMoveAction> drop_pose_server_;
  actionlib::SimpleActionServer<fetch_grasp_suggestion::PresetJointsMoveAction> preset_pose_server_;
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_client_;

  boost::mutex object_mutex_;

  //MoveIt interfaces
  moveit::planning_interface::MoveGroupInterface *arm_group_;
  moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  sensor_msgs::JointState ready_pose_;
  sensor_msgs::JointState drop_pose_;

  std::vector<std::string> gripper_names_;
  std::vector<std::string> attached_objects_;
};

#endif  // FETCH_GRASP_SUGGESTION_EXECUTOR_H
