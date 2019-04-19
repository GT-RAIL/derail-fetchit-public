#ifndef CLUTTERED_GRASPER_H
#define CLUTTERED_GRASPER_H

// C++
#include <fstream>
#include <iostream>

// Boost
#include <boost/thread/mutex.hpp>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <manipulation_actions/AttachArbitraryObject.h>
#include <manipulation_actions/BinPickAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/GetCartesianPath.h>
#include <moveit_msgs/GetPlanningScene.h>
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

class ClutteredGrasper
{

public:
    ClutteredGrasper();

private:

    void executeBlind(const manipulation_actions::BinPickGoalConstPtr &goal);

    void executeSmart(const manipulation_actions::BinPickGoalConstPtr &goal);

    void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg);

    void objectsCallback(const rail_manipulation_msgs::SegmentedObjectList &list);

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
    void sampleGraspCandidates(sensor_msgs::PointCloud2 object, std::string object_source_frame,
        std::string environment_source_frame, geometry_msgs::PoseArray &grasps_out,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out);

    void sampleGraspCandidates2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, geometry_msgs::PoseArray &grasps_out);

    bool executeCartesianMove(geometry_msgs::PoseStamped goal);

    ros::NodeHandle n_, pnh_;

    // topics
    ros::Publisher grasps_publisher_;
    ros::Publisher box_pose_publisher;
    ros::Publisher current_grasp_publisher;
    ros::Publisher sample_cloud_publisher;
    ros::Publisher planning_scene_publisher_;
    ros::Subscriber cloud_subscriber_;
    ros::Subscriber objects_subscriber_;
    ros::Subscriber grasp_feedback_subscriber_;

    // services
    ros::ServiceClient attach_arbitrary_object_client;
    ros::ServiceClient planning_scene_client_;
    ros::ServiceClient cartesian_path_client;

    // actionlib
    actionlib::SimpleActionClient<rail_grasp_calculation_msgs::SampleGraspsAction> sample_grasps_client_;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_client;
    actionlib::SimpleActionServer<manipulation_actions::BinPickAction> blind_bin_pick_server;
    actionlib::SimpleActionServer<manipulation_actions::BinPickAction> smart_bin_pick_server;

    moveit::planning_interface::MoveGroupInterface *arm_group;

    tf::TransformListener tf_listener_;

    boost::mutex cloud_mutex_;
    boost::mutex stored_grasp_mutex_;
    boost::mutex object_list_mutex_;

    rail_manipulation_msgs::SegmentedObjectList object_list_;

    sensor_msgs::PointCloud2 stored_object_cloud_;
    sensor_msgs::PointCloud2 stored_scene_cloud_;
    std::vector<int> selected_grasps_;
    std::vector<double> object_features_;
    double min_grasp_depth_, max_grasp_depth_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

    bool cloud_received_;

    std::vector<std::string> gripper_names_;

    geometry_msgs::Vector3 box_dims;
    double box_error_threshold;
};

#endif // CLUTTERED_GRASPER_H