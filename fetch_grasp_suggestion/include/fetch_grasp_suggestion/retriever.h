#ifndef FETCH_GRASP_SUGGESTION_RETREIVER_H
#define FETCH_GRASP_SUGGESTION_RETREIVER_H

// C++
#include <iostream>

// Boost
#include <boost/thread/mutex.hpp>

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <fetch_grasp_suggestion/common.h>
#include <fetch_grasp_suggestion/RetrieveGrasps.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <rail_manipulation_msgs/SegmentedObject.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


class Retriever
{
public:
    Retriever();

private:
    bool retrieveGraspsCallback(fetch_grasp_suggestion::RetrieveGrasps::Request &req,
        fetch_grasp_suggestion::RetrieveGrasps::Response &res);

    // Attributes
    ros::NodeHandle n_, pn_;

    // services
    ros::ServiceServer retrieve_grasps_service_;

    // other things
    tf::TransformListener tf_listener_;

    double min_grasp_depth_, max_grasp_depth_;
};
#endif //FETCH_GRASP_SUGGESTION_RETREIVER_H
