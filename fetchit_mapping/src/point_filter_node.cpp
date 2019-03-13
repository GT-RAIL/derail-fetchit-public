#include <ros/ros.h>
#include "fetchit_mapping/PointFilters.h"

using namespace fetchit_mapping;

int main(int argc, char** argv){
    ros::init(argc, argv, "point_filter");
    ros::NodeHandle filter_nh;

    // get some launch args
    string point_topic;
    string point_frame;
    string map_frame;
    string odom_frame;
    float radius;

    filter_nh.getParam("/point_filter/point_topic", point_topic);
    filter_nh.getParam("/point_filter/point_frame", point_frame);
    filter_nh.getParam("/point_filter/map_frame", map_frame);
    filter_nh.getParam("/point_filter/odom_frame", odom_frame);
    filter_nh.getParam("/point_filter/radius", radius);

    PointFilter2D point_filter2d(filter_nh, point_topic, point_frame, map_frame, odom_frame, radius);

    try{
        ros::spin();
    }catch(std::runtime_error& e){
        ROS_ERROR("point_filter_node exception: %s", e.what());
        return -1;
    }

    return 0;
}