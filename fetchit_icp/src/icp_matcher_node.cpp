#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "icp_matcher_node");
    ros::NodeHandle matcher_nh;
    // get file names
    std::string mesh_file = "schuck_and_wall.ply";
    matcher_nh.getParam("/icp_matcher_node/mesh_file", mesh_file);

    std::string mesh_path = ros::package::getPath("fetchit_icp")+"/cad_models/";
    const std::string mesh_filepath = mesh_path+mesh_file;

    // declare data structures
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 cloud_msg;

    // initialize data structures
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(mesh_filepath,cloud) < 0) {
        ROS_ERROR("Could not load PCD.");
        return -1;
    }

    // publish points
    pcl::toROSMsg(cloud,cloud_msg);
    ros::Publisher pub = matcher_nh.advertise<sensor_msgs::PointCloud2>("mesh_points",0);
    cloud_msg.header.frame_id = "icp";
    pub.publish(cloud_msg);

    try{
        ros::Rate loop_rate(5);
        while (ros::ok())
        {
            ros::spinOnce();
            pub.publish(cloud_msg);
            loop_rate.sleep();
        }
    }catch(std::runtime_error& e){
        ROS_ERROR("icp_matcher_node exception: %s", e.what());
        return -1;
    }

    return 0;
}