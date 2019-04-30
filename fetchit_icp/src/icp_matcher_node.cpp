#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "icp_matcher_node");
    ros::NodeHandle matcher_nh;
    std::string template_path = ros::package::getPath("fetchit_icp")+"/cad_models/";

    // gets template pcd file
    std::string template_file = "schuck_and_wall.pcd";
    matcher_nh.getParam("/icp_matcher_node/mesh_file", template_file);
    const std::string template_filepath = template_path+template_file;

    // gets environment matching pcd
    std::string test_file = "test_mesh_easy.pcd";
    const std::string test_filepath = template_path+test_file;

    // declare data structures
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr matched_template_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // load data structures with points
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(template_filepath,*template_cloud) < 0) {
        ROS_ERROR("Could not load template PCD.");
        return -1;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(test_filepath,*test_cloud) < 0) {
        ROS_ERROR("Could not load test PCD.");
        return -1;
    }

    // prepare ICP
    pcl::IterativeClosestPoint<pcl::PointXYZRGB,pcl::PointXYZRGB> icp;
    icp.setInputSource(template_cloud);
    icp.setInputTarget(test_cloud);

    int iters = 50;
    float dist = 1.0;
    float trans = 1e-8;
    float fit = 1;
    matcher_nh.getParam("/icp_matcher_node/iterations",iters);
    matcher_nh.getParam("/icp_matcher_node/max_distance",dist);
    matcher_nh.getParam("/icp_matcher_node/trans_epsilon",trans);
    matcher_nh.getParam("/icp_matcher_node/fit_epsilon",fit);

    icp.setMaximumIterations(iters);
    icp.setMaxCorrespondenceDistance(dist);
    icp.setTransformationEpsilon(trans);
    icp.setEuclideanFitnessEpsilon(fit);
    // perform ICP
    icp.align(*matched_template_cloud);

    // prepare point clouds to publish
    ros::Publisher pub_temp = matcher_nh.advertise<sensor_msgs::PointCloud2>("template_points",0);
    ros::Publisher pub_test = matcher_nh.advertise<sensor_msgs::PointCloud2>("test_points",0);
    ros::Publisher pub_mtemp = matcher_nh.advertise<sensor_msgs::PointCloud2>("matched_template_points",0);
    sensor_msgs::PointCloud2 template_msg;
    sensor_msgs::PointCloud2 test_msg;
    sensor_msgs::PointCloud2 matched_template_msg;
    pcl::toROSMsg(*template_cloud,template_msg);
    pcl::toROSMsg(*test_cloud,test_msg);
    pcl::toROSMsg(*matched_template_cloud,matched_template_msg);
    template_msg.header.frame_id = "icp";
    test_msg.header.frame_id = "icp";
    matched_template_msg.header.frame_id = "icp";

    try{
        ros::Rate loop_rate(5);
        while (ros::ok())
        {
            ros::spinOnce();
            pub_temp.publish(template_msg);
            pub_test.publish(test_msg);
            pub_mtemp.publish(matched_template_msg);
            loop_rate.sleep();
        }
    }catch(std::runtime_error& e){
        ROS_ERROR("icp_matcher_node exception: %s", e.what());
        return -1;
    }

    return 0;
}