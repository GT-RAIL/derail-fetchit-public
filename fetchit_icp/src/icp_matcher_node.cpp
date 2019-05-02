#include <ros/ros.h>
#include <ros/package.h>
#include <ros/time.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/transforms.h>

#include <geometry_msgs/Transform.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "icp_matcher_node");
    ros::NodeHandle matcher_nh;
    std::string template_path = ros::package::getPath("fetchit_icp")+"/cad_models/";
    std::string test_path = ros::package::getPath("fetchit_icp")+"/schunk_clouds/";

    // loads ICP params
    int iters = 50;
    float dist = 1.0;
    float trans = 1e-8;
    float fit = 1;
    matcher_nh.getParam("/icp_matcher_node/iterations",iters);
    matcher_nh.getParam("/icp_matcher_node/max_distance",dist);
    matcher_nh.getParam("/icp_matcher_node/trans_epsilon",trans);
    matcher_nh.getParam("/icp_matcher_node/fit_epsilon",fit);

    // gets template pcd file
    std::string template_file = "corner.pcd";
    matcher_nh.getParam("/icp_matcher_node/template_file", template_file);
    const std::string template_filepath = template_path+template_file;

    // declare data structures
    tf::TransformListener tf_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_cloud_local(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_cloud_world(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr matched_template_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    ros::Duration(5, 0).sleep();

    // loads template
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(template_filepath,*template_cloud) < 0) {
        ROS_ERROR("Could not load template PCD.");
        return -1;
    }

    // gets current point cloud
    boost::shared_ptr<sensor_msgs::PointCloud2 const> sharedMsg;
    sensor_msgs::PointCloud2 test_cloud_msg;
    sharedMsg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/head_camera/depth_registered/points");
    if(sharedMsg != NULL){
        test_cloud_msg = *sharedMsg;
    } else {
        ROS_ERROR("Could not get point cloud message from topic. Boost shared pointer is NULL.");
        return -1;
    }
    pcl::fromROSMsg(test_cloud_msg,*test_cloud_local);

    // transforms point cloud to fixed frame
    pcl_ros::transformPointCloud("map", ros::Time(0), *test_cloud_local, test_cloud_msg.header.frame_id,
                                 *test_cloud_world, tf_);

    // gets the initial_estimate for schunk corner from the launch
    std::string initial_estimate_string = "0 0 0 0 0 0";
    matcher_nh.getParam("/icp_matcher_node/initial_estimate", initial_estimate_string);
    std::vector<float> pose;
    std::istringstream initial_estimate_string_stream(initial_estimate_string);
    for(std::string value_string; initial_estimate_string_stream >> value_string;)
        pose.push_back(std::stof(value_string));

    // makes a tf for the initial_estimate
    tf::Transform initial_estimate;
    initial_estimate.setOrigin(tf::Vector3(pose[0],pose[1],pose[2]));
    initial_estimate.setRotation(tf::Quaternion(pose[4],pose[5],pose[3]));

    // transforms template by initial_estimate
    pcl_ros::transformPointCloud(*template_cloud,*template_cloud,initial_estimate);

    // TODO DEBUG
    ros::Publisher pub_temp = matcher_nh.advertise<sensor_msgs::PointCloud2>("template_points",0);
    ros::Publisher pub_test = matcher_nh.advertise<sensor_msgs::PointCloud2>("test_points",0);
    sensor_msgs::PointCloud2 template_msg;
    sensor_msgs::PointCloud2 test_msg;
    pcl::toROSMsg(*template_cloud,template_msg);
    pcl::toROSMsg(*test_cloud_world,test_msg);
    template_msg.header.frame_id = "map";
    test_msg.header.frame_id = "map";
    pub_temp.publish(template_msg);
    pub_test.publish(test_msg);
    // TODO DEBUG

    // prepare ICP
    pcl::IterativeClosestPoint<pcl::PointXYZRGB,pcl::PointXYZRGB> icp;
    icp.setInputSource(template_cloud);
    icp.setInputTarget(test_cloud_world);
    icp.setMaximumIterations(iters);
    icp.setMaxCorrespondenceDistance(dist);
    icp.setTransformationEpsilon(trans);
    icp.setEuclideanFitnessEpsilon(fit);

    // perform ICP to refine template pose
    icp.align(*matched_template_cloud);
    Eigen::Matrix4f icp_tf = icp.getFinalTransformation();
    tf::Transform tf_refinement = tf::Transform(tf::Matrix3x3(icp_tf(0,0),icp_tf(0,1),icp_tf(0,2),
                                                                 icp_tf(1,0),icp_tf(1,1),icp_tf(1,2),
                                                                 icp_tf(2,0),icp_tf(2,1),icp_tf(2,2)),
                                                   tf::Vector3(icp_tf(0,3),icp_tf(1,3),icp_tf(2,3)));

    // accounts for the chuck offset from the schunk corner
    tf::Transform tf_chuck_offset;
    tf_chuck_offset.setOrigin(tf::Vector3(0.289,0.118,0.148));
    tf_chuck_offset.setRotation(tf::Quaternion(0,-0.7071068,0,0.7071068));
    tf::Transform tf_chuck = tf_refinement * initial_estimate * tf_chuck_offset;

    // DEBUG
    tf::Vector3 chuck_trans = tf_chuck.getOrigin();
    tf::Quaternion chuck_rot = tf_chuck.getRotation();
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "map";
    static_transformStamped.child_frame_id = "guess";
    static_transformStamped.transform.translation.x = chuck_trans.x();
    static_transformStamped.transform.translation.y = chuck_trans.y();
    static_transformStamped.transform.translation.z = chuck_trans.z();
    static_transformStamped.transform.rotation.x = chuck_rot.x();
    static_transformStamped.transform.rotation.y = chuck_rot.y();
    static_transformStamped.transform.rotation.z = chuck_rot.z();
    static_transformStamped.transform.rotation.w = chuck_rot.w();
    // DEBUG

    // prepare point clouds to publish
    ros::Publisher pub_mtemp = matcher_nh.advertise<sensor_msgs::PointCloud2>("matched_template_points",0);
    sensor_msgs::PointCloud2 matched_template_msg;
    pcl::toROSMsg(*matched_template_cloud,matched_template_msg);
    matched_template_msg.header.frame_id = "map";

    try{
        ros::Rate loop_rate(5);
        while (ros::ok())
        {
            ros::spinOnce();
            static_broadcaster.sendTransform(static_transformStamped);
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