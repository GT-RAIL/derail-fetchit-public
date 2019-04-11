#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <gsl/gsl_fit.h>

#include <vector>
#include <utility>

#include "rail_manipulation_msgs/SegmentObjects.h"
#include "ApproxMVBB/ComputeApproxMVBB.hpp"

void minExtent(ApproxMVBB::OOBB& bb, ApproxMVBB::Vector3::Index& i) {
    (bb.m_maxPoint - bb.m_minPoint).minCoeff(&i);
}

void setZAxisShortest(ApproxMVBB::OOBB& bb) {
    ApproxMVBB::Vector3::Index i;
    minExtent(bb,i);
    if (i<2) {
        bb.switchZAxis(static_cast<unsigned int>(i));
    }
}

ApproxMVBB::Vector3 get_box_scale(ApproxMVBB::OOBB& bb) {
    ApproxMVBB::Vector3 min_p = bb.m_minPoint;
    ApproxMVBB::Vector3 max_p = bb.m_maxPoint;
    ApproxMVBB::Vector3 scale;
    scale.x() = max_p.x() - min_p.x();
    scale.y() = max_p.y() - min_p.y();
    scale.z() = max_p.z() - min_p.z();
    return scale;
}

ApproxMVBB::Vector3 get_box_center(ApproxMVBB::OOBB& bb) {
    ApproxMVBB::Vector3 min_p = bb.m_minPoint;
    ApproxMVBB::Vector3 max_p = bb.m_maxPoint;
    ApproxMVBB::Vector3 box_center;
    box_center.x() = min_p.x() + (max_p.x() - min_p.x())/2.0;
    box_center.y() = min_p.y() + (max_p.y() - min_p.y())/2.0;
    box_center.z() = min_p.z() + (max_p.z() - min_p.z())/2.0;
    return box_center;
}

ApproxMVBB::Vector3 get_box_pose(ApproxMVBB::OOBB& bb) {
    ApproxMVBB::Vector3 box_center = get_box_center(bb);
    ApproxMVBB::Vector3 box_pose = bb.m_q_KI * box_center;
    return box_pose;
}

void visualize_bb(ros::Publisher& pub, int id, ApproxMVBB::OOBB& bb, std::vector<float>& rgb, ApproxMVBB::Quaternion orient) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "seg_objects";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    ROS_INFO("visualizing");
    // gets scale
    ApproxMVBB::Vector3 box_scale = get_box_scale(bb);
    marker.scale.x = box_scale.x();
    marker.scale.y = box_scale.y();
    marker.scale.z = box_scale.z();
    // gets pose
    ApproxMVBB::Vector3 box_pose = get_box_pose(bb);
    marker.pose.position.x = box_pose.x();
    marker.pose.position.y = box_pose.y();
    marker.pose.position.z = box_pose.z();
    marker.pose.orientation.x = bb.m_q_KI.x();
    marker.pose.orientation.y = bb.m_q_KI.y();
    marker.pose.orientation.z = bb.m_q_KI.z();
    marker.pose.orientation.w = bb.m_q_KI.w();
    // colors and publishes
    marker.color.a = 0.3;
    marker.color.r = rgb[0];
    marker.color.g = rgb[1];
    marker.color.b = rgb[2];
    pub.publish(marker);
    // publish a tf of the pose
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "base_link";
    static_transformStamped.child_frame_id = "bin_"+std::to_string(id);
    static_transformStamped.transform.translation.x = marker.pose.position.x;
    static_transformStamped.transform.translation.y = marker.pose.position.y;
    static_transformStamped.transform.translation.z = marker.pose.position.z;
    static_transformStamped.transform.rotation.x = orient.x();
    static_transformStamped.transform.rotation.y = orient.y();
    static_transformStamped.transform.rotation.z = orient.z();
    static_transformStamped.transform.rotation.w = orient.w();
    static_broadcaster.sendTransform(static_transformStamped);
}

ApproxMVBB::Quaternion get_bin_orientation(ApproxMVBB::OOBB& bb, pcl::PointCloud<pcl::PointXYZRGB> cloud, ros::Publisher pub) {
    // selects points to get handle orientation
    std::vector<double> handle_points_x;
    std::vector<double> handle_points_y;
    ApproxMVBB::Vector3 bin_bottom = bb.m_q_KI * bb.m_minPoint;
    // transform handle points from base_link frame to
    for (int j=0;j<cloud.size();j++) {
        if (cloud[j].z > bin_bottom.z()+0.09525) {
            handle_points_x.push_back(cloud[j].x);
            handle_points_y.push_back(cloud[j].y);
        }
    }
    // fits line to points
    double m,b,cov00,cov01,cov11,sumsq;
    gsl_fit_linear(handle_points_x.data(),1,handle_points_y.data(),1,handle_points_x.size(),&b,&m,&cov00,&cov01,&cov11,&sumsq);
    // gets slope for both bin axis
    ApproxMVBB::Vector3 bin_bottom2 = bb.m_minPoint;
    ApproxMVBB::Vector3 bin_bottom3 = bb.m_maxPoint;
    bin_bottom3.x() = bb.m_minPoint.x();
    bin_bottom3.z() = bb.m_minPoint.z();

    bin_bottom2 = bb.m_q_KI * bin_bottom2;
    bin_bottom3 = bb.m_q_KI * bin_bottom3;

    float slope_ax_y = (bin_bottom3.y() - bin_bottom2.y())  /  (bin_bottom3.x() - bin_bottom2.x());
    float slope_ax_x = -1.0 / slope_ax_y;
    // checks which axis handle aligned with
    ApproxMVBB::Quaternion bin_orientation = bb.m_q_KI;
    ApproxMVBB::Quaternion adjust_orientation = ApproxMVBB::Quaternion(1,0,0,0);
    if ( fabs(slope_ax_y-m) < fabs(slope_ax_x-m) ) {
        ROS_INFO("aligned with y axis, no rotation applied");
    } else {
        ROS_INFO("aligned with x axis, rotate by 90 about z");
        adjust_orientation = ApproxMVBB::Quaternion(0.7071068,0,0,0.7071068);
        bin_orientation = bin_orientation * adjust_orientation;
    }
    // postion of second wall from origin centered at bin
    ApproxMVBB::Vector3 swc_bin = ApproxMVBB::Vector3(0.0635,0,0.0111125);
    // rotates if needed
    swc_bin = adjust_orientation * swc_bin;
    // puts in base_link frame
    ApproxMVBB::Vector3 swc_base_link = bb.m_q_KI * (get_box_center(bb) + swc_bin);

    int num_inliers = 0;
    pcl::PointCloud<pcl::PointXYZRGB> handle_cloud;
    sensor_msgs::PointCloud2 handle_msg;
    for (int j=0;j<cloud.size();j++) {
        double distance = sqrt(pow(swc_base_link.x()-cloud[j].x,2)+pow(swc_base_link.y()-cloud[j].y,2)+pow(swc_base_link.z()-cloud[j].z,2));
        if (distance <= 0.0254) {
            num_inliers += 1;
            handle_cloud.push_back(cloud[j]);
        }
    }
    if ( num_inliers < 300 ) {
        ROS_INFO("second wall misaligned, rotate by 180 about z");
        adjust_orientation = ApproxMVBB::Quaternion(0,0,0,1);
        bin_orientation = bin_orientation * adjust_orientation;
    } else {
        ROS_INFO("second wall aligned with x axis, no extra rotation applied");
    }
    pcl::toROSMsg(handle_cloud,handle_msg);
    handle_msg.header.frame_id = "base_link";
    pub.publish(handle_msg);
    ROS_INFO("Number of points at near x axis second wall: %d",num_inliers);
    return bin_orientation;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "bin_detector_node");
    ros::NodeHandle detector_nh;

    ros::ServiceClient seg_client = detector_nh.serviceClient<rail_manipulation_msgs::SegmentObjects>("/segmenter/segment_objects");
    rail_manipulation_msgs::SegmentObjects seg_srv;
    ros::Publisher vis_pub = detector_nh.advertise<visualization_msgs::Marker>("bounding_volumes",0);
    // temporary
    ros::Publisher pcl_pub = detector_nh.advertise<sensor_msgs::PointCloud2>("handle_points",0);

    if (!seg_client.call(seg_srv)) {
        ROS_ERROR("Failed to call segmentation service segment_objects");
        return -1;
    }
    ROS_INFO("Producing markers");

    ROS_INFO("Number segmented objects: %lu",seg_srv.response.segmented_objects.objects.size());
    for (int i=0;i<seg_srv.response.segmented_objects.objects.size();i++) {
        // converts point cloud to asr library compatible type
        pcl::PointCloud<pcl::PointXYZRGB> object_pcl_cloud;
        pcl::fromROSMsg(seg_srv.response.segmented_objects.objects[i].point_cloud,object_pcl_cloud);
        ApproxMVBB::Matrix3Dyn points(3,object_pcl_cloud.size());
        ROS_INFO("converting");
        for (int j=0;j<object_pcl_cloud.size();j++) {
            points(0,j) = object_pcl_cloud[j].x;
            points(1,j) = object_pcl_cloud[j].y;
            points(2,j) = object_pcl_cloud[j].z;
        }
        ROS_INFO("calculating");
        // gets the min bb
        double tolerance = 0.001;
        ApproxMVBB::OOBB oobb = ApproxMVBB::approximateMVBB(points,tolerance,200,3,0,1);
        // volume check (avg 0.0059183, std 0.0002650, 12 trials)
        ROS_INFO("volume: %f",oobb.volume());
        if ( (oobb.volume() < 0.005123) || (0.006748 < oobb.volume()) ) {
            continue;
        }
        // re-orients axes
        setZAxisShortest(oobb);
        // get absolute orientation
        ApproxMVBB::Quaternion bin_orient = get_bin_orientation(oobb,object_pcl_cloud,pcl_pub);
        // creates a bb marker
        visualize_bb(vis_pub,i,oobb,seg_srv.response.segmented_objects.objects[i].rgb,bin_orient);
    }

    try{
        ros::spin();
    }catch(std::runtime_error& e){
        ROS_ERROR("bin_detector_node exception: %s", e.what());
        return -1;
    }

    return 0;
}