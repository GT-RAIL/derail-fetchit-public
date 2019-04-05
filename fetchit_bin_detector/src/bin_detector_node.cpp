#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rail_manipulation_msgs/SegmentObjects.h"
#include "ApproxMVBB/ComputeApproxMVBB.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "bin_detector_node");
    ros::NodeHandle detector_nh;

    ros::ServiceClient seg_client = detector_nh.serviceClient<rail_manipulation_msgs::SegmentObjects>("/segmenter/segment_objects");
    rail_manipulation_msgs::SegmentObjects seg_srv;
    ros::Publisher vis_pub = detector_nh.advertise<visualization_msgs::Marker>("bounding_volumes",0);

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
        if ( (oobb.volume() < 0.0053883) || (0.006483 < oobb.volume()) ) {
            continue;
        }
        // creates a bb marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.ns = "seg_objects";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        ROS_INFO("visualizing");
        // gets scale
        ApproxMVBB::Vector3 min_p = oobb.m_minPoint;
        ApproxMVBB::Vector3 max_p = oobb.m_maxPoint;
        marker.scale.x = max_p.x() - min_p.x();
        marker.scale.y = max_p.y() - min_p.y();
        marker.scale.z = max_p.z() - min_p.z();
        // gets pose
        ApproxMVBB::Vector3 box_center;
        box_center.x() = min_p.x() + (max_p.x() - min_p.x())/2.0;
        box_center.y() = min_p.y() + (max_p.y() - min_p.y())/2.0;
        box_center.z() = min_p.z() + (max_p.z() - min_p.z())/2.0;
        ApproxMVBB::Vector3 box_pose = oobb.m_q_KI * box_center;
        marker.pose.position.x = box_pose.x();
        marker.pose.position.y = box_pose.y();
        marker.pose.position.z = box_pose.z();
        marker.pose.orientation.x = oobb.m_q_KI.x();
        marker.pose.orientation.y = oobb.m_q_KI.y();
        marker.pose.orientation.z = oobb.m_q_KI.z();
        marker.pose.orientation.w = oobb.m_q_KI.w();
        // colors and publishes
        marker.color.a = 0.3;
        marker.color.r = seg_srv.response.segmented_objects.objects[i].rgb[0];
        marker.color.g = seg_srv.response.segmented_objects.objects[i].rgb[1];
        marker.color.b = seg_srv.response.segmented_objects.objects[i].rgb[2];
        ROS_INFO("volume: %f",oobb.volume());
        ROS_INFO("green: %lf",seg_srv.response.segmented_objects.objects[i].rgb[1]);
        vis_pub.publish( marker);
    }

    try{
        ros::spin();
    }catch(std::runtime_error& e){
        ROS_ERROR("bin_detector_node exception: %s", e.what());
        return -1;
    }

    return 0;
}