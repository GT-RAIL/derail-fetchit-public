#include "fetchit_mapping/PointFilters.h"


namespace fetchit_mapping {

    PointFilter2D::PointFilter2D(ros::NodeHandle& nh, string point_topic, string point_frame, string map_frame, string odom_frame, float radius) {
        // initializes default class members
        filter_nh_ = nh;
        point_topic_ = point_topic;
        point_frame_ = point_frame;
        map_frame_ = map_frame;
        odom_frame_ = odom_frame;
        radius_ = radius;
        // start subscribing to points topic and publishing filtered points topics
        raw_points_sub_.subscribe(filter_nh_, point_topic, 1);
        tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(raw_points_sub_, tf_req_, map_frame_, 1);
        tf_filter_->registerCallback( boost::bind(&PointFilter2D::filter_points_, this, _1) );
        filtered_points_pub_ = filter_nh_.advertise<sensor_msgs::LaserScan>("/base_scan_static", 1);
    }

    void PointFilter2D::filter_points_(const boost::shared_ptr<const sensor_msgs::LaserScan>& input_scan) {
        sensor_msgs::PointCloud2 laser_cloud;
        sensor_msgs::LaserScan output_scan = *input_scan;

        tf_req_.waitForTransform(point_frame_,map_frame_,ros::Time::now(),ros::Duration(1.0));
        // request points in global frame
        try{
            projector_.transformLaserScanToPointCloud(map_frame_, *input_scan, laser_cloud, tf_req_);
        } catch(tf::TransformException& ex) {
            // try odom frame until map frame gets published
            try{
                ROS_WARN("Transform of laser_scan to laser_cloud with map_frame not available yet, using odom_frame");
                projector_.transformLaserScanToPointCloud(odom_frame_, *input_scan, laser_cloud, tf_req_);
            } catch(tf::TransformException& ex) {
                ROS_ERROR("Transform of laser_scan to laser_cloud got %s error status... Error recovery?",ex.what());
            }
        }

        // filter points not within r map radius
        const int i_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "index");
        const int x_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "x");
        const int y_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "y");
        const int i_idx_offset = laser_cloud.fields[i_idx_c].offset;
        const int x_idx_offset = laser_cloud.fields[x_idx_c].offset;
        const int y_idx_offset = laser_cloud.fields[y_idx_c].offset;
        const int pstep = laser_cloud.point_step;
        const long int pcount = laser_cloud.width * laser_cloud.height;
        const long int limit = pstep * pcount;
        int i_idx, x_idx, y_idx, z_idx;
        for (i_idx = i_idx_offset,
             x_idx = x_idx_offset,
             y_idx = y_idx_offset;

             x_idx < limit;

             i_idx += pstep,
             x_idx += pstep,
             y_idx += pstep) {

            float x = *((float*)(&laser_cloud.data[x_idx]));
            float y = *((float*)(&laser_cloud.data[y_idx]));
            int index = *((int*)(&laser_cloud.data[i_idx]));

            if(PointFilter2D::not_in_radius(x,y)){
                output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
            }
        }

        // publish new points
        filtered_points_pub_.publish(output_scan);
    }

    bool PointFilter2D::not_in_radius(float x, float y) {
        return sqrt(x*x+y*y) > radius_;
    }

}