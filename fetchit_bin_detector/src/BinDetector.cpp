#include "fetchit_bin_detector/BinDetector.h"

BinDetector::BinDetector(ros::NodeHandle& nh, const std::string& seg_srv, const std::string& seg_frame, bool viz){
    nh_ = nh;
    seg_frame_ = seg_frame;
    visualize_ = viz;

    bin_detected_ = false;
    best_bin_transform_.header.frame_id = seg_frame_;
    best_bin_transform_.child_frame_id = "kit_frame";

    table_received_ = false;

    table_sub_ = nh_.subscribe("/rail_segmentation/segmented_table", 1, &BinDetector::table_callback, this);

    seg_client_ = nh_.serviceClient<rail_manipulation_msgs::SegmentObjects>(seg_srv);
    pose_srv_ = nh_.advertiseService("detect_bins", &BinDetector::handle_bin_pose_service, this);
}

void BinDetector::table_callback(const rail_manipulation_msgs::SegmentedObject &msg)
{
    table_height_ = msg.center.z;
    table_received_ = true;
}

bool BinDetector::handle_bin_pose_service(fetchit_bin_detector::GetBinPose::Request& req, fetchit_bin_detector::GetBinPose::Response& res) {
    // initialize the bin pose array temporary pose variable
    std::vector<geometry_msgs::PoseStamped> bin_poses;
    geometry_msgs::PoseStamped new_bin_pose;
    new_bin_pose.header.frame_id = seg_frame_;

    // gets initial segmentation
    rail_manipulation_msgs::SegmentObjects seg_srv;
    table_received_ = false;
    if (!seg_client_.call(seg_srv)) {
        ROS_ERROR("Failed to call segmentation service segment_objects");
        return false;
    }
    ROS_INFO("Number segmented objects: %lu",seg_srv.response.segmented_objects.objects.size());

    // wait a couple seconds for table height info
    ros::Rate wait_rate(100);
    ros::Time wait_start = ros::Time::now();
    while (!table_received_ && ros::Time::now() - wait_start < ros::Duration(2.0))
    {
        ros::spinOnce();
        wait_rate.sleep();
    }
    if (!table_received_)
    {
        ROS_ERROR("Did not receive table info, couldn't set bin z bounds!");
        return true;
    }

    pcl::PointCloud<pcl::PointXYZRGB> object_pcl_cloud;
    double min_sqr_dst = std::numeric_limits<double>::infinity();  // for selecting the best (closest) bin to consider
    bin_detected_ = false;
    for (int i=0;i<seg_srv.response.segmented_objects.objects.size();i++) {
        // converts point cloud to asr library compatible type
        pcl::fromROSMsg(seg_srv.response.segmented_objects.objects[i].point_cloud,object_pcl_cloud);
        ApproxMVBB::Matrix3Dyn points(3,object_pcl_cloud.size());
        ROS_INFO("converting");
        for (int j=0;j<object_pcl_cloud.size();j++) {
            points(0,j) = object_pcl_cloud[j].x;
            points(1,j) = object_pcl_cloud[j].y;
            points(2,j) = object_pcl_cloud[j].z;
        }
        ROS_INFO("calculating");

        // gets the min vol b
        double tolerance = 0.001;
        ApproxMVBB::OOBB oobb = ApproxMVBB::approximateMVBB(points,tolerance,200,3,0,1);

        // re-orients bin coordinate frame so z always up
        setZAxisShortest(oobb);

        // set z-min to table height (the bottom of the bin)
        oobb.m_minPoint[2] = table_height_;

        ROS_INFO("************************************************************");
        std::cout << oobb.m_minPoint[0] << ", " << oobb.m_minPoint[1] << ", " << oobb.m_minPoint[2] << std::endl;
        std::cout << oobb.m_maxPoint[0] << ", " << oobb.m_maxPoint[1] << ", " << oobb.m_maxPoint[2] << std::endl;
        std::cout << oobb.volume() << std::endl;
        ROS_INFO("************************************************************");

        // volume check (avg 0.0059183, std 0.0002650, 12 trials)
        if ( (oobb.volume() < 0.005123) || (0.006748 < oobb.volume()) ) {
            continue;
        }

        // TODO shape check

        // get absolute orientation
        new_bin_pose.pose = get_bin_pose(oobb,object_pcl_cloud);
        bin_poses.push_back(new_bin_pose);

        double sqr_dst = pow(new_bin_pose.pose.position.x, 2) + pow(new_bin_pose.pose.position.y, 2)
            + pow(new_bin_pose.pose.position.z, 2);

        if (sqr_dst < min_sqr_dst)
        {
            bin_detected_ = true;
            min_sqr_dst = sqr_dst;
            best_bin_transform_.transform.translation.x = new_bin_pose.pose.position.x;
            best_bin_transform_.transform.translation.y = new_bin_pose.pose.position.y;
            best_bin_transform_.transform.translation.z = new_bin_pose.pose.position.z;
            best_bin_transform_.transform.rotation = new_bin_pose.pose.orientation;
        }
        // creates a bb marker (left for visualization/testing purposes, shouldn't be used for things like store object)
        if (visualize_) {
            visualize_bb(i,new_bin_pose.pose);
        }
    }
    res.bin_poses = bin_poses;
    return true;
}

geometry_msgs::Pose BinDetector::get_bin_pose(ApproxMVBB::OOBB& bb, pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
    // gets bin translation from bounding box
    ApproxMVBB::Vector3 bin_position = get_box_top(bb);

    // gets absolute bin orientation
    ApproxMVBB::Quaternion bin_orientation = bb.m_q_KI;
    ApproxMVBB::Quaternion adjust_orientation;

    // aligns y-axis to bin handle
    float handle_slope = get_handle_slope_from_cloud(bb,cloud);
    if ( slopeAlignedToXAxis(handle_slope,bb,cloud) ) {
        adjust_orientation = ApproxMVBB::Quaternion(0.7071068,0,0,0.7071068); // 90 degree rot about z
        bin_orientation = bin_orientation * adjust_orientation;
    }

    // aligns x-axis to bin second wall
    if ( !secondWallAlignedToXAxis(adjust_orientation,bb,cloud) ) {
        adjust_orientation = ApproxMVBB::Quaternion(0,0,0,1); // 180 degree rot about z
        bin_orientation = bin_orientation * adjust_orientation;
    }

    // loads return variable
    geometry_msgs::Pose bin_pose;
    bin_pose.position.x = bin_position.x();
    bin_pose.position.y = bin_position.y();
    bin_pose.position.z = bin_position.z();
    bin_pose.orientation.x = bin_orientation.x();
    bin_pose.orientation.y = bin_orientation.y();
    bin_pose.orientation.z = bin_orientation.z();
    bin_pose.orientation.w = bin_orientation.w();

    return bin_pose;
}

float BinDetector::get_handle_slope_from_cloud(ApproxMVBB::OOBB& bb, pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
    std::vector<double> handle_points_x;
    std::vector<double> handle_points_y;
    ApproxMVBB::Vector3 bin_bottom = bb.m_q_KI * bb.m_minPoint;
    float min_handle_height = bin_bottom.z()+0.09525;
    // selects points to get handle orientation
    for (int j=0;j<cloud.size();j++) {
        if (cloud[j].z > min_handle_height) {
            handle_points_x.push_back(cloud[j].x);
            handle_points_y.push_back(cloud[j].y);
        }
    }
    // fits line to points to get slope
    double slope,b,cov00,cov01,cov11,sumsq;
    gsl_fit_linear(handle_points_x.data(),1,handle_points_y.data(),1,handle_points_x.size(),&b,&slope,&cov00,&cov01,&cov11,&sumsq);
    return slope;
}

bool BinDetector::slopeAlignedToXAxis(float handle_slope, ApproxMVBB::OOBB& bb, pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
    bool aligned;
    // makes line segment along y-axis
    ApproxMVBB::Vector3 y_axis_p1 = bb.m_minPoint;
    ApproxMVBB::Vector3 y_axis_p2 = y_axis_p1;
    y_axis_p2.y() = bb.m_maxPoint.y();
    // puts line segment into the segmentation frame
    y_axis_p1 = bb.m_q_KI * y_axis_p1;
    y_axis_p2 = bb.m_q_KI * y_axis_p2;
    // gets slope of x & y axis in the segmentation frame
    float slope_ax_y = (y_axis_p2.y() - y_axis_p1.y())  /  (y_axis_p2.x() - y_axis_p1.x());
    float slope_ax_x = -1.0 / slope_ax_y;
    // checks which axis handle_slope is more aligned with
    if ( fabs(slope_ax_y-handle_slope) < fabs(slope_ax_x-handle_slope) ) {
        ROS_INFO("aligned with y axis, no rotation applied");
        aligned = false;
    } else {
        ROS_INFO("aligned with x axis, rotate by 90 about z");
        aligned = true;
    }
    return aligned;
}

bool BinDetector::secondWallAlignedToXAxis(ApproxMVBB::Quaternion adjust_orientation, ApproxMVBB::OOBB& bb, pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
    bool aligned;
    // postion of second wall from origin centered at bin
    ApproxMVBB::Vector3 swc_bin = ApproxMVBB::Vector3(0.0635,0,0.0111125);
    // rotates if needed
    swc_bin = adjust_orientation * swc_bin;
    // puts in segmentation frame
    ApproxMVBB::Vector3 swc_base_link = bb.m_q_KI * (get_box_center(bb) + swc_bin);
    // counts number of points matching expected model (inliers)
    int num_inliers = 0;
    double distance;
    for (int j=0;j<cloud.size();j++) {
        distance = sqrt(pow(swc_base_link.x()-cloud[j].x,2)+pow(swc_base_link.y()-cloud[j].y,2)+pow(swc_base_link.z()-cloud[j].z,2));
        if (distance <= 0.0254) {
            num_inliers += 1;
        }
    }
    if ( num_inliers < 300 ) {
        ROS_INFO("second wall misaligned, rotate by 180 about z");
        aligned = false;
    } else {
        ROS_INFO("second wall aligned with x axis, no extra rotation applied");
        aligned = true;
    }
    return aligned;
}

void BinDetector::minExtent(ApproxMVBB::OOBB& bb, ApproxMVBB::Vector3::Index& i) {
    (bb.m_maxPoint - bb.m_minPoint).minCoeff(&i);
}

void BinDetector::setZAxisShortest(ApproxMVBB::OOBB& bb) {
    ApproxMVBB::Vector3::Index i;
    minExtent(bb,i);
    if (i<2) {
        bb.switchZAxis(static_cast<unsigned int>(i));
    }
}

ApproxMVBB::Vector3 BinDetector::get_box_scale(ApproxMVBB::OOBB& bb) {
    ApproxMVBB::Vector3 min_p = bb.m_minPoint;
    ApproxMVBB::Vector3 max_p = bb.m_maxPoint;
    ApproxMVBB::Vector3 scale;
    scale.x() = max_p.x() - min_p.x();
    scale.y() = max_p.y() - min_p.y();
    scale.z() = max_p.z() - min_p.z();
    return scale;
}

ApproxMVBB::Vector3 BinDetector::get_box_center(ApproxMVBB::OOBB& bb) {
    ApproxMVBB::Vector3 min_p = bb.m_minPoint;
    ApproxMVBB::Vector3 max_p = bb.m_maxPoint;
    ApproxMVBB::Vector3 box_center;
    box_center.x() = min_p.x() + (max_p.x() - min_p.x())/2.0;
    box_center.y() = min_p.y() + (max_p.y() - min_p.y())/2.0;
    box_center.z() = min_p.z() + (max_p.z() - min_p.z())/2.0;
    return box_center;
}

ApproxMVBB::Vector3 BinDetector::get_box_pose(ApproxMVBB::OOBB& bb) {
    ApproxMVBB::Vector3 box_center = get_box_center(bb);
    ApproxMVBB::Vector3 box_pose = bb.m_q_KI * box_center;
    return box_pose;
}

ApproxMVBB::Vector3 BinDetector::get_box_top(ApproxMVBB::OOBB& bb) {
    ApproxMVBB::Vector3 box_center = get_box_center(bb);
    box_center.z() = bb.m_maxPoint.z();
    ApproxMVBB::Vector3 box_pose = bb.m_q_KI * box_center;
    return box_pose;
}

void BinDetector::visualize_bb(int id, geometry_msgs::Pose bin_pose) {
    // publish a tf of the pose
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = seg_frame_;
    static_transformStamped.child_frame_id = "bin_"+std::to_string(id);
    static_transformStamped.transform.translation.x = bin_pose.position.x;
    static_transformStamped.transform.translation.y = bin_pose.position.y;
    static_transformStamped.transform.translation.z = bin_pose.position.z;
    static_transformStamped.transform.rotation = bin_pose.orientation;
    static_broadcaster.sendTransform(static_transformStamped);
}

void BinDetector::publish_bin_tf()
{
    if (bin_detected_)
    {
        best_bin_transform_.header.stamp = ros::Time::now();
        tf_broadcaster_.sendTransform(best_bin_transform_);
    }

}
