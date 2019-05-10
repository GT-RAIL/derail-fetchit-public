#include <tf2/LinearMath/Matrix3x3.h>
#include "fetchit_bin_detector/BinDetector.h"

BinDetector::BinDetector(ros::NodeHandle& nh, const std::string& seg_node, const std::string& seg_frame, bool viz){
    nh_ = nh;
    seg_frame_ = seg_frame;
    visualize_ = viz;

    base_right_bin_transform_.header.frame_id = seg_frame_;       // NOTE: The hard-coded values only work for "base_link"
    base_right_bin_transform_.child_frame_id = "kit_frame";
    base_right_bin_transform_.transform.translation.x = 0.219;
    base_right_bin_transform_.transform.translation.y = -0.140;
    base_right_bin_transform_.transform.translation.z = 0.522;
    base_right_bin_transform_.transform.rotation.w = 1.0;

    base_left_bin_transform_ = base_right_bin_transform_;
    base_left_bin_transform_.transform.translation.x = 0.215;
    base_left_bin_transform_.transform.translation.y = 0.153;
    base_left_bin_transform_.transform.translation.z = 0.529;

    bin_detected_ = false;
    best_bin_transform_ = base_right_bin_transform_;

    table_received_ = false;

    table_sub_ = nh_.subscribe(seg_node+"/segmented_table", 1, &BinDetector::table_callback, this);

    seg_client_ = nh_.serviceClient<rail_manipulation_msgs::SegmentObjects>(seg_node+"/segment_objects");
    merge_client_ = nh_.serviceClient<rail_manipulation_msgs::ProcessSegmentedObjects>("merger/merge_objects");
    attach_base_client_ = nh_.serviceClient<manipulation_actions::AttachToBase>("collision_scene_manager/attach_to_base");
    detach_base_client_ = nh_.serviceClient<std_srvs::Empty>("collision_scene_manager/detach_all_from_base");
    pose_srv_ = nh_.advertiseService("detect_bins", &BinDetector::handle_bin_pose_service, this);

    //pub2_ = nh_.advertise<sensor_msgs::PointCloud2>("wall_points",0); // TODO DEBUG
}

void BinDetector::table_callback(const rail_manipulation_msgs::SegmentedObject &msg)
{
    table_height_ = msg.center.z;
    table_received_ = true;
}

bool BinDetector::handle_bin_pose_service(fetchit_bin_detector::GetBinPose::Request& req, fetchit_bin_detector::GetBinPose::Response& res)
{
    ros::Time begin = ros::Time::now();

    // initialize the bin pose array temporary pose variable
    std::vector<geometry_msgs::PoseStamped> bin_poses;
    geometry_msgs::PoseStamped new_bin_pose;
    new_bin_pose.header.frame_id = seg_frame_;

    // gets initial segmentation
    rail_manipulation_msgs::SegmentObjects seg_srv;
    table_received_ = false;
    if (!seg_client_.call(seg_srv))
    {
        ROS_ERROR("Failed to call segmentation service segment_objects");
        return false;
    }
    ROS_INFO("Number segmented objects: %lu", seg_srv.response.segmented_objects.objects.size());

    if (seg_srv.response.segmented_objects.objects.empty())
    {
        ROS_INFO("Returning, as no objects were found.");
        return true;
    }

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
        return false;
    }

    rail_manipulation_msgs::SegmentedObjectList segmented_objects = seg_srv.response.segmented_objects;
    rail_manipulation_msgs::ProcessSegmentedObjects merge_srv;
    merge_srv.request.segmented_objects = seg_srv.response.segmented_objects;
    if (!merge_client_.call(merge_srv))
    {
        ROS_ERROR("Could not call segmentation merging postprocessing service!  Continuing execution anyway...");
    } else
    {
        segmented_objects = merge_srv.response.segmented_objects;
    }
    ROS_INFO("Number segmented objects after merging: %lu", segmented_objects.objects.size());

    pcl::PointCloud<pcl::PointXYZRGB> object_pcl_cloud;
    double min_sqr_dst = std::numeric_limits<double>::infinity();  // for selecting the best (closest) bin to consider
    bin_detected_ = false;
    rail_manipulation_msgs::SegmentedObject attach_object;
    for (int i = 0; i < segmented_objects.objects.size(); i++)
    {
        // converts point cloud to asr library compatible type
        pcl::fromROSMsg(segmented_objects.objects[i].point_cloud, object_pcl_cloud);
        ApproxMVBB::Matrix3Dyn points(3, object_pcl_cloud.size());
        for (int j = 0; j < object_pcl_cloud.size(); j++)
        {
            points(0, j) = object_pcl_cloud[j].x;
            points(1, j) = object_pcl_cloud[j].y;
            points(2, j) = object_pcl_cloud[j].z;
        }

        // gets the min vol b
        double tolerance = 0.001;
        ApproxMVBB::OOBB oobb = ApproxMVBB::approximateMVBB(points, tolerance, 200, 3, 0, 1);

        ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");

        // re-orients bin coordinate frame so z axis is the shortest
        setZAxisShortest(oobb);

        // set z-min to table height (the bottom of the bin)
        ApproxMVBB::Vector3 table_point = oobb.m_q_KI * oobb.m_minPoint;
        ROS_INFO("min point in baselink frame: %f,%f,%f", table_point.x(), table_point.y(), table_point.z());
        table_point.z() = table_height_;
        ROS_INFO("table z height: %f", table_height_);
        table_point = oobb.m_q_KI.inverse() * table_point;
        ROS_INFO("table min point in bin frame: %f,%f,%f", table_point.x(), table_point.y(), table_point.z());
        ROS_INFO("bb min point in bin frame: %f,%f,%f", oobb.m_minPoint.x(), oobb.m_minPoint.y(), oobb.m_minPoint.z());

        oobb.m_minPoint[2] = table_point.z();

        // checks inverted z-axis case, and flips if needed
        ROS_INFO("z before invert: %f", oobb.getDirection(2).z());
        if (oobb.getDirection(2).z() < 0)
        {
            invertZAxis(oobb);
            ROS_INFO("z after invert: %f", oobb.getDirection(2).z());
        }

        // volume check (avg 0.0059183, std 0.0002650, 12 trials)
        ROS_INFO("********************************************");
        ROS_INFO("volume: %f", oobb.volume());
        if ((oobb.volume() < 0.005) || (0.01 < oobb.volume()))
        {
            continue;
        }

        // shape check (side: avg 0.228, std 0.005; height: avg 0.133 std 0.013)
        ApproxMVBB::Vector3 bb_shape = get_box_scale(oobb);
        ROS_INFO("shape x: %f", bb_shape.x());
        ROS_INFO("shape y: %f", bb_shape.y());
        ROS_INFO("shape z: %f", bb_shape.z());
        ROS_INFO("********************************************");
        if ((bb_shape.x() < 0.2) || (0.27 < bb_shape.x()))
        { // checks one side
            continue;
        }
        if ((bb_shape.y() < 0.2) || (0.27 < bb_shape.y()))
        { // checks other side
            continue;
        }
        //if ( (bb_shape.z() < 0.05) || (0.175 < bb_shape.z()) ) { // checks height
        //    continue;
        //}

        // get absolute orientation
        bool pose_extraction_success = get_bin_pose(oobb, object_pcl_cloud, new_bin_pose.pose);
        if (pose_extraction_success)
        {
            bin_poses.push_back(new_bin_pose);
        } else
        {
            return false;
        }

        double sqr_dst = pow(new_bin_pose.pose.position.x, 2) + pow(new_bin_pose.pose.position.y, 2)
                         + pow(new_bin_pose.pose.position.z, 2);

        if (sqr_dst < min_sqr_dst)
        {
            bin_detected_ = true;
            min_sqr_dst = sqr_dst;
            if (req.bin_location == fetchit_bin_detector::GetBinPose::Request::BIN_ON_BASE_RIGHT)
            {
                // We just want to use the hard-coded pose on the base of the robot
                best_bin_transform_ = base_right_bin_transform_;
            }
            else if (req.bin_location == fetchit_bin_detector::GetBinPose::Request::BIN_ON_BASE_LEFT)
            {
                // We just want to use the hard-coded pose on the base of the robot
                best_bin_transform_ = base_left_bin_transform_;
            }
            else
            {
                best_bin_transform_.transform.translation.x = new_bin_pose.pose.position.x;
                best_bin_transform_.transform.translation.y = new_bin_pose.pose.position.y;
                best_bin_transform_.transform.translation.z = new_bin_pose.pose.position.z;
                best_bin_transform_.transform.rotation = new_bin_pose.pose.orientation;
            }
            attach_object = segmented_objects.objects[i];
        }

        ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");

        // creates a bb marker for each bin detected (left for visualization/testing purposes, shouldn't be used for things like store object)
        if (visualize_)
        {
            visualize_bb(i, new_bin_pose.pose);
        }
    }

    if (bin_detected_ && req.attach_collision_object)
    {
        std_srvs::Empty empty_srv;
        if (!detach_base_client_.call(empty_srv))
        {
            ROS_ERROR("Couldn't call detach from base collision service!  Continuing execution anyway...");
        }

        manipulation_actions::AttachToBase attach_srv;
        attach_srv.request.segmented_object = attach_object;
        if (!attach_base_client_.call(attach_srv))
        {
            ROS_ERROR("Couldn't call attach to base collision service!  Continuing execution anyway...");
        }
    }

    // small angle adjustments to align bin pose with gravity vector
    if (bin_detected_)
    {
        tf2::Quaternion bin_rot(best_bin_transform_.transform.rotation.x, best_bin_transform_.transform.rotation.y,
                                best_bin_transform_.transform.rotation.z, best_bin_transform_.transform.rotation.w);
        tf2::Matrix3x3 bin_rot_m(bin_rot);
        double roll, pitch, yaw;
        bin_rot_m.getRPY(roll, pitch, yaw);
        tf2::Quaternion corrected_bin_rot;
        corrected_bin_rot.setRPY(0, 0, yaw);
        best_bin_transform_.transform.rotation.x = corrected_bin_rot.x();
        best_bin_transform_.transform.rotation.y = corrected_bin_rot.y();
        best_bin_transform_.transform.rotation.z = corrected_bin_rot.z();
        best_bin_transform_.transform.rotation.w = corrected_bin_rot.w();

        best_bin_transform_;
    }


    res.bin_poses = bin_poses;
    std::cout << "run duration:  " << ros::Time::now()-begin << std::endl;
    return true;
}

bool BinDetector::get_bin_pose(ApproxMVBB::OOBB& bb, pcl::PointCloud<pcl::PointXYZRGB>& cloud, geometry_msgs::Pose& bin_pose) {
    // gets bin translation from bounding box
    ApproxMVBB::Vector3 bin_position = get_box_top_in_world(bb);

    // gets absolute bin orientation
    ApproxMVBB::Quaternion bin_orientation = bb.m_q_KI;
    ApproxMVBB::Quaternion adjust_orientation = ApproxMVBB::Quaternion(1.0,0,0,0);

    // aligns y-axis to bin handle
    float handle_slope = get_handle_slope_from_cloud(bb,cloud);
    if ( slopeAlignedToXAxis(handle_slope,bb,cloud) ) {
        adjust_orientation = ApproxMVBB::Quaternion(0.7071068,0,0,0.7071068); // 90 degree rot about z
        bin_orientation = bin_orientation * adjust_orientation;
    }

    // aligns x-axis to bin second wall
    int response = secondWallAlignedToXAxis(adjust_orientation,bb,cloud);
    if ( response == 0 ) {
        adjust_orientation = ApproxMVBB::Quaternion(0,0,0,1); // 180 degree rot about z
        bin_orientation = bin_orientation * adjust_orientation;
    } else if (response < 0) {
        return false;
    }

    // loads return variable
    bin_pose.position.x = bin_position.x();
    bin_pose.position.y = bin_position.y();
    bin_pose.position.z = bin_position.z();
    bin_pose.orientation.x = bin_orientation.x();
    bin_pose.orientation.y = bin_orientation.y();
    bin_pose.orientation.z = bin_orientation.z();
    bin_pose.orientation.w = bin_orientation.w();
    // success signal
    return true;
}

float BinDetector::get_handle_slope_from_cloud(ApproxMVBB::OOBB& bb, pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
    std::vector<double> handle_points_x;
    std::vector<double> handle_points_y;

    // approximates expected handle height
    ApproxMVBB::Vector3 bin_top = bb.m_q_KI * bb.m_maxPoint;
    float min_handle_height = bin_top.z()-0.015;

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

int BinDetector::secondWallAlignedToXAxis(ApproxMVBB::Quaternion adjust_orientation, ApproxMVBB::OOBB& bb, pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
    int aligned;
    // postion of second wall from origin centered at bin
    ApproxMVBB::Vector3 swc_bin1 = ApproxMVBB::Vector3(0.0635,0,-0.051);
    ApproxMVBB::Vector3 swc_bin2 = ApproxMVBB::Vector3(-0.0635,0,-0.051);

    // rotates if needed
    swc_bin1 = adjust_orientation * swc_bin1;
    swc_bin2 = adjust_orientation * swc_bin2;

    // puts in segmentation frame
    ApproxMVBB::Vector3 swc_base_link1 = bb.m_q_KI * (get_box_top_in_bin(bb) + swc_bin1);
    ApproxMVBB::Vector3 swc_base_link2 = bb.m_q_KI * (get_box_top_in_bin(bb) + swc_bin2);

    // counts number of points matching expected model (inliers)
    int num_inliers1 = 0;
    int num_inliers2 = 0;
    double distance;

    // point clouds of supposed 2nd position
    /*pcl::PointCloud<pcl::PointXYZRGB> handle_cloud; // TODO DEBUG
    sensor_msgs::PointCloud2 handle_msg; // TODO DEBUG
    pcl::PointXYZRGB temp_point = cloud[0]; // TODO DEBUG
    temp_point.x = swc_base_link1.x(); // TODO DEBUG
    temp_point.y = swc_base_link1.y(); // TODO DEBUG
    temp_point.z = swc_base_link1.z(); // TODO DEBUG
    handle_cloud.push_back(temp_point); // TODO DEBUG
    temp_point.x = swc_base_link2.x(); // TODO DEBUG
    temp_point.y = swc_base_link2.y(); // TODO DEBUG
    temp_point.z = swc_base_link2.z(); // TODO DEBUG
    handle_cloud.push_back(temp_point); // TODO DEBUG
    pcl::toROSMsg(handle_cloud,handle_msg); // TODO DEBUG
    handle_msg.header.frame_id = "base_link"; // TODO DEBUG
    pub2_.publish(handle_msg); // TODO DEBUG*/


    for (int j=0;j<cloud.size();j++) {
        distance = sqrt(pow(swc_base_link1.x()-cloud[j].x,2)+pow(swc_base_link1.y()-cloud[j].y,2)+pow(swc_base_link1.z()-cloud[j].z,2));
        if (distance <= 0.0254) {
            num_inliers1 += 1;
        }
        distance = sqrt(pow(swc_base_link2.x()-cloud[j].x,2)+pow(swc_base_link2.y()-cloud[j].y,2)+pow(swc_base_link2.z()-cloud[j].z,2));
        if (distance <= 0.0254) {
            num_inliers2 += 1;
        }
    }
    if ( num_inliers1 < num_inliers2 ) {
        ROS_INFO("second wall misaligned, rotate by 180 about z");
        aligned = 0;
    } else if ( num_inliers1 > num_inliers2 ) {
        ROS_INFO("second wall aligned with x axis, no extra rotation applied");
        aligned = 1;
    } else {
        ROS_INFO("occlusion of second wall, new bin detection suggested");
        aligned = -1;
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

void BinDetector::invertZAxis(ApproxMVBB::OOBB& bb) {
    // Make new -z-Axis the z-Axis
    // R_NK = Rotate around 90∞ around Y, and 90∞ around X (always in K frame)
    bb.m_q_KI = bb.m_q_KI * ApproxMVBB::Quaternion(0.5 ,0.5, 0.5, 0.5);
    // R_NK = Rotate 90∞ around X (always in K frame)
    bb.m_q_KI = bb.m_q_KI * ApproxMVBB::Quaternion(0.7071068,0.7071068,0,0);
    // Change points  Coordinates I_[x,y,z] -> K_[y,x,-z]
    // swap x and y
    std::swap(bb.m_minPoint(0),bb.m_minPoint(1));
    std::swap(bb.m_maxPoint(0),bb.m_maxPoint(1));
    // makes z = -z, also swaps max and min z
    bb.m_minPoint(2) = -bb.m_minPoint(2);
    bb.m_maxPoint(2) = -bb.m_maxPoint(2);
    std::swap(bb.m_minPoint(2),bb.m_maxPoint(2));
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

ApproxMVBB::Vector3 BinDetector::get_box_top_in_world(ApproxMVBB::OOBB& bb) {
    ApproxMVBB::Vector3 box_center = get_box_center(bb);
    box_center.z() = bb.m_maxPoint.z();
    ApproxMVBB::Vector3 box_pose = bb.m_q_KI * box_center;
    return box_pose;
}

ApproxMVBB::Vector3 BinDetector::get_box_top_in_bin(ApproxMVBB::OOBB& bb) {
    ApproxMVBB::Vector3 box_pose = get_box_center(bb);
    box_pose.z() = bb.m_maxPoint.z();
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
    else
    {
        // Broadcast the base bin frame until we have a detection to keep
        // MoveIt! happy
        base_right_bin_transform_.header.stamp = ros::Time::now();
        tf_broadcaster_.sendTransform(base_right_bin_transform_);
    }

}
