#include "fetchit_icp/TemplateMatching.h"

TemplateMatcher::TemplateMatcher(ros::NodeHandle& nh, std::string& matching_frame, std::string& pcl_topic,
                                 std::string& template_file, tf::Transform& initial_estimate,
                                 tf::Transform& template_offset, std::string& template_frame, bool& visualize) {
    matcher_nh_ = nh;
    matching_frame_ = matching_frame;
    pcl_topic_ = pcl_topic;
    initial_estimate_ = initial_estimate;
    template_offset_ = template_offset;
    template_frame_ = template_frame;
    viz_ = visualize;

    // gets template pcd file
    std::string templates_path = ros::package::getPath("fetchit_icp")+"/cad_models/";
    std::string template_filepath = templates_path+template_file;

    // loads template cloud
    template_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(template_filepath,*template_cloud_) < 0) {
        ROS_ERROR("Could not load template PCD.");
        exit(-1);
    }

    // prepares point cloud for matching by transforming by initial_estimate
    pcl_ros::transformPointCloud(*template_cloud_,*template_cloud_,initial_estimate_);

    // creates service client to request ICP matches
    icp_client_ = matcher_nh_.serviceClient<fetchit_icp::ICPMatch>("/icp_match_clouds");

    // visualization publishers
    pub_temp_ = matcher_nh_.advertise<sensor_msgs::PointCloud2>("template_points",0);
    pub_targ_ = matcher_nh_.advertise<sensor_msgs::PointCloud2>("target",0);
    pub_mtemp_ = matcher_nh_.advertise<sensor_msgs::PointCloud2>("matched_template_points",0);

    // creates service handler for template matching
    pose_srv_ = matcher_nh_.advertiseService("match_template", &TemplateMatcher::handle_match_template, this);
}

bool TemplateMatcher::handle_match_template(fetchit_icp::TemplateMatch::Request& req, fetchit_icp::TemplateMatch::Response& res) {
    // declare data structures
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr matched_template_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // gets current point cloud from pcl_topic_
    boost::shared_ptr<sensor_msgs::PointCloud2 const> sharedMsg;
    sensor_msgs::PointCloud2 target_cloud_msg;
    sharedMsg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pcl_topic_);
    if(sharedMsg != NULL){
        target_cloud_msg = *sharedMsg;
    } else {
        ROS_ERROR("Could not get point cloud message from topic. Boost shared pointer is NULL.");
        return false;
    }
    pcl::fromROSMsg(target_cloud_msg,*target_cloud);

    // transforms point cloud to the matching frame
    //if (matching_frame_ != target_cloud_msg.header.frame_id) {
    pcl_ros::transformPointCloud(matching_frame_, ros::Time(0), *target_cloud, target_cloud_msg.header.frame_id,
                                     *target_cloud, tf_);
    //}

    // prepares sensor_msgs to make ICP request
    sensor_msgs::PointCloud2 template_msg;
    sensor_msgs::PointCloud2 target_msg;
    pcl::toROSMsg(*template_cloud_,template_msg);
    pcl::toROSMsg(*target_cloud,target_msg);

    // visualizes the transformed point cloud and estimated template pose
    if (viz_) {
        template_msg.header.frame_id = matching_frame_;
        target_msg.header.frame_id = matching_frame_;
        pub_temp_.publish(template_msg);
        pub_targ_.publish(target_msg);
    }

    // makes ICP request
    fetchit_icp::ICPMatch icp_srv;
    icp_srv.request.template_cloud = template_msg;
    icp_srv.request.target_cloud = target_msg;
    if (!icp_client_.call(icp_srv)) {
        ROS_ERROR("Failed to call ICP service.");
        return false;
    }

    // gets service results
    pcl::fromROSMsg(icp_srv.response.matched_template_cloud,*matched_template_cloud);
    tf::Transform icp_refinement;
    icp_refinement.setOrigin(tf::Vector3(icp_srv.response.match_tf.translation.x,
                                         icp_srv.response.match_tf.translation.y,
                                         icp_srv.response.match_tf.translation.z));
    icp_refinement.setRotation(tf::Quaternion(icp_srv.response.match_tf.rotation.x,
                                              icp_srv.response.match_tf.rotation.y,
                                              icp_srv.response.match_tf.rotation.z,
                                              icp_srv.response.match_tf.rotation.w));

    // calculates the final estimated tf in the matching frame
    tf::Transform tf_final = icp_refinement * initial_estimate_ * template_offset_;

    // prepares the service response
    tf::Vector3 final_trans = tf_final.getOrigin();
    tf::Quaternion final_rot = tf_final.getRotation();
    geometry_msgs::TransformStamped final_pose_stamped;
    final_pose_stamped.header.stamp = ros::Time::now();
    final_pose_stamped.header.frame_id = matching_frame_;
    final_pose_stamped.child_frame_id = template_frame_;
    final_pose_stamped.transform.translation.x = final_trans.x();
    final_pose_stamped.transform.translation.y = final_trans.y();
    final_pose_stamped.transform.translation.z = final_trans.z();
    final_pose_stamped.transform.rotation.x = final_rot.x();
    final_pose_stamped.transform.rotation.y = final_rot.y();
    final_pose_stamped.transform.rotation.z = final_rot.z();
    final_pose_stamped.transform.rotation.w = final_rot.w();

    // visualizes the matched point cloud and final estimated pose
    if (viz_) {
        static tf2_ros::StaticTransformBroadcaster static_broadcaster;
        static_broadcaster.sendTransform(final_pose_stamped);
        sensor_msgs::PointCloud2 matched_template_msg = icp_srv.response.matched_template_cloud;
        matched_template_msg.header.frame_id = matching_frame_;
        pub_mtemp_.publish(matched_template_msg);
    }

    res.template_pose = final_pose_stamped;
    return true;
}