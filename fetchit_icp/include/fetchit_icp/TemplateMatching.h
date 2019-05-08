#include <ros/package.h>
#include <ros/time.h>
#include <pcl/io/ply_io.h>

#include <geometry_msgs/Transform.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "fetchit_icp/ICPMatching.h"
#include "fetchit_icp/TemplateMatch.h"

class TemplateMatcher {
    public:
        TemplateMatcher(ros::NodeHandle& nh, std::string& matching_frame, std::string& pcl_topic,
                        std::string& template_file, tf::Transform& initial_estimate,
                        tf::Transform& template_offset, std::string& template_frame, bool& visualize);

        // handles requests to match a template CAD model (in PCD form) to a point cloud from a point cloud topic
        bool handle_match_template(fetchit_icp::TemplateMatch::Request& req, fetchit_icp::TemplateMatch::Response& res);

    protected:
        ros::NodeHandle matcher_nh_;
        std::string matching_frame_;
        std::string template_frame_;
        std::string pcl_topic_;
        tf::TransformListener tf_;
        tf::Transform initial_estimate_;
        tf::Transform template_offset_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud_;
        ros::ServiceClient icp_client_;
        ros::ServiceServer pose_srv_;
        bool viz_;
        ros::Publisher pub_temp_;
        ros::Publisher pub_targ_;
        ros::Publisher pub_mtemp_;
};