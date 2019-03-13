#include <math.h>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>


using namespace std;

namespace fetchit_mapping {

    class PointFilter2D {
        public:
            PointFilter2D(ros::NodeHandle& nh, string point_topic, string point_frame, string map_frame, string odom_frame, float radius);

            void filter_points_(const boost::shared_ptr<const sensor_msgs::LaserScan>& input_scan);

            bool not_in_radius(float x, float y);

        protected:
            string point_topic_;
            string point_frame_;
            string map_frame_;
            string odom_frame_;
            float radius_;

            tf::TransformListener tf_req_;
            ros::NodeHandle filter_nh_;
            ros::Publisher  filtered_points_pub_;
            laser_geometry::LaserProjection projector_;

            message_filters::Subscriber<sensor_msgs::LaserScan> raw_points_sub_;
            tf::MessageFilter<sensor_msgs::LaserScan> * tf_filter_;
    };

}