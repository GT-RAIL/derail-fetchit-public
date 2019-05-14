#include "ros/ros.h"
#include "std_msgs/String.h"
#include "string.h"
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/ModelStates.h"
#include <cstdlib>
#include <cstdio>
#include <time.h>
#include <cmath>
#include <vector>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <actionlib/server/simple_action_server.h>
#include <fetchit_mapping/RepositionAction.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class RepositionActionServer
{
protected:
    // Action server functionality
    ros::NodeHandle nh_, pnh_;
    actionlib::SimpleActionServer<fetchit_mapping::RepositionAction> as_;
    std::string action_name_;
    fetchit_mapping::RepositionFeedback feedback_;
    fetchit_mapping::RepositionResult result_;

    // Odom monitor to avoid stalling/overturning issues
    const float ZERO_VEL_THRES = 0.015; // Less than this value signifies zero
    ros::Subscriber sub = nh_.subscribe("odom", 1, &RepositionActionServer::odom_callback, this);
    boost::mutex odom_mutex_; // mutex for odom callback
    nav_msgs::Odometry curr_odom_reading;
    double max_odom_rotation; // Rotations above this range are disallowed
    float total_odom_rotation_ = 0.0;
    float prev_odom_angle_ = 180.0;
    double stall_initial_time; // Initial time on timer for stall detection
    float stall_timer_;
    float prev_time_odom_ = 0.0;

    // Velocity publisher
    ros::Publisher fetch_vel;
    tf::TransformListener base_tf_listener;
    double linear_vel;
    double angular_vel;

    // Tolerances for the controllers
    double p_tolerance = 0.06;
    double w_tolerance = 0.1;
    geometry_msgs::PoseStamped goal, world_goal;
    bool goal_reached = true;

    // PID controller gains
    double k_p, k_i, k_d;    // linear vel gains
    double k_pw, k_iw, k_dw; // angular vel gains

    // Test parameters
    bool test_nav_param;
    string logfile_path;

    void logData(geometry_msgs::PoseStamped goal, double theta_error, double duration, string logfile_path)
    {
        std::ofstream logfile;
        logfile.open(logfile_path+"testing-reposition-final.csv", std::ofstream::out | std::ofstream::app);
        logfile<<goal.pose.position.x<<","<<goal.pose.position.y<<","<<theta_error<<","<<duration<<std::endl;
        logfile.close();
    }

    double signOf(double number)
    {
        if(number >= 0)
            return 1.0;
        else
            return -1.0;
    }

public:
    RepositionActionServer(std::string name) :
        as_(nh_, name, boost::bind(&RepositionActionServer::executeCB, this, _1), false),
        action_name_(name),
        pnh_("~")
    {
        pnh_.param<bool>("test_nav", test_nav_param, false);
        pnh_.param<std::string>("logfile_path", logfile_path, "");

        pnh_.param<double>("k_p", k_p, 1.0);
        pnh_.param<double>("k_i", k_i, 0.0);
        pnh_.param<double>("k_d", k_d, 0.0);
        pnh_.param<double>("k_pw", k_pw, 1.7);
        pnh_.param<double>("k_iw", k_iw, 0.008);
        pnh_.param<double>("k_dw", k_dw, 0.0007);

        pnh_.param<double>("max_odom_rotation", max_odom_rotation, 720.0);
        pnh_.param<double>("stall_initial_time", stall_initial_time, 15.0);
        stall_timer_ = stall_initial_time;

        fetch_vel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        as_.start();
    }

    ~RepositionActionServer(void)
    {}

    void executeCB(const fetchit_mapping::RepositionGoalConstPtr &request_goal)
    {
        ros::Rate rate(50.0);
        bool success = true;
        world_goal = request_goal->goal;
        feedback_.status = 0;

        double nav_start_time = ros::Time::now().toSec();
        goal = world_goal;
        base_tf_listener.transformPose("base_link", ros::Time(0), world_goal, world_goal.header.frame_id, goal);
        geometry_msgs::Twist vel;   // velocity message to be published

        double dist_to_goal = sqrt(goal.pose.position.x*goal.pose.position.x + goal.pose.position.y*goal.pose.position.y);

        //variables for PID controller
        double last_error_w= 0;
        double sum_error_w = 0;
        double prev_time = ros::Time::now().toSec();
        double delta_time;
        double error_now_w = signOf(goal.pose.position.x)*goal.pose.position.y/fabs(goal.pose.position.x);

        if(dist_to_goal > 0.1)
        {
            feedback_.status = 1;

            /* Rotate at the same point until heading aligns with the goal point */
            ROS_INFO("Aligning with the goal point %f", error_now_w*180/M_PI);
            reset_monitor_odom();
            while(abs(error_now_w) > 2*w_tolerance)
            {
                if (monitor_odom()) {
                    result_.ack = -1;
                    as_.setAborted(result_);
                    return;
                }

                if(as_.isPreemptRequested() || !ros::ok())
                {
                    ROS_INFO("%s: Preempted", action_name_.c_str());
                    as_.setPreempted();
                    success = false;
                    break;
                }
                feedback_.status = 2;

                delta_time = ros::Time::now().toSec() - prev_time;
                sum_error_w += error_now_w*delta_time;
                //PID control calculation
                angular_vel = k_pw*error_now_w + k_iw*sum_error_w + k_dw*(error_now_w - last_error_w)/delta_time;

                vel.linear.x = 0.0;
                vel.angular.z = angular_vel;
                ROS_DEBUG("X Linear Velocity: %f, Z Angular Velocity: %f", vel.linear.x, vel.angular.z);

                prev_time = ros::Time::now().toSec();
                last_error_w = error_now_w;

                fetch_vel.publish(vel);
                as_.publishFeedback(feedback_);

                rate.sleep();
                ros::spinOnce();
                base_tf_listener.transformPose("base_link", ros::Time(0), world_goal, world_goal.header.frame_id, goal);

                ROS_DEBUG("Transformed Coordinate: (%f, %f)", goal.pose.position.x, goal.pose.position.y);
                error_now_w = signOf(goal.pose.position.x)*goal.pose.position.y/fabs(goal.pose.position.x);
                ROS_DEBUG("error : %f", error_now_w*180/M_PI);
            }
            ROS_INFO("Alignment complete");

            vel.angular.z = 0.0;
            fetch_vel.publish(vel);

            /* Move towards the goal point controlling both linear and angular velocity */
            ROS_INFO("Moving to goal location x: %f y: %f", goal.pose.position.x, goal.pose.position.y);
            sum_error_w = 0; // clearing the error integral

            double last_error_v= 0;
            double sum_error_v = 0;
            double error_now_v = signOf(goal.pose.position.x)*goal.pose.position.y/fabs(goal.pose.position.x);

            while(fabs(goal.pose.position.x) >= p_tolerance || fabs(goal.pose.position.y) >= p_tolerance)
            {
                if(as_.isPreemptRequested() || !ros::ok() || (!success))
                {
                    ROS_INFO("%s: Preempted", action_name_.c_str());
                    if(success)
                        as_.setPreempted();
                    success = false;
                    break;
                }
                feedback_.status = 3;
                error_now_w = signOf(goal.pose.position.x)*goal.pose.position.y/fabs(goal.pose.position.x);
                error_now_v = goal.pose.position.x;
                delta_time = ros::Time::now().toSec() - prev_time;

                sum_error_w += error_now_w*delta_time;
                sum_error_v += error_now_v*delta_time;

                angular_vel = k_pw*error_now_w + k_iw*sum_error_w + k_dw*(error_now_w - last_error_w)/delta_time;
                last_error_w = error_now_w;

                linear_vel = k_p*error_now_v + k_i*sum_error_v + k_d*(error_now_v - last_error_v)/delta_time;
                last_error_v = error_now_v;

                prev_time = ros::Time::now().toSec();

                vel.linear.x = linear_vel;
                vel.angular.z = angular_vel;
                fetch_vel.publish(vel);
                as_.publishFeedback(feedback_);
                ROS_DEBUG("X Linear Velocity: %f, Z Angular Velocity: %f", vel.linear.x, vel.angular.z);

                ros::spinOnce();
                base_tf_listener.transformPose("base_link", ros::Time(0), world_goal, world_goal.header.frame_id, goal);
                rate.sleep();
            }
        }
        ROS_INFO("Reached goal point");
        vel.angular.z = 0.0;
        vel.linear.x = 0.0;
        fetch_vel.publish(vel);

        // Align to the pose at the goal
        tf::Quaternion quat;
        tf::quaternionMsgToTF(goal.pose.orientation, quat);
        double roll, pitch;
        tf::Matrix3x3(quat).getRPY(roll, pitch, error_now_w);
        ROS_INFO("Aligning to the pose angle : %f", error_now_w*180/M_PI);
        //last_error = 0; //ignore differential for first iteration

        if(abs(error_now_w) > 0.2)
        {
            /* Rotate at the same point until heading aligns with the final goal orientation */
            reset_monitor_odom();
            while(abs(error_now_w) > w_tolerance)
            {
                if (monitor_odom()) {
                    result_.ack = -1;
                    as_.setAborted(result_);
                    return;
                }

                if(as_.isPreemptRequested() || !ros::ok() || (!success))
                {
                    ROS_INFO("%s: Preempted", action_name_.c_str());
                    if(success)
                        as_.setPreempted();
                    success = false;
                    break;
                }
                feedback_.status = 4;
                delta_time = ros::Time::now().toSec() - prev_time;
                sum_error_w += error_now_w*delta_time;
                angular_vel = k_pw*error_now_w + k_iw*sum_error_w + k_d*(error_now_w - last_error_w)/delta_time;

                vel.linear.x = 0.0;
                vel.angular.z = angular_vel;
                ROS_DEBUG("X Linear Velocity: %f, Z Angular Velocity: %f", vel.linear.x, vel.angular.z);

                prev_time = ros::Time::now().toSec();
                last_error_w = error_now_w;

                fetch_vel.publish(vel);
                as_.publishFeedback(feedback_);
                rate.sleep();
                ros::spinOnce();

                goal = world_goal;
                base_tf_listener.transformPose("base_link", ros::Time(0), world_goal, world_goal.header.frame_id, goal);
                ROS_DEBUG("Transformed Coordinate: (%f, %f)", goal.pose.position.x, goal.pose.position.y);
                tf::quaternionMsgToTF(goal.pose.orientation, quat);
                tf::Matrix3x3(quat).getRPY(roll, pitch, error_now_w);
                ROS_DEBUG("current alignment error %f", error_now_w*180/M_PI);
            }
        }
        if(success)
        {
            ROS_INFO("Waypoint reached");
            result_.ack = 1;
            as_.setSucceeded(result_);
        }

        if(test_nav_param)
        {
            logData(goal, error_now_w, ros::Time::now().toSec() - nav_start_time, logfile_path);
        }
    }

    void odom_callback(nav_msgs::Odometry odom_reading) {
        boost::mutex::scoped_lock lock(odom_mutex_);
        curr_odom_reading = odom_reading;
    }

    bool monitor_odom() {
        boost::mutex::scoped_lock odom_lock(odom_mutex_);

        tf::Quaternion odom_Q;
        tf::quaternionMsgToTF(curr_odom_reading.pose.pose.orientation, odom_Q);
        double radians_roll, radians_pitch, radians_yaw;
        tf::Matrix3x3(odom_Q).getRPY(radians_roll, radians_pitch, radians_yaw);

        float curr_odom_angle_ = radians_yaw * 180.0 / M_PI + 180.0;
        ROS_DEBUG("Yaw odometry (Degrees): %f", curr_odom_angle_);

        float odom_change = curr_odom_angle_ - prev_odom_angle_;
        if (odom_change < -300.0) {
            odom_change += 360.0;
        } else if (odom_change > 300.0) {
            odom_change -= 360.0;
        }

        total_odom_rotation_ += odom_change;
        prev_odom_angle_ = curr_odom_angle_;
        ROS_DEBUG("Total rotation (Degrees): %f", total_odom_rotation_);

        // Over rotation check
        if (fabs(total_odom_rotation_) > max_odom_rotation) {
            ROS_INFO("ABORTED MOTION due to rotation greater than allowed range.");
            return true;
        }

        // Stall check
        if (fabs(odom_change) <= ZERO_VEL_THRES) {
            stall_timer_ -= ros::Time::now().toSec() - prev_time_odom_;

            if (stall_timer_ <= 0.0) {
                ROS_INFO("ABORTED MOTION due to stalling for more than allowed timer.");
                return true;
            }
        } else {
            stall_timer_ = stall_initial_time;
        }

        ROS_DEBUG("Stall Timer (Seconds): %f", stall_timer_);
        prev_time_odom_ = ros::Time::now().toSec();

        return false;
    }

    void reset_monitor_odom() {
        boost::mutex::scoped_lock odom_lock(odom_mutex_);

        tf::Quaternion odom_Q;
        tf::quaternionMsgToTF(curr_odom_reading.pose.pose.orientation, odom_Q);
        double radians_roll, radians_pitch, radians_yaw;
        tf::Matrix3x3(odom_Q).getRPY(radians_roll, radians_pitch, radians_yaw);

        total_odom_rotation_ = 0.0;
        stall_timer_ = stall_initial_time;
        prev_odom_angle_ = radians_yaw * 180.0 / M_PI + 180.0;
        prev_time_odom_ = ros::Time::now().toSec();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "reposition");
    RepositionActionServer reposition("reposition");
    ros::spin();

    return 0;
}
