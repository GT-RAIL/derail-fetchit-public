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
#include <fetchit_mapping/NavigationAction.h>

using namespace std;

class NavigationActionServer
{
protected:
	// Action server functionality
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<fetchit_mapping::NavigationAction> as_;
    std::string action_name_;
    fetchit_mapping::NavigationFeedback feedback_;
    fetchit_mapping::NavigationResult result_;

    // Velocity publisher
    ros::Publisher fetch_vel;
    tf::TransformListener base_tf_listener;

    // Tolerances for the controllers - controller loop stops when error is less than 
    double p_tolerance = 0.06;
    double w_tolerance = 0.1;
    geometry_msgs::PoseStamped goal, world_goal;
    bool goal_reached = true;

    // PID control gains - angular velocity controller
    double k_p = 1.7;
    double k_i = 0.008;
    double k_d = 0.0007;

    // we maintain constant linear velocity
    double linear_vel = 0.3;
    double angular_vel;

    // stop the robot turning around - condition for abort
    const double loop_terminate_thresh = 0.75*M_PI;

    bool test_nav_param;
    string logfile_path;

    void logData(geometry_msgs::PoseStamped goal, double theta_error, double duration, string logfile_path)
    {
        std::ofstream logfile;
        logfile.open(logfile_path+"testing-nav-final.csv", std::ofstream::out | std::ofstream::app);
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
    NavigationActionServer(std::string name) :
        as_(nh_, name, boost::bind(&NavigationActionServer::executeCB, this, _1), false),
        action_name_(name)
    {
        nh_.getParam("test_nav", test_nav_param);
        nh_.getParam("logfile_path", logfile_path);

        nh_.getParam("/navigation_server/k_pw", k_p);
        nh_.getParam("/navigation_server/k_iw", k_i);
        nh_.getParam("/navigation_server/k_dw", k_d);

        std::cout << "k_pw" << k_p << std::endl;
        std::cout << "k_iw" << k_i << std::endl;
        std::cout << "k_dw" << k_d << std::endl;

        fetch_vel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        as_.start();
    }

    ~NavigationActionServer(void)
    {}

    void executeCB(const fetchit_mapping::NavigationGoalConstPtr &request_goal)
    {
        ros::Rate rate(50.0);
        bool success = true;
        world_goal = request_goal->goal;
        feedback_.status = 0;

        double nav_start_time = ros::Time::now().toSec();
        goal = world_goal;
        base_tf_listener.transformPose("base_link", ros::Time(0), world_goal, world_goal.header.frame_id, goal); // latest common time
        geometry_msgs::Twist vel;   // velocity message to be published

        double dist_to_goal = sqrt(goal.pose.position.x*goal.pose.position.x + goal.pose.position.y*goal.pose.position.y);

        //variables for PID controller 
        double last_error= 0; 
        double sum_error = 0; // integration of error over time
        double prev_time = ros::Time::now().toSec();
        double delta_time;
        double error_now = atan2(goal.pose.position.y, goal.pose.position.x); // current error

        if(dist_to_goal > 0.1)
        {
            feedback_.status = 1;
            /* Rotate at the same point until heading aligns with the goal point */

            ROS_INFO("Aligning with the goal point %f", error_now*180/M_PI);
            while(abs(error_now) > w_tolerance && sum_error < loop_terminate_thresh)
            {
                if(as_.isPreemptRequested() || !ros::ok())
                {
                    ROS_INFO("%s: Preempted", action_name_.c_str());
                    as_.setPreempted();
                    success = false;
                    break;
                }
                feedback_.status = 2;

                delta_time = ros::Time::now().toSec() - prev_time;
                sum_error += error_now*delta_time;
                //PID control calculation
                angular_vel = k_p*error_now + k_i*sum_error + k_d*(error_now-last_error)/delta_time;

                vel.linear.x = 0;
                vel.angular.z = angular_vel;
                //cout<<"velocity "<<vel.linear.x<<" "<<vel.angular.z<<endl;

                prev_time = ros::Time::now().toSec();
                last_error = error_now;

                fetch_vel.publish(vel);
                as_.publishFeedback(feedback_);
                rate.sleep();
                ros::spinOnce();
                base_tf_listener.transformPose("base_link", ros::Time(0), world_goal, world_goal.header.frame_id, goal);

                //cout<<"transformed coordinate "<<goal.pose.position.x<<" "<<goal.pose.position.y<<endl;
                error_now = atan2(goal.pose.position.y, goal.pose.position.x);
                ROS_INFO("Error integral %f", sum_error*180/M_PI);
            }
            if(sum_error < loop_terminate_thresh)
            {
            	result_.ack = -1;
            	as_.setAborted(result_);
            	return;
            }

            ROS_INFO("Alignment complete");

            vel.angular.z = 0;
            fetch_vel.publish(vel);

            /* move towards the goal point controlling both linear and angular velocity */
            ROS_INFO("Moving to goal location x: %f y: %f", goal.pose.position.x, goal.pose.position.y);

            sum_error = 0; // clearing the error integral

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

                feedback_.status = 3; // not being used
                // here we transform the goal to the frame of base_link. Once this has been done, the error is 
                // simply the angle of the goal point in this frame. 

                error_now = atan2(goal.pose.position.y, goal.pose.position.x); 

                delta_time = ros::Time::now().toSec() - prev_time;
                sum_error += error_now*delta_time;

                angular_vel = k_p*error_now + k_i*sum_error + k_d*(error_now - last_error)/delta_time;
                last_error = error_now;
                prev_time = ros::Time::now().toSec();

                vel.linear.x = linear_vel;
                vel.angular.z = angular_vel;
                fetch_vel.publish(vel);
                as_.publishFeedback(feedback_);

                ROS_INFO("Error integral %f", sum_error*180/M_PI);

                ROS_DEBUG_STREAM("velocity "<<vel.linear.x<<" "<<vel.angular.z);
                ros::spinOnce();
                base_tf_listener.transformPose("base_link", ros::Time(0), world_goal, world_goal.header.frame_id, goal);
                rate.sleep();
            }
        }
        ROS_INFO("Reached goal point");
        vel.angular.z = 0;
        vel.linear.x = 0;
        fetch_vel.publish(vel);

        // Align to the pose at the goal

        tf::Quaternion quat;
        tf::quaternionMsgToTF(goal.pose.orientation, quat);
        double roll, pitch;
        tf::Matrix3x3(quat).getRPY(roll, pitch, error_now);
        ROS_INFO("Aligning to the pose angle : %f", error_now*180/M_PI);
        //last_error = 0; //ignore differential for first iteration

        if(abs(error_now) > 0.2)
        {
            while(abs(error_now) > w_tolerance && sum_error < loop_terminate_thresh)
            {
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
                sum_error += error_now*delta_time;
                angular_vel = k_p*error_now + k_i*sum_error + k_d*(error_now - last_error)/delta_time;

                vel.linear.x = 0;
                vel.angular.z = angular_vel;
                //cout<<"velocity "<<vel.linear.x<<" "<<vel.angular.z<<endl;

                prev_time = ros::Time::now().toSec();
                last_error = error_now;

                fetch_vel.publish(vel);
                as_.publishFeedback(feedback_);
                rate.sleep();
                ros::spinOnce();
                goal = world_goal;
                base_tf_listener.transformPose("base_link", ros::Time(0), world_goal, world_goal.header.frame_id, goal);
                //cout<<"transformed coordinate "<<goal.pose.position.x<<" "<<goal.pose.position.y<<endl;
                tf::quaternionMsgToTF(goal.pose.orientation, quat);
                tf::Matrix3x3(quat).getRPY(roll, pitch, error_now);
                ROS_DEBUG("current alignment error %f", error_now*180/M_PI);
            }
            if(sum_error < loop_terminate_thresh)
            {
            	result_.ack = -1;
            	as_.setAborted(result_);
            	return;
            }
        }
        if(success)
        {
            ROS_INFO("waypoint reached");
            result_.ack = 1;
            as_.setSucceeded(result_);
        }

        if(test_nav_param)
        {
            logData(goal, error_now, ros::Time::now().toSec() - nav_start_time, logfile_path);
        }
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation");
    NavigationActionServer navigation("navigation");
    ros::spin();

    return 0;
}

