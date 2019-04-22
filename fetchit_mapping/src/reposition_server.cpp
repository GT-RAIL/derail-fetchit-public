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

using namespace std;

class RepositionAction
{
protected: 
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<fetchit_mapping::RepositionAction> as_;
	std::string action_name_;
	fetchit_mapping::RepositionFeedback feedback_;
	fetchit_mapping::RepositionResult result_;
	ros::Publisher fetch_vel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	tf::TransformListener base_tf_listener;
	double p_tolerance = 0.06;
	double w_tolerance = 0.1;
	geometry_msgs::PoseStamped goal, world_goal;
	bool goal_reached = true;

	const double k_p = 3.0;
	const double k_i = 0.01;
	const double k_d = 0.08;

	const double k_pw = 1.5;
	const double k_iw = 0.15;
	const double k_dw = 0.03;

	double linear_vel;
	double angular_vel;

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
	RepositionAction(std::string name) :
		as_(nh_, name, boost::bind(&RepositionAction::executeCB, this, _1), false), 
		action_name_(name)
	{
		as_.start();
		nh_.getParam("test_nav", test_nav_param);
		nh_.getParam("logfile_path", logfile_path);
	}

	~RepositionAction(void)
	{}

	void executeCB(const fetchit_mapping::RepositionGoalConstPtr &request_goal)
	{
		ros::Rate rate(50.0);
		bool success = true;
		world_goal = request_goal->goal;
		feedback_.status = 0;

		double nav_start_time = ros::Time::now().toSec();
		goal = world_goal;
		base_tf_listener.transformPose("base_link", world_goal, goal);
		geometry_msgs::Twist vel;	// velocity message to be published

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
			while(abs(error_now_w) > 2*w_tolerance)
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
				sum_error_w += error_now_w*delta_time;
				//PID control calculation
				angular_vel = k_pw*error_now_w + k_iw*sum_error_w + k_dw*(error_now_w - last_error_w)/delta_time;

				vel.linear.x = 0;
				vel.angular.z = angular_vel;
				//cout<<"velocity "<<vel.linear.x<<" "<<vel.angular.z<<endl;

				prev_time = ros::Time::now().toSec();
				last_error_w = error_now_w;

				fetch_vel.publish(vel);
      			as_.publishFeedback(feedback_);

				rate.sleep();
				ros::spinOnce();
				base_tf_listener.transformPose("base_link", world_goal, goal);
				
				//cout<<"transformed coordinate "<<goal.pose.position.x<<" "<<goal.pose.position.y<<endl;
				error_now_w = signOf(goal.pose.position.x)*goal.pose.position.y/fabs(goal.pose.position.x);
				ROS_INFO("error : %f", error_now_w*180/M_PI);
			}
			ROS_INFO("Alignment complete");

			vel.angular.z = 0;
			fetch_vel.publish(vel);

			/* move towards the goal point controlling both linear and angular velocity */
			ROS_INFO("Moving to goal location x: %f y: %f", goal.pose.position.x, goal.pose.position.y);

			sum_error_w = 0;

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
                
                cout<<"velocity "<<vel.linear.x<<" "<<vel.angular.z<<endl;
                ros::spinOnce();
                base_tf_listener.transformPose("base_link", world_goal, goal);
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
	    tf::Matrix3x3(quat).getRPY(roll, pitch, error_now_w);
	    ROS_INFO("Aligning to the pose angle : %f", error_now_w*180/M_PI);
		//last_error = 0; //ignore differential for first iteration

        if(abs(error_now_w) > 0.2)
        {
            while(abs(error_now_w) > w_tolerance)
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
                sum_error_w += error_now_w*delta_time;
                angular_vel = k_pw*error_now_w + k_iw*sum_error_w + k_d*(error_now_w - last_error_w)/delta_time;

                vel.linear.x = 0;
                vel.angular.z = angular_vel;
                //cout<<"velocity "<<vel.linear.x<<" "<<vel.angular.z<<endl;

                prev_time = ros::Time::now().toSec();
                last_error_w = error_now_w;

                fetch_vel.publish(vel);
				as_.publishFeedback(feedback_);
                rate.sleep();
                ros::spinOnce();
                goal = world_goal;
                base_tf_listener.transformPose("base_link", world_goal, goal);
                //cout<<"transformed coordinate "<<goal.pose.position.x<<" "<<goal.pose.position.y<<endl;
                tf::quaternionMsgToTF(goal.pose.orientation, quat);
                tf::Matrix3x3(quat).getRPY(roll, pitch, error_now_w);
                ROS_INFO("current alignment error %f", error_now_w*180/M_PI);
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
			logData(goal, error_now_w, ros::Time::now().toSec() - nav_start_time, logfile_path);
		}
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "reposition");
	RepositionAction reposition("reposition");
	ros::spin();

	return 0;
}