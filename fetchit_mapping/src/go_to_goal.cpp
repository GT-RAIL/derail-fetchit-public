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

#define PI 3.14159

using namespace std;

double goalX = 10;
double goalY = 10;
double robotX = 0;
double robotY = 0;
double robotZ = 0;
double robotHeadingX = 0;
double robotHeadingY = 0;
double currentX = 0;

double p_tolerance = 0.05;
double w_tolerance = 10*PI/180;

geometry_msgs::PoseStamped goal, world_goal;

bool goal_reached = true;

int fetch_index = -1;


/* ground truth pose of robot */
void truthCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	// the first time this is executed, find the index of fetch among all 
	// objects in gazebo.
	if(fetch_index == -1)
	{
		string name = "fetch";
		for(int i = 0;;i++)
		{
			if(msg->name[i] == name)
			{
				fetch_index = i;
				break;
			}
		}
	}
	robotX = msg->pose[fetch_index].position.x;
	robotY = msg->pose[fetch_index].position.y;
}


void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	world_goal = *msg;
	goal_reached = false;
	//std::cout<<"new messge "<<world_goal.pose.position.x<<" "<<world_goal.pose.position.y;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "fetch_navig");
	ros::NodeHandle nh;

	//subscriber to true pose of fetch from gazebo. use if theres no localization set up
	//ros::Subscriber fetchtruth_sub = nh.subscribe("/gazebo/model_states", 1000, truthCallback);

	//subscriber to goal point 
	ros::Subscriber goal_sub = nh.subscribe("/goal_waypoint", 1000, goalCallback);

	ros::Publisher fetch_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Rate rate(50.);
	tf::TransformListener listener;
    listener.waitForTransform("base_link", "map", ros::Time(0), ros::Duration(10.0) );

    const double k_p = 2.0;
    const double k_i = 0.01;
    const double k_d = 0.001;

    const double linear_vel = 0.3;
    double angular_vel;

    double last_error= 0;
    double sum_error = 0;
    double prev_time = ros::Time::now().toSec();
    double delta_time;
    double error_now;

	while(nh.ok())
	{
		ros::spinOnce();
		//if a new goal point has been set
		if(!goal_reached)
		{
			goal = world_goal;
			listener.transformPose("base_link", world_goal, goal);

			//cout<<"transformed coordinate "<<goal.pose.position.x<<" "<<goal.pose.position.y<<endl;
			geometry_msgs::Twist vel;	// velocity message to be published

			/* Rotate at the same point until heading aligns with the goal point */
			//variables for PID controller
			double last_error= 0;
		    double sum_error = 0;
    		double prev_time = ros::Time::now().toSec();
    		double delta_time;
			double error_now = atan2(goal.pose.position.y, goal.pose.position.x);
			ROS_INFO("Aligning with the goal point");
			last_error = error_now; //ignore differential for first iteration

			while(abs(error_now) > w_tolerance)
			{
				delta_time = ros::Time::now().toSec() - prev_time;
				sum_error += error_now*delta_time;
				//PID control calculation
				angular_vel = k_p*error_now + k_i*sum_error + k_d*(error_now - last_error)/delta_time;

				vel.linear.x = 0;
				vel.angular.z = angular_vel;
				//cout<<"velocity "<<vel.linear.x<<" "<<vel.angular.z<<endl;

				prev_time = ros::Time::now().toSec();
				last_error = error_now;

				fetch_vel.publish(vel); 
				rate.sleep();
				ros::spinOnce();

				goal = world_goal;
				listener.transformPose("base_link", world_goal, goal);
				//cout<<"transformed coordinate "<<goal.pose.position.x<<" "<<goal.pose.position.y<<endl;
				error_now = atan2(goal.pose.position.y, goal.pose.position.x);			
			}
			ROS_INFO("Alignment complete");

			vel.angular.z = 0;
			fetch_vel.publish(vel);

			/* move towards the goal point with constant linear velocity, controlling angular velocity */
			ROS_INFO("Moving to goal location x: %f y: %f", goal.pose.position.x, goal.pose.position.y);

			sum_error = 0;
			while(fabs(goal.pose.position.x) >= p_tolerance || fabs(goal.pose.position.y) >= p_tolerance)
			{
				error_now = atan2(goal.pose.position.y, goal.pose.position.x);
				delta_time = ros::Time::now().toSec() - prev_time;
				sum_error += error_now*delta_time;

				angular_vel = k_p*error_now + k_i*sum_error + k_d*(error_now - last_error)/delta_time;
				last_error = error_now;
				prev_time = ros::Time::now().toSec();

				vel.linear.x = linear_vel;
				vel.angular.z = angular_vel;
				fetch_vel.publish(vel); 
				//cout<<"velocity "<<vel.linear.x<<" "<<vel.angular.z<<endl;
				ros::spinOnce();

				goal = world_goal;
				listener.transformPose("base_link", world_goal, goal);
				//cout<<"transformed coordinate "<<goal.pose.position.x<<" "<<goal.pose.position.y<<endl;
				rate.sleep();
			}
			ROS_INFO("Reached goal point");
			vel.angular.z = 0;
			fetch_vel.publish(vel);

			// Align to the pose at the goal

			tf::Quaternion quat;
			tf::quaternionMsgToTF(goal.pose.orientation, quat);
		    double roll, pitch;
		    tf::Matrix3x3(quat).getRPY(roll, pitch, error_now);
		    ROS_INFO("Aligning to the pose angle : %f", error_now*180/PI);
			last_error = error_now; //ignore differential for first iteration

			while(abs(error_now) > w_tolerance)
			{
				delta_time = ros::Time::now().toSec() - prev_time;
				sum_error += error_now*delta_time;
				angular_vel = k_p*error_now + k_i*sum_error + k_d*(error_now - last_error)/delta_time;

				vel.linear.x = 0;
				vel.angular.z = angular_vel;
				//cout<<"velocity "<<vel.linear.x<<" "<<vel.angular.z<<endl;

				prev_time = ros::Time::now().toSec();
				last_error = error_now;

				fetch_vel.publish(vel); 
				rate.sleep();
				ros::spinOnce();
				goal = world_goal;
				listener.transformPose("base_link", world_goal, goal);
				//cout<<"transformed coordinate "<<goal.pose.position.x<<" "<<goal.pose.position.y<<endl;
				tf::quaternionMsgToTF(goal.pose.orientation, quat);
			    tf::Matrix3x3(quat).getRPY(roll, pitch, error_now);
			    //ROS_INFO("pose angle is %f", error_now*180/PI);
			}
			goal_reached = true;
		}			
		rate.sleep();
	}
}

