#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#define PI 3.14159

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "goal_publisher");
  ros::NodeHandle nh;
  tf2::Quaternion q;
  double goalX, goalY, goalTheta;

  ros::Publisher goal_pub;
  geometry_msgs::PoseStamped msg;
  ros::Rate loop_rate(100);

  msg.header.frame_id = "map";
  msg.pose.position.z = 0; 
  goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal_waypoint", 100);
  
  while (ros::ok()) 
  {
    std::cout<<"Enter goal x, y and theta(degrees) : ";
    std::cin>>goalX>>goalY>>goalTheta;

    q.setRPY(0,0,PI*goalTheta/180.0); 
    q.normalize();
    tf2::convert(q, msg.pose.orientation);

    msg.pose.position.x = goalX; 
    msg.pose.position.y = goalY; 
    goal_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}