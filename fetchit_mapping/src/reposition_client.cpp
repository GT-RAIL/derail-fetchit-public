#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <fetchit_mapping/RepositionAction.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define PI 3.14159

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_reposition");
  tf2::Quaternion q;
  double goalX, goalY, goalTheta;
  geometry_msgs::PoseStamped msg;

  actionlib::SimpleActionClient<fetchit_mapping::RepositionAction> ac("reposition", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  fetchit_mapping::RepositionGoal goal;

  ros::Rate loop_rate(100);

  msg.header.frame_id = "map";
  msg.pose.position.z = 0; 
  
  while(ros::ok())
  {
    std::cout<<"Enter goal x, y and theta(degrees) : ";
    std::cin>>goalX>>goalY>>goalTheta;

    q.setRPY(0,0,PI*goalTheta/180.0); 
    q.normalize();
    tf2::convert(q, msg.pose.orientation);

    msg.pose.position.x = goalX; 
    msg.pose.position.y = goalY; 

    goal.goal = msg;
    ac.sendGoal(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
      ROS_INFO("Action did not finish before the time out.");
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  //exit
  return 0;
}

