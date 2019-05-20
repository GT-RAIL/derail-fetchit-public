#include <math.h>
#include <stdlib.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

//#include "fetchit_mapping/map_savers.h"

#define ONE_DEGREE 0.01745329251

control_msgs::FollowJointTrajectoryGoal look_goal(float tilt_angle, float sspeed) {
    control_msgs::FollowJointTrajectoryGoal head_goal;
    head_goal.trajectory.joint_names.push_back("head_pan_joint");
    head_goal.trajectory.joint_names.push_back("head_tilt_joint");
    head_goal.trajectory.points.resize(1);
    head_goal.trajectory.points[0].positions.resize(2);
    head_goal.trajectory.points[0].positions[0] = 0.0;
    head_goal.trajectory.points[0].positions[1] = tilt_angle;
    head_goal.trajectory.points[0].velocities.resize(2);
    head_goal.trajectory.points[0].velocities[0] = 0.0;
    head_goal.trajectory.points[0].velocities[1] = 0.0;
    head_goal.trajectory.points[0].time_from_start = ros::Duration(sspeed);
    return head_goal;
}


void rotate_fetch(ros::Publisher pub, float sspeed) {
    // gets the inital yaw angle to return to
    tf::TransformListener tf_req;
    tf::StampedTransform base_T_world_0;
    float initial_yaw;
    float current_yaw;
    bool continue_rotating;
    geometry_msgs::Twist rotate_vel;

    // initializes the speed for rotation motion
    rotate_vel.linear.x = 0.0;
    rotate_vel.linear.y = 0.0;
    rotate_vel.linear.z = 0.0;
    rotate_vel.angular.x = 0.0;
    rotate_vel.angular.y = 0.0;
    rotate_vel.angular.z = sspeed;
    //boost::this_thread::sleep_for(boost::chrono::seconds{3});

    // gets the initial yaw
    if (!tf_req.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(2.0))) {
        ROS_ERROR("base_link_T_map timed out... Error recovery?");
    }
    try{
        tf_req.lookupTransform("/base_link", "/map", ros::Time(0), base_T_world_0);
    } catch (tf::TransformException ex){
        ROS_ERROR("Transform of base_link got %s error status... Error recovery?",ex.what());
    }
    initial_yaw = tf::getYaw(base_T_world_0.getRotation());

    // moves past initial yaw
    for (int i=0;i<10;i++) {
        pub.publish(rotate_vel);
        boost::this_thread::sleep_for(boost::chrono::milliseconds{300});
        pub.publish(rotate_vel);
        boost::this_thread::sleep_for(boost::chrono::milliseconds{300});
        pub.publish(rotate_vel);
        boost::this_thread::sleep_for(boost::chrono::milliseconds{300});
    }

    // rotates while mapping until initial yaw reached
    continue_rotating = true;
    while (ros::ok() && continue_rotating) {
        try{
            tf_req.lookupTransform("/base_link", "/map", ros::Time(0), base_T_world_0);
        } catch (tf::TransformException ex){
            ROS_ERROR("Transform of base_link got %s error status... Error recovery?",ex.what());
        }
        current_yaw = tf::getYaw(base_T_world_0.getRotation());
        continue_rotating = !( (current_yaw*initial_yaw >= 0) && (fabs(current_yaw-initial_yaw) < ONE_DEGREE*2) );
        boost::this_thread::sleep_for(boost::chrono::milliseconds{300});
        pub.publish(rotate_vel);
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "challenge_mapping");
    ros::NodeHandle demo_nh;

    // gets roslaunch params
    float look_up_angle;
    float look_down_angle;
    float tilt_period;
    float rotation_speed;
    demo_nh.getParam("/challenge_mapping/look_up_angle", look_up_angle);
    demo_nh.getParam("/challenge_mapping/look_down_angle", look_down_angle);
    demo_nh.getParam("/challenge_mapping/tilt_period", tilt_period);
    demo_nh.getParam("/challenge_mapping/rotation_speed", rotation_speed);
    std::string map_path_2d;
    std::string map_path_3d;
    demo_nh.getParam("/challenge_mapping/2d_map_path", map_path_2d);
    demo_nh.getParam("/challenge_mapping/3d_map_path", map_path_3d);

    ROS_INFO("period: %f",tilt_period);
    ROS_INFO("down angle: %f",look_down_angle);
    ROS_INFO("up angle: %f",look_up_angle);
    ROS_INFO("rot speed: %f",rotation_speed);

    // prepares head motion client
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_action_client("/head_controller/follow_joint_trajectory", true);
    ROS_INFO("Waiting for head controllers.");
    head_action_client.waitForServer();
    // prepages base motion client
    ros::Publisher cmd_vel_pub = demo_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ROS_INFO("Motion control ready.");

    // makes two look goals
    control_msgs::FollowJointTrajectoryGoal look_down = look_goal(look_down_angle,tilt_period);
    control_msgs::FollowJointTrajectoryGoal look_up = look_goal(look_up_angle,tilt_period);

    // starts rotation thread for mapping
    boost::thread rot_thread{rotate_fetch, cmd_vel_pub, rotation_speed};

    // begins head tilting until rotation thread has stopped
    bool goal_toggle = 0;
    bool continue_tilting = true;
    while (ros::ok() && continue_tilting) {
        if (goal_toggle) {
            ROS_DEBUG("Sending look up.");
            head_action_client.sendGoal(look_up);
            goal_toggle = 0;
        } else {
            ROS_DEBUG("Sending look down.");
            head_action_client.sendGoal(look_down);
            goal_toggle = 1;
        }
        //wait for the action to return and check head motion success
        bool finished_before_timeout = head_action_client.waitForResult(ros::Duration(5.0));
        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState action_error = head_action_client.getState();
            if (action_error != actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
                ROS_ERROR("Head tilting got %s error status... Error recovery?",action_error.toString().c_str());
            }
        } else {
            ROS_ERROR("Head tilting did't finish in time... Error recovery?");
        }

        continue_tilting = !rot_thread.timed_join(boost::posix_time::seconds(0));
    }

    std::string str;
    std::cout << "enter any string to save the map" << std::endl;
    std::cin >> str;

    // saves 3D map
    std::system(("rosrun octomap_server octomap_saver "+map_path_3d).c_str());
    // saves 2D map
    std::system(("rosrun map_server map_saver -f "+map_path_2d).c_str());

    ros::Duration(3,0).sleep();

    exit(0);
}
