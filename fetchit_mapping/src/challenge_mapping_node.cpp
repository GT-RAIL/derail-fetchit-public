#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>


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
    geometry_msgs::Twist rotate_vel;
    rotate_vel.linear.x = 0.0;
    rotate_vel.linear.y = 0.0;
    rotate_vel.linear.z = 0.0;
    rotate_vel.angular.x = 0.0;
    rotate_vel.angular.y = 0.0;
    rotate_vel.angular.z = sspeed;
    boost::this_thread::sleep_for(boost::chrono::seconds{5});
    while (ros::ok()) {
        boost::this_thread::sleep_for(boost::chrono::milliseconds{300});
        pub.publish(rotate_vel);
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "challenge_mapping");
    ros::NodeHandle demo_nh;
    // prepares head motion client
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_action_client("/head_controller/follow_joint_trajectory", true);
    ROS_INFO("Waiting for head controllers.");
    head_action_client.waitForServer();
    // prepages base motion client
    ros::Publisher cmd_vel_pub = demo_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ROS_INFO("Motion control ready.");

    // makes two look goals
    control_msgs::FollowJointTrajectoryGoal look_down = look_goal(0.7,2.0);
    control_msgs::FollowJointTrajectoryGoal look_up = look_goal(-0.2,2.0);

    // starts rotation thread for mapping
    boost::thread rot_thread{rotate_fetch, cmd_vel_pub, 0.2};

    // begins head tilting for mapping
    bool goal_toggle = 0;
    while (ros::ok()) {
        if (goal_toggle) {
            ROS_DEBUG("Sending loop up.");
            head_action_client.sendGoal(look_up);
            goal_toggle = 0;
        } else {
            ROS_DEBUG("Sending loop down.");
            head_action_client.sendGoal(look_down);
            goal_toggle = 1;
        }
        //wait for the action to return and check
        bool finished_before_timeout = head_action_client.waitForResult(ros::Duration(5.0));
        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState action_error = head_action_client.getState();
            if (action_error != actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
                ROS_DEBUG("Head tilting got %s error status... Error recovery?",action_error.toString().c_str());
            }
        } else {
            ROS_DEBUG("Head tilting did't finish in time... Error recovery?");
        }
    }

    //exit
    return 0;
}