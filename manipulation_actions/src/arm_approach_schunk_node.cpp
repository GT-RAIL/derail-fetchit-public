#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "arm_approach_schunk_node");

    // preps moveit
    moveit::planning_interface::MoveGroupInterface* arm_group = new moveit::planning_interface::MoveGroupInterface("arm");
    arm_group->startStateMonitor();

    // approach the schunk pose that is static tf
    geometry_msgs::PoseStamped schunk_approach_pose;
    schunk_approach_pose.header.frame_id = "arm_approach_schunk_pose";
    schunk_approach_pose.pose.position.x = 0;
    schunk_approach_pose.pose.position.y = 0;
    schunk_approach_pose.pose.position.z = 0;
    schunk_approach_pose.pose.orientation.x = 0;
    schunk_approach_pose.pose.orientation.y = 0;
    schunk_approach_pose.pose.orientation.z = 0;
    schunk_approach_pose.pose.orientation.w = 1;

    // plan and move to approach pose
    arm_group->setPlannerId("arm[RRTConnectkConfigDefault]");
    arm_group->setPlanningTime(1.5);
    arm_group->setStartStateToCurrentState();
    arm_group->setPoseTarget(schunk_approach_pose, "gripper_link");
    moveit_msgs::MoveItErrorCodes error_code = arm_group->move();

    // checks results
    if (error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED) {
        ROS_INFO("Preempted while moving to approach pose.");
        return -1;
    } else if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_INFO("Failed to move to approach pose.");
        return -1;
    } else {
        ROS_INFO("Succeeded to move to approach pose.");
    }

    ros::spin();

    return 1;
}