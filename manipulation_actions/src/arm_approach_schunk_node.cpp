#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <manipulation_actions/InHandLocalizeAction.h>
#include <manipulation_actions/InHandLocalizeGoal.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "arm_approach_schunk_node");

    // preps tf
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // preps moveit
    moveit::planning_interface::MoveGroupInterface* arm_group = new moveit::planning_interface::MoveGroupInterface("arm");
    arm_group->startStateMonitor();
    // preps planning scene for adding collision objects as needed
    moveit::planning_interface::PlanningSceneInterface* planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();

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

    // localize the object in hand
    /*actionlib::SimpleActionClient<manipulation_actions::InHandLocalizeAction> localize_client("/in_hand_localizer/localize");
    ROS_INFO("Waiting for in-hand localization server");
    localize_client.waitForServer();
    manipulation_actions::InHandLocalizeGoal ihl_goal;
    ihl_goal.correct_object_direction = true;
    localize_client.sendGoal(ihl_goal);
    localize_client.waitForResult();
    actionlib::SimpleClientGoalState status = localize_client.getState();
    if (status == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("in-hand localization succeeded");
    }*/

    // removes the object from the planning scene
    bool attach_arbitrary_object = true;
    if (attach_arbitrary_object) {
        // add an arbitrary object to planning scene for testing (typically this would be done at grasp time)
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.resize(1);
        collision_objects[0].header.frame_id = "gripper_link";
        collision_objects[0].id = "arbitrary_gripper_object";
        shape_msgs::SolidPrimitive shape;
        shape.type = shape_msgs::SolidPrimitive::SPHERE;
        shape.dimensions.resize(1);
        shape.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = 0.15;
        collision_objects[0].primitives.push_back(shape);
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        collision_objects[0].primitive_poses.push_back(pose);
        planning_scene_interface->addCollisionObjects(collision_objects);

        ros::Duration(0.5).sleep();

        std::vector<std::string> touch_links;
        touch_links.emplace_back("r_gripper_finger_link");
        touch_links.emplace_back("l_gripper_finger_link");
        touch_links.emplace_back("gripper_link");
        touch_links.emplace_back("wrist_roll_link");
        touch_links.emplace_back("wrist_flex_link");
        arm_group->attachObject("arbitrary_gripper_object", "gripper_link", touch_links);
    }

    // gets tf between object and gripper
    geometry_msgs::TransformStamped object_to_gripper = tf_buffer.lookupTransform("object_frame", "gripper_link",
                                                                                  ros::Time(0), ros::Duration(1.0));
    tf2::Transform object_to_gripper_tf;
    tf2::fromMsg(object_to_gripper.transform, object_to_gripper_tf);

    // gets tf for schunk approach pose
    tf2::Transform schunk_approach_pose_tf;
    tf2::fromMsg(schunk_approach_pose.pose, schunk_approach_pose_tf);

    // gets the final tf which accounts for object
    tf2::Transform gripper_pose_tf;
    gripper_pose_tf = schunk_approach_pose_tf * object_to_gripper_tf;

    // converts from tf to geometry_msgs
    geometry_msgs::Pose gripper_pose;
    tf2::toMsg(gripper_pose_tf, gripper_pose);
    geometry_msgs::PoseStamped gripper_pose_stamped;
    gripper_pose_stamped.pose = gripper_pose;
    gripper_pose_stamped.header.frame_id = "arm_approach_schunk_pose";

    std::cout << gripper_pose_stamped << std::endl;

    // plan and move to approach pose
    arm_group->setPlannerId("arm[RRTConnectkConfigDefault]");
    arm_group->setPlanningTime(1.5);
    arm_group->setStartStateToCurrentState();
    arm_group->setPoseTarget(gripper_pose_stamped, "gripper_link");
    arm_group->setMaxVelocityScalingFactor(0.3);

    ROS_INFO("Publishing tfs");

    // TODO DEBUG
    static tf2_ros::StaticTransformBroadcaster object_to_gripper_broadcaster;
    object_to_gripper.header.frame_id = "arm_approach_schunk_pose";
    object_to_gripper.child_frame_id = "final_object_pose";
    object_to_gripper_broadcaster.sendTransform(object_to_gripper);

    static tf2_ros::StaticTransformBroadcaster gripper_pose_broadcaster;
    geometry_msgs::TransformStamped gripper_pose_transformStamped;
    gripper_pose_transformStamped.header.stamp = ros::Time::now();
    gripper_pose_transformStamped.header.frame_id = "arm_approach_schunk_pose";
    gripper_pose_transformStamped.child_frame_id = "final_gripper_pose";
    gripper_pose_transformStamped.transform.translation.x = gripper_pose.position.x;
    gripper_pose_transformStamped.transform.translation.y = gripper_pose.position.y;
    gripper_pose_transformStamped.transform.translation.z = gripper_pose.position.z;
    gripper_pose_transformStamped.transform.rotation = gripper_pose.orientation;
    gripper_pose_broadcaster.sendTransform(gripper_pose_transformStamped);

    // TODO add rotations to final desired object pose

    ROS_INFO("Starting to move arm");

    moveit_msgs::MoveItErrorCodes error_code = arm_group->move();

    if (attach_arbitrary_object) {
        arm_group->detachObject("arbitrary_gripper_object");
        std::vector<std::string> obj_ids;
        obj_ids.push_back("arbitrary_gripper_object");
        planning_scene_interface->removeCollisionObjects(obj_ids);
    }

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

    try{
        ros::Rate loop_rate(100);
        while (ros::ok())
        {
            ros::spinOnce();

            loop_rate.sleep();
        }
    }catch(std::runtime_error& e){
        ROS_ERROR("arm_approach_schunk_node exception: %s", e.what());
        return -1;
    }

    return 1;
}