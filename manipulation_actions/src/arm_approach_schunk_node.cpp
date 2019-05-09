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
#include <tf2/impl/utils.h>

#include <math.h>

void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::TransformStamped& msg, ros::Time stamp, const std::string& frame_id, const std::string& child_frame_id) {
    msg.transform.translation.x = tf2.getOrigin().x();
    msg.transform.translation.y = tf2.getOrigin().y();
    msg.transform.translation.z = tf2.getOrigin().z();
    msg.transform.rotation.x = tf2.getRotation().x();
    msg.transform.rotation.y = tf2.getRotation().y();
    msg.transform.rotation.z = tf2.getRotation().z();
    msg.transform.rotation.w = tf2.getRotation().w();
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.child_frame_id = child_frame_id;
}

geometry_msgs::TransformStamped alignObjectFrame(tf2_ros::Buffer& tf_buffer) {
    // gets tf between object_frame and gripper
    geometry_msgs::TransformStamped gripper_to_object = tf_buffer.lookupTransform("gripper_link", "object_frame",
                                                                                  ros::Time(0), ros::Duration(1.0));
    tf2::Transform gripper_to_object_tf;
    tf2::fromMsg(gripper_to_object.transform, gripper_to_object_tf);

    // extracts roll, pitch, yaw from current gripper_to_object_tf
    tf2::Quaternion gripper_to_object_Q = gripper_to_object_tf.getRotation();
    double roll, pitch, yaw;
    tf2::Matrix3x3 mat(gripper_to_object_Q);
    mat.getRPY(roll, pitch, yaw);

    // sets roll amount required to align object_frame
    double roll_amount = 0;
    if (roll == 0) {
        if (pitch < 0) {
            roll_amount = -yaw;
        } else if (pitch > 0) {
            roll_amount = yaw+M_PI;
        }
    } else {
        if (pitch < 0) {
            roll_amount = -yaw-M_PI;
        } else if (pitch > 0) {
            roll_amount = yaw;
        }
    }

    // computes the aligned_object_frame_tf
    tf2::Quaternion update_gripper_to_object_Q = tf2::Quaternion(0,roll_amount,0);
    tf2::Quaternion aligned_gripper_to_object_Q = gripper_to_object_Q * update_gripper_to_object_Q;
    tf2::Transform aligned_gripper_to_object_tf;
    aligned_gripper_to_object_tf.setOrigin(tf2::Vector3(gripper_to_object.transform.translation.x,gripper_to_object.transform.translation.y,gripper_to_object.transform.translation.z));
    aligned_gripper_to_object_tf.setRotation(aligned_gripper_to_object_Q);

    // final check to see if we need to rotate by 180 degrees
    tf2::Vector3 align_vector(-1, 0, 0);
    tf2::Matrix3x3 rotation_mat(aligned_gripper_to_object_tf.getRotation());
    tf2::Vector3 z_vector(0, 0, 1);
    tf2::Vector3 new_vector = rotation_mat*z_vector;
    double angle_offset = acos(new_vector.dot(align_vector));
    if (angle_offset > M_PI_2)
    {
        // roll by 180 degrees
        ROS_INFO("Flipping orientation to align object_frame z-axis with gripper negative x-axis");
        tf2::Quaternion roll_adjustment;
        roll_adjustment.setRPY(M_PI, 0, 0);
        aligned_gripper_to_object_tf.setRotation(aligned_gripper_to_object_tf.getRotation() * roll_adjustment);
    }

    // generates geometry_msg from tf for publishing
    geometry_msgs::TransformStamped aligned_gripper_to_object_transformStamped;
    transformTF2ToMsg(aligned_gripper_to_object_tf, aligned_gripper_to_object_transformStamped,ros::Time::now(),
                      "gripper_link","aligned_object_frame");
    return aligned_gripper_to_object_transformStamped;
}

geometry_msgs::PoseStamped getGripperPreApproachPose(tf2_ros::Buffer& tf_buffer) {
    // gets tf between aligned_object_frame and gripper_link
    geometry_msgs::TransformStamped object_to_gripper = tf_buffer.lookupTransform("aligned_object_frame", "gripper_link",
                                                                                  ros::Time(0), ros::Duration(1.0));
    tf2::Transform object_to_gripper_tf;
    tf2::fromMsg(object_to_gripper.transform, object_to_gripper_tf);

    // gets tf between aligned_object_frame and arm_approach_schunk_pose
    geometry_msgs::TransformStamped approach_to_object = tf_buffer.lookupTransform("arm_approach_schunk_pose", "aligned_object_frame",
                                                                                  ros::Time(0), ros::Duration(1.0));

    // gets pose to align the aligned_object_frame to the arm_approach_schunk_pose outside of the schunk
    approach_to_object.transform.rotation.x = 0;
    approach_to_object.transform.rotation.y = 0;
    approach_to_object.transform.rotation.z = 0;
    approach_to_object.transform.rotation.w = 1;
    tf2::Transform pre_schunk_approach_pose_tf;
    tf2::fromMsg(approach_to_object.transform, pre_schunk_approach_pose_tf);

    // gets the pre_approach_gripper_pose_tf which accounts for object
    tf2::Transform pre_approach_gripper_pose_tf;
    pre_approach_gripper_pose_tf = pre_schunk_approach_pose_tf * object_to_gripper_tf;

    // converts from tf to geometry_msgs
    geometry_msgs::PoseStamped pre_approach_gripper_pose_stamped;
    tf2::toMsg(pre_approach_gripper_pose_tf, pre_approach_gripper_pose_stamped.pose);
    pre_approach_gripper_pose_stamped.header.frame_id = "arm_approach_schunk_pose";
    return pre_approach_gripper_pose_stamped;
}

geometry_msgs::PoseStamped getGripperFinalApproachPose(tf2_ros::Buffer& tf_buffer) {
    // gets tf between aligned_object_frame and gripper_link
    geometry_msgs::TransformStamped object_to_gripper = tf_buffer.lookupTransform("aligned_object_frame", "gripper_link",
                                                                                  ros::Time(0), ros::Duration(1.0));
    tf2::Transform object_to_gripper_tf;
    tf2::fromMsg(object_to_gripper.transform, object_to_gripper_tf);

    // converts from tf to geometry_msgs
    geometry_msgs::PoseStamped final_approach_gripper_pose_stamped;
    tf2::toMsg(object_to_gripper_tf, final_approach_gripper_pose_stamped.pose);
    final_approach_gripper_pose_stamped.header.frame_id = "arm_approach_schunk_pose";
    return final_approach_gripper_pose_stamped;
}

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

    /* TODO task will run following:
    1. grasp large gear
    2. move are to ready
    3. go to origin
    4. in hand localize
    5. move arm to ready
    6. go to schunk
    7. look at schunk corner
    8. get pose of chuck
    9. move arm to pre-chuck approach joint pose
    10. move arm to approach chuck pose accounting for large gear
    Intermmedite action commands:
    rosrun task_executor run_action.py arm '{"poses":"joint_poses.ready"}'
    rosrun task_executor run_action.py arm '{"poses":[0.09128400231079102,-1.144515101536499,2.9994977412109374,-1.1364141782196044,-3.147721389633179,1.4671756200775146,-1.5945427632195435]}'
    rosrun task_executor run_action.py in_hand_localize '{"disambiguate_direction":true}'
    rosrun task_executor run_action.py load_static_octomap {}
    */

    // removes the object from the planning scene
    bool attach_arbitrary_object = false;
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

    // updates object_frame alignment to make chuck approach easier
    geometry_msgs::TransformStamped aligned_object_to_gripper_tfS = alignObjectFrame(tf_buffer);
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    static_broadcaster.sendTransform(aligned_object_to_gripper_tfS);

    // gets the pre_approach_gripper_pose for planning
    geometry_msgs::PoseStamped pre_approach_gripper_pose_stamped = getGripperPreApproachPose(tf_buffer);

    // plan and move to approach pose
    arm_group->setPlannerId("arm[RRTConnectkConfigDefault]");
    arm_group->setPlanningTime(1.5);
    arm_group->setStartStateToCurrentState();
    arm_group->setPoseTarget(pre_approach_gripper_pose_stamped, "gripper_link");
    arm_group->setMaxVelocityScalingFactor(0.3);

    // debugging tfs
    ROS_INFO("Publishing tfs");
    geometry_msgs::TransformStamped pre_approach_gripper_pose_transformStamped;
    pre_approach_gripper_pose_transformStamped.header.stamp = ros::Time::now();
    pre_approach_gripper_pose_transformStamped.header.frame_id = "arm_approach_schunk_pose";
    pre_approach_gripper_pose_transformStamped.child_frame_id = "pre_approach_gripper_pose";
    pre_approach_gripper_pose_transformStamped.transform.translation.x = pre_approach_gripper_pose_stamped.pose.position.x;
    pre_approach_gripper_pose_transformStamped.transform.translation.y = pre_approach_gripper_pose_stamped.pose.position.y;
    pre_approach_gripper_pose_transformStamped.transform.translation.z = pre_approach_gripper_pose_stamped.pose.position.z;
    pre_approach_gripper_pose_transformStamped.transform.rotation = pre_approach_gripper_pose_stamped.pose.orientation;

    // publishes transforms
    std::vector<geometry_msgs::TransformStamped> static_poses;
    static_poses.push_back(aligned_object_to_gripper_tfS);
    static_poses.push_back(pre_approach_gripper_pose_transformStamped);
    static_broadcaster.sendTransform(static_poses);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Moving arm to pre-approach");
    moveit_msgs::MoveItErrorCodes error_code = arm_group->move();
    ROS_INFO("Done");
    // checks results
    if (error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED) {
        ROS_INFO("Preempted while moving to pre-approach pose.");
        if (attach_arbitrary_object) {
            arm_group->detachObject("arbitrary_gripper_object");
            std::vector<std::string> obj_ids;
            obj_ids.push_back("arbitrary_gripper_object");
            planning_scene_interface->removeCollisionObjects(obj_ids);
        }
    } else if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_INFO("Failed to move to pre-approach pose.");
        if (attach_arbitrary_object) {
            arm_group->detachObject("arbitrary_gripper_object");
            std::vector<std::string> obj_ids;
            obj_ids.push_back("arbitrary_gripper_object");
            planning_scene_interface->removeCollisionObjects(obj_ids);
        }
    } else {
        ROS_INFO("Succeeded to move to pre-approach pose.");

        // gets the final_approach_gripper_pose for planning
        geometry_msgs::PoseStamped final_approach_gripper_pose_stamped = getGripperFinalApproachPose(tf_buffer);

        // debugging tfs
        ROS_INFO("Publishing tfs");
        geometry_msgs::TransformStamped final_approach_gripper_pose_transformStamped;
        final_approach_gripper_pose_transformStamped.header.stamp = ros::Time::now();
        final_approach_gripper_pose_transformStamped.header.frame_id = "arm_approach_schunk_pose";
        final_approach_gripper_pose_transformStamped.child_frame_id = "pre_approach_gripper_pose";
        final_approach_gripper_pose_transformStamped.transform.translation.x = final_approach_gripper_pose_stamped.pose.position.x;
        final_approach_gripper_pose_transformStamped.transform.translation.y = final_approach_gripper_pose_stamped.pose.position.y;
        final_approach_gripper_pose_transformStamped.transform.translation.z = final_approach_gripper_pose_stamped.pose.position.z;
        final_approach_gripper_pose_transformStamped.transform.rotation = final_approach_gripper_pose_stamped.pose.orientation;

        // publishes transforms
        static_poses.push_back(final_approach_gripper_pose_transformStamped);
        static_broadcaster.sendTransform(static_poses);

        // plans and moves to final gripper pose
        arm_group->setStartStateToCurrentState();
        arm_group->setPoseTarget(final_approach_gripper_pose_stamped, "gripper_link");
        ROS_INFO("Moving arm to pre-approach");
        error_code = arm_group->move();

        if (attach_arbitrary_object) {
            arm_group->detachObject("arbitrary_gripper_object");
            std::vector<std::string> obj_ids;
            obj_ids.push_back("arbitrary_gripper_object");
            planning_scene_interface->removeCollisionObjects(obj_ids);
        }

        // checks results
        if (error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED) {
            ROS_INFO("Preempted while moving to approach pose.");
        } else if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
            ROS_INFO("Failed to move to approach pose.");
        } else {
            ROS_INFO("Succeeded to move to approach pose.");
        }
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