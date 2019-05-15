#include <manipulation_actions/SchunkDoor.h>
#include <manipulation_actions/ApproachSchunkResult.h>

SchunkDoor::SchunkDoor():
    pnh("~"),
    tf_listener(tf_buffer),
    schunk_door_server(pnh, "schunk_door", boost::bind(&SchunkDoor::executeDoorAction, this, _1), false),
    linear_move_client("schunk_linear_controller/linear_move")
{
    pnh.param<float>("approach_angle", approach_angle, PI/2); // have linear controller hold position or not

    // preps moveit
    arm_group_ = new moveit::planning_interface::MoveGroupInterface("arm");
    arm_group_->startStateMonitor();

    // preps planning scene for adding collision objects as needed
    planning_scene_interface_ = new moveit::planning_interface::PlanningSceneInterface();

    schunk_door_server.start();
    ROS_INFO("schunk door node ready!");

}

void SchunkDoor::executeDoorAction (const manipulation_actions::SchunkDoorGoalConstPtr &goal){

    manipulation_actions::SchunkDoorResult result;
    reference_frame_ = goal->approach_transform.child_frame_id;


    if (reference_frame_ != handle_frame_) {
        schunk_door_server.setAborted(result);
        return;
    }


    // publishes static approach pose to tf
    std::vector<geometry_msgs::TransformStamped> static_poses;
    static_poses.push_back(goal->approach_transform);
    static_broadcaster_.sendTransform(static_poses);


    geometry_msgs::PoseStamped pre_approach_gripper_pose_stamped;
    geometry_msgs::Point door_operation_finish_goal;


    // get gripper preapproach pose in base link frame
    if (!getGripperPreApproachPose(pre_approach_gripper_pose_stamped)) {
        schunk_door_server.setAborted(result);
        return;
    }

    planToPose(pre_approach_gripper_pose_stamped, "base_link", approach_plan)


//    if (!goal->open) {
//    }


    // moveit call to preapproach pose above the door handle

    // linear controller call to move down to the door handle

    manipulation_actions::LinearMoveGoal linear_goal;
    linear_goal.hold_final_pose = true;
    linear_goal.point.x = pre_approach_gripper_pose_stamped.pose.position.x;
    linear_goal.point.y = pre_approach_gripper_pose_stamped.pose.position.y;
    linear_goal.point.z = pre_approach_gripper_pose_stamped.pose.position.z - 0.2;

    linear_move_client.sendGoal(linear_goal);
    linear_move_client.waitForResult();



    // get gripper position in required to close the schunk door


    // linear controller call to close the schunk door

    // linear_goal.point.x = ;
    // linear_goal.point.y = ;


    // linear controller call to move arm back up

    // linear_goal.point.z += 0.2;



    return;
}


void SchunkDoor::planToPose(geometry_msgs::PoseStamped& pose, std::string& pose_frame, moveit::planning_interface::MoveGroupInterface::Plan& pose_plan) {
    int max_planning_attempts = 3;
    for (int num_attempts = 0; num_attempts < max_planning_attempts; num_attempts++) {
        ROS_INFO("Planning path to %s pose. Attempt: %d/%d", pose_name, num_attempts + 1, max_planning_attempts);

        // preps moveit
        arm_group_->setPlannerId("arm[RRTConnectkConfigDefault]");
        arm_group_->setPlanningTime(1.5);
        arm_group_->setStartStateToCurrentState();
        if (motion_speed_scale_factor_ != 1.0)
            arm_group_->setMaxVelocityScalingFactor(motion_speed_scale_factor_);

        // sets the planning goal
        arm_group_->setPoseTarget(pose, pose_frame);

        // plans to the target pose
        moveit::planning_interface::MoveItErrorCode plan_result = arm_group_->plan(pose_plan);

        // checks the results
        if (approach_schunk_server_.isPreemptRequested()) {
            return;
        } else if (plan_result.val != moveit::planning_interface::MoveItErrorCode::SUCCESS
                   && num_attempts >= max_planning_attempts - 1) {
            ROS_WARN("Moveit planning failure");
            ROS_ERROR("Planning error code is: %d",plan_result.val);
            return;
        } else if (plan_result.val != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            continue;
        } else {
            break; // successful plan so return
        }
    }
    return;
}


// get the transform for a preapproach frame in the base_link using the schunk_approach_frame
bool SchunkDoor::getGripperPreApproachPose(geometry_msgs::PoseStamped& pre_approach_gripper_pose_stamped) {

    geometry_msgs::TransformStamped handle_in_base_msg;
    try {
        handle_in_base_msg = tf_buffer.lookupTransform(handle_frame_, "base_link", ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        return false;
    }

    handle_in_base_msg.transform.translation.z += 0.2;
    handle_in_base_msg.transform.rotation.w = cos(PI/4)*cos(approach_angle / 2);
    handle_in_base_msg.transform.rotation.x = sin(PI/4)*sin(approach_angle / 2);
    handle_in_base_msg.transform.rotation.y = cos(PI/4)*sin(approach_angle / 2);
    handle_in_base_msg.transform.rotation.z = -sin(PI/4)*cos(approach_angle / 2);


    tf2::Transform handle_in_base_tf;

//    tf2::fromMsg(handle_in_base_msg, handle_in_base_tf);
//    tf2::toMsg(handle_in_base_tf, pre_approach_gripper_pose_stamped.pose);
    pre_approach_gripper_pose_stamped.header.frame_id = "base_link";

    return true;
}

bool SchunkDoor::getGripperDoorClosePos(geometry_msgs::Point& door_closed_gripper_pos){
    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "schunk_door_node");
    // ros::NodeHandle nh, pnh("~");
}