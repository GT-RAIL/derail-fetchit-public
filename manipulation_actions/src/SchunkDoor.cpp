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

    // segmentation client to get the handle
    std::string seg_node = "rail_segmentation";
    pnh.param("segmentation_node", seg_node);
    seg_client_ = n.serviceClient<rail_manipulation_msgs::SegmentObjects>(seg_node+"/segment_objects");

    // handle filtering shape params
    handle_width_min_ = 0.001;
    handle_width_max_ = 0.03;
    handle_depth_min_ = 0.05;
    handle_depth_max_ = 0.05;
    handle_height_min_ = 0.001;
    handle_height_max_ = 0.03;
    pnh.param("handle_width_min", handle_width_min_);
    pnh.param("handle_width_max", handle_width_max_);
    pnh.param("handle_depth_min", handle_depth_min_);
    pnh.param("handle_depth_max", handle_depth_max_);
    pnh.param("handle_height_min", handle_height_min_);
    pnh.param("handle_height_max", handle_height_max_);

    schunk_door_server.start();
    ROS_INFO("schunk door node ready!");

}

void SchunkDoor::executeDoorAction (const manipulation_actions::SchunkDoorGoalConstPtr &goal){
    manipulation_actions::SchunkDoorResult result;

    // runs segmentation to get the center of the handle in base_link frame. no orientation assumed
    tf2::Transform base_link_to_handle_tf;
    if (!getHandleInBase(goal->approach_transform,base_link_to_handle_tf)) {
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

    // moveit call to preapproach pose above the door handle

    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    planToPose(pre_approach_gripper_pose_stamped, base_frame_, approach_plan);
    moveit_msgs::MoveItErrorCodes error_code = arm_group_->execute(approach_plan);


    // linear controller call to move down to the door handle

    manipulation_actions::LinearMoveGoal linear_goal;
    linear_goal.hold_final_pose = true;
    linear_goal.point.x = pre_approach_gripper_pose_stamped.pose.position.x;
    linear_goal.point.y = pre_approach_gripper_pose_stamped.pose.position.y;
    linear_goal.point.z = pre_approach_gripper_pose_stamped.pose.position.z - 0.2;

    linear_move_client.sendGoal(linear_goal);
    linear_move_client.waitForResult();



    // get gripper position in required to close the schunk door
    getGripperDoorClosePos(door_operation_finish_goal);

    // linear controller call to close the schunk door
    linear_goal.point.x = door_operation_finish_goal.x;
    linear_goal.point.y = door_operation_finish_goal.y;
    linear_goal.point.z = door_operation_finish_goal.z;

    linear_move_client.sendGoal(linear_goal);
    linear_move_client.waitForResult();

    // linear controller call to move arm back up

     linear_goal.point.z += 0.2;

    linear_move_client.sendGoal(linear_goal);
    linear_move_client.waitForResult();

    
    return;
}

bool SchunkDoor::getHandleInBase(geometry_msgs::TransformStamped og_map_to_schunk, tf2::Transform base_link_to_handle_tf) {
    // gets transform from map to base_link
    geometry_msgs::TransformStamped base_link_to_map;
    try{
        base_link_to_map = tf_buffer.lookupTransform("base_link","map",ros::Time(0),ros::Duration(1.0));
    } catch (tf2::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return false;
    }

    // gets the handle orientation in base_link
    tf2::Quaternion roll90_Q = tf2::Quaternion(0.7071068,0,0,0.7071068);
    tf2::Quaternion og_map_to_schunk_Q;
    tf2::fromMsg(og_map_to_schunk.transform.rotation,og_map_to_schunk_Q);
    tf2::Quaternion og_schunk_to_map_Q = og_map_to_schunk_Q.inverse();
    tf2::Quaternion base_link_to_map_Q;
    tf2::fromMsg(base_link_to_map.transform.rotation,base_link_to_map_Q);
    tf2::Quaternion base_link_to_handle_Q = roll90_Q * og_schunk_to_map_Q * base_link_to_map_Q;

    // gets the handle position in base_link using segmentation
    geometry_msgs::Point base_link_to_handle_P;
    rail_manipulation_msgs::SegmentObjects seg_srv;
    if (!seg_client_.call(seg_srv)) {
        ROS_ERROR("Failed to call segmentation service segment_objects");
        return false;
    }
    ROS_INFO("Number segmented objects: %lu", seg_srv.response.segmented_objects.objects.size());

    // filters segmented objects by shape and gets handle position
    for (int i = 0; i < seg_srv.response.segmented_objects.objects.size(); i++) {
        rail_manipulation_msgs::SegmentedObject handle_candidate = seg_srv.response.segmented_objects.objects[i];
        if (!inTolerance(handle_candidate.width,handle_width_min_,handle_width_max_) &&
                !inTolerance(handle_candidate.depth,handle_depth_min_,handle_depth_max_) &&
                !inTolerance(handle_candidate.height,handle_height_min_,handle_height_max_)) {
            continue;
        } else {
            base_link_to_handle_P = handle_candidate.center;
            break;
        }
    }

    // sets return values
    base_link_to_handle_tf.setOrigin(tf2::Vector3(base_link_to_handle_P.x,base_link_to_handle_P.y,base_link_to_handle_P.z));
    base_link_to_handle_tf.setRotation(base_link_to_handle_Q);
    return true;
}

bool SchunkDoor::inTolerance(float value, float min, float max) {
    if (value > min && value < max) {
        return true;
    } else {
        return false;
    }
}


void SchunkDoor::planToPose(geometry_msgs::PoseStamped& pose, std::string& pose_frame, moveit::planning_interface::MoveGroupInterface::Plan& pose_plan) {
    int max_planning_attempts = 3;
    for (int num_attempts = 0; num_attempts < max_planning_attempts; num_attempts++) {
//        ROS_INFO("Planning path to %s pose. Attempt: %d/%d", pose_name, num_attempts + 1, max_planning_attempts);

        // preps moveit
        arm_group_->setPlannerId("arm[RRTConnectkConfigDefault]");
        arm_group_->setPlanningTime(1.5);
        arm_group_->setStartStateToCurrentState();
        arm_group_->setMaxVelocityScalingFactor(0.3);

        // sets the planning goal
        arm_group_->setPoseTarget(pose, pose_frame);

        // plans to the target pose
        moveit::planning_interface::MoveItErrorCode plan_result = arm_group_->plan(pose_plan);

        // checks the results
        if (schunk_door_server.isPreemptRequested()) {
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

    tf2::fromMsg(handle_in_base_msg.transform, handle_in_base_tf);
    tf2::toMsg(handle_in_base_tf, pre_approach_gripper_pose_stamped.pose);
    pre_approach_gripper_pose_stamped.header.frame_id = "base_link";

    return true;
}


bool SchunkDoor::getGripperDoorClosePos(geometry_msgs::Point& door_closed_gripper_pos){


    geometry_msgs::TransformStamped handle_in_base_msg;
    try {
        handle_in_base_msg = tf_buffer.lookupTransform(handle_frame_, "base_link", ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        return false;
    }


    tf2::Transform handle_in_base_tf;
    tf2::fromMsg(handle_in_base_msg.transform, handle_in_base_tf);

    door_closed_gripper_pos.x = -0.4;
    door_closed_gripper_pos.y = 0.0;
    door_closed_gripper_pos.z = 0.0;

    tf2::Vector3 door_closed_gripper_pos_goal;
    tf2::fromMsg(door_closed_gripper_pos, door_closed_gripper_pos_goal);
    door_closed_gripper_pos_goal = handle_in_base_tf * door_closed_gripper_pos_goal;

    tf2::toMsg(door_closed_gripper_pos_goal, door_closed_gripper_pos);

    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "schunk_door_node");
    // ros::NodeHandle nh, pnh("~");
}