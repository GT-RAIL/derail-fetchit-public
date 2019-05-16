#include <manipulation_actions/SchunkDoor.h>
#include <manipulation_actions/ApproachSchunkResult.h>

SchunkDoor::SchunkDoor():
    pnh("~"),
    tf_listener(tf_buffer),
    schunk_door_server(pnh, "operate_door", boost::bind(&SchunkDoor::executeDoorAction, this, _1), false),
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
    viz_ = true;
    pnh.param("handle_width_min", handle_width_min_);
    pnh.param("handle_width_max", handle_width_max_);
    pnh.param("handle_depth_min", handle_depth_min_);
    pnh.param("handle_depth_max", handle_depth_max_);
    pnh.param("handle_height_min", handle_height_min_);
    pnh.param("handle_height_max", handle_height_max_);
    pnh.param("visualize", viz_);

    schunk_door_server.start();
    ROS_INFO("schunk door node ready!!!");

}

void SchunkDoor::executeDoorAction (const manipulation_actions::SchunkDoorGoalConstPtr &goal){
    manipulation_actions::SchunkDoorResult result;

    // runs segmentation to get the center of the handle in base_link frame. no orientation assumed
    geometry_msgs::TransformStamped base_link_to_handle_tf;
    if (!getHandleInBase(base_link_to_handle_tf)) {
        schunk_door_server.setAborted(result);
        return;
    }

    // publishes static approach pose to tf
    if (viz_) {
        std::vector<geometry_msgs::TransformStamped> static_poses;
        static_poses.push_back(base_link_to_handle_tf);
        static_broadcaster_.sendTransform(static_poses);
    }

    geometry_msgs::PoseStamped pre_approach_gripper_pose_stamped;
    geometry_msgs::Point door_operation_finish_goal;

    // get gripper preapproach pose in base link frame
    if (!getGripperPreApproachPose(pre_approach_gripper_pose_stamped, base_link_to_handle_tf, goal->open)) {
        schunk_door_server.setAborted(result);
        return;
    }

    ROS_INFO("Pre approach pose in base frame:");

    std::cout << pre_approach_gripper_pose_stamped << std::endl;

    
    // moveit call to preapproach pose above the door handle
    // TODO Debug, uncomment to move

    // moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    // planToPose(pre_approach_gripper_pose_stamped, base_frame_, approach_plan);
    // moveit_msgs::MoveItErrorCodes error_code = arm_group_->execute(approach_plan);


    // linear controller call to move down to the door handle

    manipulation_actions::LinearMoveGoal linear_goal;
    linear_goal.hold_final_pose = true;
    linear_goal.point.x = pre_approach_gripper_pose_stamped.pose.position.x;
    linear_goal.point.y = pre_approach_gripper_pose_stamped.pose.position.y;
    linear_goal.point.z = pre_approach_gripper_pose_stamped.pose.position.z - 0.2;

    // debug
    ROS_INFO("linear move down goal pose");
    std::cout << linear_goal << std::endl;


    // TODO Debug, uncomment to move

    // linear_move_client.sendGoal(linear_goal);
    // linear_move_client.waitForResult();


    // get gripper position in required to open/close the schunk door
    if (goal->open) {
        getGripperDoorOpenPos(door_operation_finish_goal, base_link_to_handle_tf); 
    } else {
        getGripperDoorClosePos(door_operation_finish_goal, base_link_to_handle_tf);
    }

    // linear controller call to close the schunk door
    linear_goal.point.x = door_operation_finish_goal.x;
    linear_goal.point.y = door_operation_finish_goal.y;
    linear_goal.point.z = door_operation_finish_goal.z;

    // debug
    std::cout << linear_goal << std::endl;


    // TODO Debug, uncomment to move

    // linear_move_client.sendGoal(linear_goal);
    // linear_move_client.waitForResult();

    // linear controller call to move arm back up, just set height += 0.2

    linear_goal.point.z += 0.2;

    // TODO Debug, uncomment to move

    // linear_move_client.sendGoal(linear_goal);
    // linear_move_client.waitForResult();

    schunk_door_server.setSucceeded(result);
    return;
}

bool SchunkDoor::getHandleInBase(geometry_msgs::TransformStamped& base_link_to_handle_tf) {
    // gets the handle orientation in base_link
    geometry_msgs::TransformStamped base_link_to_schunk;
    try{
        base_link_to_schunk = tf_buffer.lookupTransform("base_link","initial_estimate",ros::Time(0),ros::Duration(1.0));
    } catch (tf2::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return false;
    }
    tf2::Quaternion base_link_to_schunk_Q;
    tf2::fromMsg(base_link_to_schunk.transform.rotation,base_link_to_schunk_Q);

    // gets the handle position in base_link using segmentation
    geometry_msgs::Point base_link_to_handle_P;
    rail_manipulation_msgs::SegmentObjects seg_srv;
    if (!seg_client_.call(seg_srv) || seg_srv.response.segmented_objects.objects.size() == 0) {
        ROS_ERROR("Failed to segment_objects");
        return false;
    }
    ROS_INFO("Number segmented objects: %lu", seg_srv.response.segmented_objects.objects.size());

    // filters segmented objects by shape and gets handle position
    // TODOS check that the handle is above the surface
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
    base_link_to_handle_tf.header.stamp = ros::Time::now();
    base_link_to_handle_tf.header.frame_id = "base_link";
    base_link_to_handle_tf.child_frame_id = "handle_ahhhhhh";
    base_link_to_handle_tf.transform.translation.x = base_link_to_handle_P.x;
    base_link_to_handle_tf.transform.translation.y = base_link_to_handle_P.y;
    base_link_to_handle_tf.transform.translation.z = base_link_to_handle_P.z;
    base_link_to_handle_tf.transform.rotation.x = base_link_to_schunk_Q[0];
    base_link_to_handle_tf.transform.rotation.y = base_link_to_schunk_Q[1];
    base_link_to_handle_tf.transform.rotation.z = base_link_to_schunk_Q[2];
    base_link_to_handle_tf.transform.rotation.w = base_link_to_schunk_Q[3];
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
bool SchunkDoor::getGripperPreApproachPose(geometry_msgs::PoseStamped& pre_approach_gripper_pose_stamped, geometry_msgs::TransformStamped& base_link_to_handle_tf, bool open) {

    geometry_msgs::PoseStamped approach_in_handle_msg;


    // Get pose in handle_frame
    if (open){
        approach_in_handle_msg.pose.position.x = -0.05;
    } else {
        approach_in_handle_msg.pose.position.x = 0.05;
    }
    approach_in_handle_msg.pose.position.z = 0.2;
    approach_in_handle_msg.pose.orientation.w = cos(PI/4)*cos(approach_angle / 2);
    approach_in_handle_msg.pose.orientation.x = sin(PI/4)*sin(approach_angle / 2);
    approach_in_handle_msg.pose.orientation.y = cos(PI/4)*sin(approach_angle / 2);
    approach_in_handle_msg.pose.orientation.z = -sin(PI/4)*cos(approach_angle / 2);


    // Transform pose to base link (pose in, pose out, tf msg)
    tf2::doTransform(approach_in_handle_msg, pre_approach_gripper_pose_stamped, base_link_to_handle_tf);
    
    pre_approach_gripper_pose_stamped.header.frame_id = "base_link";

    return true;
}


bool SchunkDoor::getGripperDoorClosePos(geometry_msgs::Point& door_closed_gripper_pos, geometry_msgs::TransformStamped& base_link_to_handle_tf){

    tf2::Transform handle_in_base_tf;
    tf2::fromMsg(base_link_to_handle_tf.transform, handle_in_base_tf);

    // Get door closed pose in handle frame

    geometry_msgs::Point door_closed_goal_point;
    door_closed_goal_point.x = -0.3;
    door_closed_goal_point.y = 0.0;
    door_closed_goal_point.z = 0.0;

    //convert to tf::vector3
    tf2::Vector3 door_closed_gripper_pos_goal;

    tf2::fromMsg(door_closed_goal_point, door_closed_gripper_pos_goal);

    // transform to base_link
    door_closed_gripper_pos_goal = handle_in_base_tf * door_closed_gripper_pos_goal;

    // back to geometry_msgs:point msg
    tf2::toMsg(door_closed_gripper_pos_goal, door_closed_gripper_pos);

    return true;
}



bool SchunkDoor::getGripperDoorOpenPos(geometry_msgs::Point& door_opened_gripper_pos, geometry_msgs::TransformStamped& base_link_to_handle_tf){

    tf2::Transform handle_in_base_tf;
    tf2::fromMsg(base_link_to_handle_tf.transform, handle_in_base_tf);

    // Get door opened pose in handle frame

    geometry_msgs::Point door_opened_goal_point;
    door_opened_goal_point.x = 0.3;
    door_opened_goal_point.y = 0.0;
    door_opened_goal_point.z = 0.0;

    //convert to tf::vector3
    tf2::Vector3 door_opened_gripper_pos_goal;

    tf2::fromMsg(door_opened_goal_point, door_opened_gripper_pos_goal);

    // transform to base_link
    door_opened_gripper_pos_goal = handle_in_base_tf * door_opened_gripper_pos_goal;

    // back to geometry_msgs:point msg
    tf2::toMsg(door_opened_gripper_pos_goal, door_opened_gripper_pos);

    return true;
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "schunk_door_node");
    // ros::NodeHandle nh, pnh("~");
    SchunkDoor schunk_door_action_server;

    try{
        ros::Rate loop_rate(1000);
        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }catch(std::runtime_error& e){
        ROS_ERROR("schunk_door_node exception: %s", e.what());
        return -1;
    }

    return 1;
}
