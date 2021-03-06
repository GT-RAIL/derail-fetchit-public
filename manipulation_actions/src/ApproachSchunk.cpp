#include "manipulation_actions/ApproachSchunk.h"


ApproachSchunk::ApproachSchunk(ros::NodeHandle& nh, std::string object_frame, std::string eef_frame,
                               bool attach_arbitrary_object, float motion_speed_scale_factor, tf2::Transform template_offset) :
                               pnh_("~"),
                               approach_schunk_server_(pnh_, "approach_schunk", boost::bind(&ApproachSchunk::executeApproachSchunk, this, _1), false)
{
    nh_ = nh;
    object_frame_ = object_frame;
    eef_frame_ = eef_frame;
    attach_arbitrary_object_ = attach_arbitrary_object;
    motion_speed_scale_factor_ = motion_speed_scale_factor;
    template_offset_to_schunk_corner_ = template_offset;

    ROS_INFO("preparing schunk arm approach node");

    // preps tf
    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

    // preps moveit
    arm_group_ = new moveit::planning_interface::MoveGroupInterface("arm");
    arm_group_->startStateMonitor();

    // preps planning scene for adding collision objects as needed
    planning_scene_interface_ = new moveit::planning_interface::PlanningSceneInterface();

    attach_simple_geometry_client_ = nh_.serviceClient<manipulation_actions::AttachSimpleGeometry>("/collision_scene_manager/attach_simple_geometry");
    detach_simple_geometry_client_ = nh_.serviceClient<manipulation_actions::DetachFromBase>("/collision_scene_manager/detach_from_base");

    // approach_schunk_server_ = new actionlib::SimpleActionServer<manipulation_actions::ApproachSchunkAction>(nh, "approach_schunk", boost::bind(&ApproachSchunk::executeApproachSchunk, this, _1), false);
    approach_schunk_server_.start();
    ROS_INFO("schunk arm approach node ready!!!");
}

void ApproachSchunk::executeApproachSchunk(const manipulation_actions::ApproachSchunkGoalConstPtr& goal) {
    manipulation_actions::ApproachSchunkResult result;
    approach_frame_ = goal->approach_transform.child_frame_id;

    // adds schunk collision objects
    if (!addSchunkCollisionObjects()) {
        // aborts because attaching schunk collision objects failed
        approach_schunk_server_.setAborted(result);
        return;
    }

    // publishes static approach pose to tf
    std::vector<geometry_msgs::TransformStamped> static_poses;

    // adds octomap voxels from large gear to collision objects
    if (attach_arbitrary_object_) {
        addCollisionObject();
    }

    // updates object_frame alignment to make chuck approach easier
    geometry_msgs::TransformStamped aligned_object_to_gripper_tfS;
    if (!alignObjectFrame(aligned_object_to_gripper_tfS)) {
        // aborts because aligning object_frame to gripper failed
        removeSchunkCollisionObjects({"schunk_right_wall","schunk_back_wall","schunk_handle_wall"});
        approach_schunk_server_.setAborted(result);
        if (attach_arbitrary_object_) {
            removeCollisionObject();
        }
        return;
    }

    // updates tf with static aligned_object_frame_
    static_poses.push_back(aligned_object_to_gripper_tfS);
    static_broadcaster_.sendTransform(static_poses);

    // gets the pre_approach_gripper_pose for planning
    geometry_msgs::PoseStamped pre_approach_gripper_pose_stamped;
    if (!getGripperPreApproachPose(pre_approach_gripper_pose_stamped)) {
        // aborts because getting pre-approach pose failed
        removeSchunkCollisionObjects({"schunk_right_wall","schunk_back_wall","schunk_handle_wall"});
        approach_schunk_server_.setAborted(result);
        if (attach_arbitrary_object_) {
            removeCollisionObject();
        }
        return;
    }

    // plan and move to pre-approach eef pose
    moveit::planning_interface::MoveGroupInterface::Plan pre_approach_plan;
    if(!planToPose(pre_approach_gripper_pose_stamped,eef_frame_,"pre-approach",pre_approach_plan)) {
        if (approach_schunk_server_.isPreemptRequested()) {
            ROS_WARN("Schunk approach preempted during pre-approach planning");
            removeSchunkCollisionObjects({"schunk_right_wall","schunk_back_wall","schunk_handle_wall"});
            approach_schunk_server_.setPreempted(result);
            if (attach_arbitrary_object_) {
                removeCollisionObject();
            }
            return;
        } else {
            ROS_ERROR("Failed to plan to pre-approach pose.");
            removeSchunkCollisionObjects({"schunk_right_wall","schunk_back_wall","schunk_handle_wall"});
            approach_schunk_server_.setAborted(result);
            if (attach_arbitrary_object_) {
                removeCollisionObject();
            }
            return;
        }
    }
    ROS_INFO("Moving arm to pre-approach");
    moveit_msgs::MoveItErrorCodes error_code = arm_group_->execute(pre_approach_plan);

    // checks results
    int num_exec_fails = 0;
    while (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        if (error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED) {
            ROS_INFO("Preempted while moving to pre-approach pose.");
            removeSchunkCollisionObjects({"schunk_right_wall","schunk_back_wall","schunk_handle_wall"});
            approach_schunk_server_.setPreempted(result);
            if (attach_arbitrary_object_) {
                removeCollisionObject();
            }
            return;
        } else if (error_code.val == moveit_msgs::MoveItErrorCodes::CONTROL_FAILED) {
            ROS_INFO("Plan execution control failed, retrying.");
            // allows a couple plan execuction tries
            if (num_exec_fails > 1) {
                ROS_INFO("Failed to move to pre-approach pose.");
                ROS_ERROR("Execution error code is: %d",error_code.val);
                removeSchunkCollisionObjects({"schunk_right_wall","schunk_back_wall","schunk_handle_wall"});
                approach_schunk_server_.setAborted(result);
                if (attach_arbitrary_object_) {
                    removeCollisionObject();
                }
                return;
            }
        } else if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
            ROS_INFO("Failed to move to pre-approach pose.");
            ROS_ERROR("Execution error code is: %d",error_code.val);
            removeSchunkCollisionObjects({"schunk_right_wall","schunk_back_wall","schunk_handle_wall"});
            approach_schunk_server_.setAborted(result);
            if (attach_arbitrary_object_) {
                removeCollisionObject();
            }
            return;
        }
        num_exec_fails += 1;
        error_code = arm_group_->execute(pre_approach_plan);
    }
    ROS_INFO("Succeeded to move to pre-approach pose.");

    // gets the final_approach_gripper_pose for planning
    geometry_msgs::PoseStamped final_approach_gripper_pose_stamped;
    if (!getGripperFinalApproachPose(final_approach_gripper_pose_stamped)) {
        // aborts because getting pre-approach pose failed
        removeSchunkCollisionObjects({"schunk_right_wall","schunk_back_wall","schunk_handle_wall"});
        approach_schunk_server_.setAborted(result);
        if (attach_arbitrary_object_) {
            removeCollisionObject();
        }
        return;
    }


    // plan and move to approach eef pose
    int num_full_fails = 0;
    error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    while (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
        if(!planToPose(final_approach_gripper_pose_stamped,eef_frame_,"approach",approach_plan)) {
            if (approach_schunk_server_.isPreemptRequested()) {
                ROS_WARN("Schunk approach preempted during approach planning");
                removeSchunkCollisionObjects({"schunk_right_wall","schunk_back_wall","schunk_handle_wall"});
                approach_schunk_server_.setPreempted(result);
                if (attach_arbitrary_object_) {
                    removeCollisionObject();
                }
                return;
            } else {
                ROS_ERROR("Failed to plan to approach pose.");
                removeSchunkCollisionObjects({"schunk_right_wall","schunk_back_wall","schunk_handle_wall"});
                approach_schunk_server_.setAborted(result);
                if (attach_arbitrary_object_) {
                    removeCollisionObject();
                }
                return;
            }
        }
        ROS_INFO("Moving arm to approach");
        error_code = arm_group_->execute(approach_plan);

        num_exec_fails = 0;
        while (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
            if (error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED) {
                ROS_INFO("Preempted while moving to approach pose.");
                removeSchunkCollisionObjects({"schunk_right_wall","schunk_back_wall","schunk_handle_wall"});
                approach_schunk_server_.setPreempted(result);
                if (attach_arbitrary_object_) {
                    removeCollisionObject();
                }
                return;
            } else if (error_code.val == moveit_msgs::MoveItErrorCodes::CONTROL_FAILED) {
                ROS_INFO("Plan execution control failed. Attempt: %d", num_exec_fails);
                // allows a couple plan execuction tries
                if (num_exec_fails > 1) {
                    ROS_INFO("Failed to move to approach pose. Re-planning.");
                    break;
                }
            } else if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
                ROS_INFO("Failed to move to approach pose.");
                ROS_ERROR("Execution error code is: %d",error_code.val);
                removeSchunkCollisionObjects({"schunk_right_wall","schunk_back_wall","schunk_handle_wall"});
                approach_schunk_server_.setAborted(result);
                if (attach_arbitrary_object_) {
                    removeCollisionObject();
                }
                return;
            }
            num_exec_fails += 1;
            error_code = arm_group_->execute(approach_plan);
        }

        // allows a couple attempts at full re-planning and execution
        if (num_full_fails > 1) {
            ROS_INFO("Failed to move to approach pose.");
            ROS_ERROR("Execution error code is: %d",error_code.val);
            removeSchunkCollisionObjects({"schunk_right_wall","schunk_back_wall","schunk_handle_wall"});
            approach_schunk_server_.setAborted(result);
            if (attach_arbitrary_object_) {
                removeCollisionObject();
            }
            return;
        } else {
            num_full_fails += 1;
        }
    }

    ROS_INFO("Succeeded to move to approach pose.");
    removeSchunkCollisionObjects({"schunk_right_wall","schunk_back_wall","schunk_handle_wall"});
    if (attach_arbitrary_object_) {
        removeCollisionObject();
    }
    approach_schunk_server_.setSucceeded(result);
    return;
}

bool ApproachSchunk::addSchunkCollisionObjects() {
    // gets the schunk corner in base_link
    geometry_msgs::TransformStamped base_link_to_template_pose;
    try{
        base_link_to_template_pose = tf_buffer_.lookupTransform("base_link","template_pose",ros::Time(0),ros::Duration(1.0));
    } catch (tf2::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return false;
    }
    tf2::Transform base_link_to_template_offset;
    tf2::fromMsg(base_link_to_template_pose.transform,base_link_to_template_offset);
    tf2::Transform base_link_to_schunk_corner = base_link_to_template_offset * template_offset_to_schunk_corner_;

    // gets tf for schunk_right_wall in base_link
    tf2::Transform schunk_corner_to_schunk_right_wall;
    schunk_corner_to_schunk_right_wall.setRotation(tf2::Quaternion(0,0,0,1));
    schunk_corner_to_schunk_right_wall.setOrigin(tf2::Vector3(-0.015,0.145,0.145));
    tf2::Transform base_link_to_schunk_right_wall = base_link_to_schunk_corner * schunk_corner_to_schunk_right_wall;

    // gets tf for schunk_back_wall in base_link
    tf2::Transform schunk_corner_to_schunk_back_wall;
    schunk_corner_to_schunk_back_wall.setRotation(tf2::Quaternion(0,0,0,1));
    schunk_corner_to_schunk_back_wall.setOrigin(tf2::Vector3(0.195,-0.015,0.145));
    tf2::Transform base_link_to_schunk_back_wall = base_link_to_schunk_corner * schunk_corner_to_schunk_back_wall;

    // gets tf for schunk_handle_wall in base_link
    tf2::Transform schunk_corner_to_schunk_handle_wall;
    schunk_corner_to_schunk_handle_wall.setRotation(tf2::Quaternion(0,0,0,1));
    schunk_corner_to_schunk_handle_wall.setOrigin(tf2::Vector3(0.64,0.08,0.355));
    tf2::Transform base_link_to_schunk_handle_wall = base_link_to_schunk_corner * schunk_corner_to_schunk_handle_wall;

    // attach schunk_right_wall to base object
    tf2::Vector3 translation = base_link_to_schunk_right_wall.getOrigin();
    tf2::Quaternion orientation = base_link_to_schunk_right_wall.getRotation();
    manipulation_actions::AttachSimpleGeometry collision;
    collision.request.name = "schunk_right_wall";
    collision.request.shape = manipulation_actions::AttachSimpleGeometryRequest::BOX;
    collision.request.location = manipulation_actions::AttachSimpleGeometryRequest::BASE;
    collision.request.use_touch_links = false;
    collision.request.dims.resize(3);
    collision.request.dims[0] = 0.05;  // x
    collision.request.dims[1] = 0.36;  // y
    collision.request.dims[2] = 0.36;  // z
    collision.request.pose.header.frame_id = "base_link";
    collision.request.pose.pose.position.x = translation[0];
    collision.request.pose.pose.position.y = translation[1];
    collision.request.pose.pose.position.z = translation[2];
    collision.request.pose.pose.orientation.x = orientation[0];
    collision.request.pose.pose.orientation.y = orientation[1];
    collision.request.pose.pose.orientation.z = orientation[2];
    collision.request.pose.pose.orientation.w = orientation[3];
    if (!attach_simple_geometry_client_.call(collision)) {
        ROS_INFO("Could not call attach simple geometry client!  Aborting.");
        return false;
    }

    // attach schunk_back_wall to base object
    translation = base_link_to_schunk_back_wall.getOrigin();
    orientation = base_link_to_schunk_back_wall.getRotation();
    collision.request.name = "schunk_back_wall";
    collision.request.shape = manipulation_actions::AttachSimpleGeometryRequest::BOX;
    collision.request.location = manipulation_actions::AttachSimpleGeometryRequest::BASE;
    collision.request.use_touch_links = false;
    collision.request.dims.resize(3);
    collision.request.dims[0] = 0.45;  // x
    collision.request.dims[1] = 0.05;  // y
    collision.request.dims[2] = 0.37;  // z
    collision.request.pose.header.frame_id = "base_link";
    collision.request.pose.pose.position.x = translation[0];
    collision.request.pose.pose.position.y = translation[1];
    collision.request.pose.pose.position.z = translation[2];
    collision.request.pose.pose.orientation.x = orientation[0];
    collision.request.pose.pose.orientation.y = orientation[1];
    collision.request.pose.pose.orientation.z = orientation[2];
    collision.request.pose.pose.orientation.w = orientation[3];
    if (!attach_simple_geometry_client_.call(collision)) {
        removeSchunkCollisionObjects({"schunk_right_wall"});
        ROS_INFO("Could not call attach simple geometry client!  Aborting.");
        return false;
    }

    // attach schunk_back_wall to base object
    translation = base_link_to_schunk_handle_wall.getOrigin();
    orientation = base_link_to_schunk_handle_wall.getRotation();
    collision.request.name = "schunk_handle_wall";
    collision.request.shape = manipulation_actions::AttachSimpleGeometryRequest::BOX;
    collision.request.location = manipulation_actions::AttachSimpleGeometryRequest::BASE;
    collision.request.use_touch_links = false;
    collision.request.dims.resize(3);
    collision.request.dims[0] = 0.44;  // x
    collision.request.dims[1] = 0.22;  // y
    collision.request.dims[2] = 0.07;  // z
    collision.request.pose.header.frame_id = "base_link";
    collision.request.pose.pose.position.x = translation[0];
    collision.request.pose.pose.position.y = translation[1];
    collision.request.pose.pose.position.z = translation[2];
    collision.request.pose.pose.orientation.x = orientation[0];
    collision.request.pose.pose.orientation.y = orientation[1];
    collision.request.pose.pose.orientation.z = orientation[2];
    collision.request.pose.pose.orientation.w = orientation[3];
    if (!attach_simple_geometry_client_.call(collision)) {
        removeSchunkCollisionObjects({"schunk_right_wall","schunk_back_wall"});
        ROS_INFO("Could not call attach simple geometry client!  Aborting.");
        return false;
    }
    return true;
}

bool ApproachSchunk::removeSchunkCollisionObjects(std::vector<std::string> collision_object_names) {
    manipulation_actions::DetachFromBase detach_srv;
    detach_srv.request.object_names.resize(collision_object_names.size());
    for (unsigned i=0; i<collision_object_names.size(); i++) {
        detach_srv.request.object_names[i] = collision_object_names[i];
    }
    if (!detach_simple_geometry_client_.call(detach_srv)) {
        ROS_INFO("Could not call detach from base client!  Aborting.");
        return false;
    }
}

void ApproachSchunk::addCollisionObject(){
    // adds an arbitrary object to planning scene for testing (typically this would be done at grasp time)
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
    planning_scene_interface_->addCollisionObjects(collision_objects);

    ros::Duration(0.5).sleep();

    std::vector<std::string> touch_links;
    touch_links.emplace_back("r_gripper_finger_link");
    touch_links.emplace_back("l_gripper_finger_link");
    touch_links.emplace_back("gripper_link");
    touch_links.emplace_back("wrist_roll_link");
    touch_links.emplace_back("wrist_flex_link");
    arm_group_->attachObject("arbitrary_gripper_object", "gripper_link", touch_links);
}

void ApproachSchunk::removeCollisionObject(){
    arm_group_->detachObject("arbitrary_gripper_object");
    std::vector<std::string> obj_ids;
    obj_ids.push_back("arbitrary_gripper_object");
    planning_scene_interface_->removeCollisionObjects(obj_ids);
}

void ApproachSchunk::transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::TransformStamped& msg, ros::Time stamp, const std::string& frame_id, const std::string& child_frame_id) {
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

bool ApproachSchunk::alignObjectFrame(geometry_msgs::TransformStamped& aligned_gripper_to_object_transformStamped) {
    // tries to get tf between object_frame and gripper
    geometry_msgs::TransformStamped gripper_to_object;
    try{
        gripper_to_object = tf_buffer_.lookupTransform(eef_frame_, object_frame_,ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return false;
    }
    tf2::Transform gripper_to_object_tf;
    tf2::fromMsg(gripper_to_object.transform, gripper_to_object_tf);

    // extracts roll, pitch, yaw from current gripper_to_object_tf
    tf2::Quaternion gripper_to_object_Q = gripper_to_object_tf.getRotation();

    /* TODO buggy code, should remove
    double roll, pitch, yaw;
    tf2::Matrix3x3 mat(gripper_to_object_Q);
    mat.getRPY(roll, pitch, yaw);
      //  sets roll amount required to align object_frame
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
    */

    // iteratively computes the aligned_object_frame_tf
    tf2::Transform aligned_gripper_to_object_tf;
    aligned_gripper_to_object_tf.setOrigin(
        tf2::Vector3(gripper_to_object.transform.translation.x, gripper_to_object.transform.translation.y,
                     gripper_to_object.transform.translation.z));
    double min_offset = std::numeric_limits<double>::infinity();
    tf2::Quaternion best_roll_adjustment;
    for (size_t i = 0; i < 48; i ++)
    {
        tf2::Quaternion update_gripper_to_object_Q;
        update_gripper_to_object_Q.setRPY(i*M_PI/24, 0, 0);
        tf2::Quaternion aligned_gripper_to_object_Q = gripper_to_object_Q * update_gripper_to_object_Q;
        aligned_gripper_to_object_tf.setRotation(aligned_gripper_to_object_Q);

        // final check to see if we need to rotate by 180 degrees
        tf2::Vector3 align_vector(-1, 0, 0);
        tf2::Matrix3x3 rotation_mat(aligned_gripper_to_object_tf.getRotation());
        tf2::Vector3 z_vector(0, 0, 1);
        tf2::Vector3 new_vector = rotation_mat * z_vector;
        double angle_offset = acos(new_vector.dot(align_vector));
        if (angle_offset < min_offset)
        {
            // roll by 180 degrees
            ROS_INFO("Flipping orientation to align object_frame z-axis with gripper negative x-axis");
            best_roll_adjustment = aligned_gripper_to_object_Q;
            min_offset = angle_offset;
        }
    }
    aligned_gripper_to_object_tf.setRotation(best_roll_adjustment);

    // generates geometry_msg from tf for publishing
    transformTF2ToMsg(aligned_gripper_to_object_tf, aligned_gripper_to_object_transformStamped,ros::Time::now(),
                      eef_frame_,aligned_object_frame_);
    return true;
}

bool ApproachSchunk::getGripperPreApproachPose(geometry_msgs::PoseStamped& pre_approach_gripper_pose_stamped) {
    // gets tf between aligned_object_frame_ and gripper_link
    geometry_msgs::TransformStamped object_to_gripper;
    try{
        object_to_gripper = tf_buffer_.lookupTransform(aligned_object_frame_,eef_frame_,ros::Time(0),ros::Duration(1.0));
    } catch (tf2::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return false;
    }
    tf2::Transform object_to_gripper_tf;
    tf2::fromMsg(object_to_gripper.transform, object_to_gripper_tf);

    // gets tf between aligned_object_frame_ and arm_approach_schunk_pose
    geometry_msgs::TransformStamped approach_to_object;
    try{
        approach_to_object = tf_buffer_.lookupTransform(approach_frame_,aligned_object_frame_,ros::Time(0),ros::Duration(1.0));
    } catch (tf2::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return false;
    }

    // gets pose to align the aligned_object_frame_ to the arm_approach_schunk_pose outside of the schunk
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
    tf2::toMsg(pre_approach_gripper_pose_tf, pre_approach_gripper_pose_stamped.pose);
    pre_approach_gripper_pose_stamped.header.frame_id = approach_frame_;
    return true;
}

bool ApproachSchunk::getGripperFinalApproachPose(geometry_msgs::PoseStamped& final_approach_gripper_pose_stamped) {
    // gets tf between aligned_object_frame_ and gripper_link
    geometry_msgs::TransformStamped object_to_gripper;
    try{
        object_to_gripper = tf_buffer_.lookupTransform(aligned_object_frame_,eef_frame_,ros::Time(0),ros::Duration(1.0));
    } catch (tf2::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return false;
    }
    tf2::Transform object_to_gripper_tf;
    tf2::fromMsg(object_to_gripper.transform, object_to_gripper_tf);

    // converts from tf to geometry_msgs
    tf2::toMsg(object_to_gripper_tf, final_approach_gripper_pose_stamped.pose);
    final_approach_gripper_pose_stamped.header.frame_id = approach_frame_;
    return true;
}

bool ApproachSchunk::planToPose(geometry_msgs::PoseStamped& pose, std::string& pose_frame, std::string pose_name,
                moveit::planning_interface::MoveGroupInterface::Plan& pose_plan) {
    int max_planning_attempts = 3;
    for (int num_attempts = 0; num_attempts < max_planning_attempts; num_attempts++) {
        ROS_INFO("Planning path to %s pose. Attempt: %d/%d", pose_name.c_str(), num_attempts + 1, max_planning_attempts);

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
            return false;
        } else if (plan_result.val != moveit::planning_interface::MoveItErrorCode::SUCCESS
         && num_attempts >= max_planning_attempts - 1) {
            ROS_WARN("Moveit planning failure");
            ROS_ERROR("Planning error code is: %d",plan_result.val);
            return false;
        } else if (plan_result.val != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            continue;
        } else {
            break; // successful plan so return
        }
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "approach_schunk_node");
    ros::NodeHandle nh, pnh("~");

    std::string object_frame = "object_frame";
    std::string eef_frame = "wrist_roll_link";
    bool attach_arbitrary_object = false;
    float motion_speed_scale_factor = 0.3;
    std::string template_offset_string = "0.144 0.118 0.148 0 0 -0.785";

    pnh.getParam("object_frame", object_frame);
    pnh.getParam("eef_frame", eef_frame);
    pnh.getParam("add_object", attach_arbitrary_object);
    pnh.getParam("moveit_gain", motion_speed_scale_factor);
    pnh.getParam("template_offset_string", template_offset_string);

    // initializes a tf for the template_offset
    tf2::Transform template_offset;
    std::vector<float> offset;
    std::istringstream offset_string_stream(template_offset_string);
    for(std::string value_string; offset_string_stream >> value_string;)
        offset.push_back(std::stof(value_string));
    template_offset.setOrigin(tf2::Vector3(offset[0],offset[1],offset[2]));
    template_offset.setRotation(tf2::Quaternion(offset[4],offset[5],offset[3]));

    ApproachSchunk approach_schunk_action_server(nh,object_frame,eef_frame,attach_arbitrary_object,
                                                 motion_speed_scale_factor, template_offset.inverse());

    try{
        ros::Rate loop_rate(1000);
        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }catch(std::runtime_error& e){
        ROS_ERROR("approach_schunk_node exception: %s", e.what());
        return -1;
    }

    return 1;
}
