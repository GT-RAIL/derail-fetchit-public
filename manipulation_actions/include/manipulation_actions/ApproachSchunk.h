#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <boost/thread/mutex.hpp>
#include <math.h>

#include "manipulation_actions/ApproachSchunkAction.h"
#include "manipulation_actions/AttachSimpleGeometry.h"
#include "manipulation_actions/AttachSimpleGeometry.h""

class ApproachSchunk {
    public:
        ApproachSchunk(ros::NodeHandle& nh, std::string object_frame, std::string eef_frame, bool attach_arbitrary_object, float motion_speed_scale_factor);
        // approach schunk action server
        void executeApproachSchunk( const manipulation_actions::ApproachSchunkGoalConstPtr& goal);

    private:
        // attaches schunk collision objects
        bool addSchunkCollisionObjects();
        // removes schunk collision objects
        bool removeSchunkCollisionObjects(std::string collision_object_name);
        // adds collision objects to arm for planning
        void addCollisionObject();
        // removes collision objects from arm for planning
        void removeCollisionObject();
        // easy way to transform a tf2 to a geometry_msg
        void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::TransformStamped& msg, ros::Time stamp, const std::string& frame_id, const std::string& child_frame_id);
        // aligns the object_frame z with the eef frame -x (this makes the planning problem much easier and increases the likelihood of success)
        bool alignObjectFrame(geometry_msgs::TransformStamped& aligned_gripper_to_object_transformStamped);
        // calculates a pre-approach schunk eef pose
        bool getGripperPreApproachPose(geometry_msgs::PoseStamped& pre_approach_gripper_pose_stamped);
        // calculates a final approach schunk eef pose
        bool getGripperFinalApproachPose(geometry_msgs::PoseStamped& final_approach_gripper_pose_stamped);
        // makes 3 attempts to plan to the pose goal
        bool planToPose(geometry_msgs::PoseStamped& pose, std::string& pose_frame, std::string pose_name,moveit::planning_interface::MoveGroupInterface::Plan& pose_plan);

        ros::NodeHandle nh_, pnh_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener* tf_listener_;
        tf2_ros::StaticTransformBroadcaster static_broadcaster_;
        moveit::planning_interface::MoveGroupInterface* arm_group_;
        moveit::planning_interface::PlanningSceneInterface* planning_scene_interface_;
        bool attach_arbitrary_object_;
        float motion_speed_scale_factor_;
        actionlib::SimpleActionServer<manipulation_actions::ApproachSchunkAction> approach_schunk_server_;
        std::string object_frame_ = "object_frame";
        std::string aligned_object_frame_ = "aligned_object_frame";
        std::string eef_frame_ = "gripper_link";
        std::string approach_frame_ = "template_frame";
        ros::ServiceClient attach_simple_geometry_client_;

        // schunk collision object related
        tf2::Transform template_offset_to_schunk_corner_;

};
