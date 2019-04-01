#include <manipulation_actions/Placer.h>

using std::ios;
using std::string;
using std::stringstream;
using std::vector;

Placer::Placer() :
    pnh("~"),
    tf_listener(tf_buffer),
    store_object_server(pnh, "store_object", boost::bind(&Placer::executeStore, this, _1), false)
{
  pnh.param<double>("default_place_height", default_place_height, 0.2);
  pnh.param<bool>("add_object", attach_arbitrary_object, false);
  pnh.param<bool>("debug", debug, true);

  object_place_pose_debug = pnh.advertise<geometry_msgs::PoseStamped>("object_place_debug", 1);
  place_pose_bin_debug = pnh.advertise<geometry_msgs::PoseStamped>("place_bin_debug", 1);
  place_pose_base_debug = pnh.advertise<geometry_msgs::PoseStamped>("place_base_debug", 1);

  arm_group = new moveit::planning_interface::MoveGroupInterface("arm");
  arm_group->startStateMonitor();

  planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();

  store_object_server.start();
}

void Placer::executeStore(const manipulation_actions::StoreObjectGoalConstPtr &goal)
{
  manipulation_actions::StoreObjectResult result;

  if (attach_arbitrary_object)
  {
    // add an arbitrary object to planning scene for testing (typically this would be done at grasp time)
    vector<moveit_msgs::CollisionObject> collision_objects;
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

    vector<string> touch_links;
    touch_links.emplace_back("r_gripper_finger_link");
    touch_links.emplace_back("l_gripper_finger_link");
    touch_links.emplace_back("gripper_link");
    touch_links.emplace_back("wrist_roll_link");
    touch_links.emplace_back("wrist_flex_link");
    arm_group->attachObject("arbitrary_gripper_object", "gripper_link", touch_links);
  }

  // TODO (enhancement): Consider multiple poses and order them based on which will be most likely to cleanly drop...
  // TODO (enhancement): ...the object (i.e. gripper pointing down)

  geometry_msgs::PoseStamped object_pose;
  geometry_msgs::PoseStamped place_pose_bin;
  geometry_msgs::PoseStamped place_pose_base;
  object_pose.header.frame_id = "active_bin_frame";
  object_pose.pose.orientation.w = 1.0;
  object_pose.pose.position.z += default_place_height;

  if (goal->object == manipulation_actions::StoreObjectGoal::BOLT)
  {
    object_pose.pose.position.x += 0.05;
    object_pose.pose.position.y += 0.05;
  }
  else if (goal->object == manipulation_actions::StoreObjectGoal::SMALL_GEAR
    || goal->object == manipulation_actions::StoreObjectGoal::LARGE_GEAR)
  {
    object_pose.pose.position.x -= 0.05;
    object_pose.pose.position.y += 0.05;
  }
  else if (goal->object == manipulation_actions::StoreObjectGoal::GEARBOX_TOP
           || goal->object == manipulation_actions::StoreObjectGoal::GEARBOX_BOTTOM)
  {
    object_pose.pose.position.y -= 0.05;
  }

  object_place_pose_debug.publish(object_pose);

  geometry_msgs::TransformStamped object_to_wrist = tf_buffer.lookupTransform("object_frame", "wrist_roll_link",
                                                                                ros::Time(0), ros::Duration(1.0));
  place_pose_bin.header.frame_id = "active_bin_frame";

//  Eigen::Affine3d transform_matrix;
//  tf::transformMsgToEigen(object_to_wrist.transform, transform_matrix);
//
//  Eigen::Vector3d transform_pose_matrix, transformed_pose_matrix;
//  transform_point[0] = distance;
//  transform_point[1] = 0;
//  transform_point[2] = 0;
//  transformed_point = transform_matrix*transform_point;
//  tf::pointEigenToMsg(transformed_point, result.position);

//  tf2::doTransform(object_pose, place_pose_bin, object_to_wrist);
//  place_pose_bin.header.frame_id = "active_bin_frame";

  tf2::Transform object_to_wrist_tf;
  tf2::fromMsg(object_to_wrist.transform, object_to_wrist_tf);
  tf2::Transform object_pose_tf;
  tf2::fromMsg(object_pose.pose, object_pose_tf);
  tf2::Transform place_pose_bin_tf;
  place_pose_bin_tf = object_pose_tf*object_to_wrist_tf;
  place_pose_bin.pose.position.x = place_pose_bin_tf.getOrigin().x();
  place_pose_bin.pose.position.y = place_pose_bin_tf.getOrigin().y();
  place_pose_bin.pose.position.z = place_pose_bin_tf.getOrigin().z();
  place_pose_bin.pose.orientation.x = place_pose_bin_tf.getRotation().x();
  place_pose_bin.pose.orientation.y = place_pose_bin_tf.getRotation().y();
  place_pose_bin.pose.orientation.z = place_pose_bin_tf.getRotation().z();
  place_pose_bin.pose.orientation.w = place_pose_bin_tf.getRotation().w();

  place_pose_bin_debug.publish(place_pose_bin);

  geometry_msgs::TransformStamped bin_to_base = tf_buffer.lookupTransform("base_link", "active_bin_frame",
                                                                                ros::Time(0), ros::Duration(1.0));
  place_pose_base.header.frame_id = "base_link";
  tf2::doTransform(place_pose_bin, place_pose_base, bin_to_base);
  place_pose_base.header.frame_id = "base_link";

  place_pose_base_debug.publish(place_pose_base);

  ROS_INFO("Moving to place pose...");
  arm_group->setPlannerId("arm[RRTConnectkConfigDefault]");
  arm_group->setPlanningTime(5.0);
  arm_group->setStartStateToCurrentState();
  arm_group->setJointValueTarget(place_pose_base);

  moveit::planning_interface::MoveItErrorCode move_result = arm_group->move();
  if (move_result != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    store_object_server.setAborted(result);
    if (attach_arbitrary_object)
    {
      arm_group->detachObject("arbitrary_gripper_object");
      vector<string> obj_ids;
      obj_ids.push_back("arbitrary_gripper_object");
      planning_scene_interface->removeCollisionObjects(obj_ids);
    }
    return;
  }

  if (attach_arbitrary_object)
  {
    arm_group->detachObject("arbitrary_gripper_object");
    vector<string> obj_ids;
    obj_ids.push_back("arbitrary_gripper_object");
    planning_scene_interface->removeCollisionObjects(obj_ids);
  }

  // TODO: open gripper
  store_object_server.setSucceeded(result);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "placer");

  Placer p;

  ros::spin();

  return EXIT_SUCCESS;
}
