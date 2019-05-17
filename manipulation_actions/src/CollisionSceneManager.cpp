#include <manipulation_actions/CollisionSceneManager.h>

using std::ios;
using std::string;
using std::stringstream;
using std::vector;

CollisionSceneManager::CollisionSceneManager() :
    pnh("~"),
    tf_listener(tf_buffer)
{
//  pnh.param<bool>("debug", debug, true);

  arm_group = new moveit::planning_interface::MoveGroupInterface("arm");
  arm_group->startStateMonitor();
  planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();

  touch_links.clear();
  touch_links.emplace_back("r_gripper_finger_link");
  touch_links.emplace_back("l_gripper_finger_link");
  touch_links.emplace_back("gripper_link");
  touch_links.emplace_back("wrist_roll_link");
  touch_links.emplace_back("wrist_flex_link");

  attach_simple_geometry_server = pnh.advertiseService("attach_simple_geometry", &CollisionSceneManager::attachSimpleGeometry, this);
  detach_all_server = pnh.advertiseService("detach_objects", &CollisionSceneManager::detachAllObjects, this);
  attach_arbitrary_server= pnh.advertiseService("attach_arbitrary_object", &CollisionSceneManager::attachArbitraryObject, this);
  clear_unattached_server = pnh.advertiseService("clear_unattached_objects", &CollisionSceneManager::clearAll, this);
  attach_gripper_server = pnh.advertiseService("attach_to_gripper", &CollisionSceneManager::attachGripper, this);
  attach_base_server = pnh.advertiseService("attach_to_base", &CollisionSceneManager::attachBase, this);
  detach_base_server = pnh.advertiseService("detach_all_from_base", &CollisionSceneManager::detachBase, this);
  detach_named_base_server = pnh.advertiseService("detach_from_base", &CollisionSceneManager::detachListFromBase, this);
  reattach_held_to_base_server =
      pnh.advertiseService("reattach_held_to_base", &CollisionSceneManager::reattachHeldToBase, this);
  toggle_gripper_collisions_server = pnh.advertiseService("toggle_gripper_collisions",
      &CollisionSceneManager::toggleGripperCollisions, this);
  planning_scene_client = n.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  planning_scene_publisher = n.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
//  objects_subscriber = n.subscribe("rail_segmentation/segmented_objects", 1, &CollisionSceneManager::objectsCallback, this);
}

//void CollisionSceneManager::objectsCallback(const rail_manipulation_msgs::SegmentedObjectList &msg)
//{
//  //remove previously detected collision objects
//  clearUnattachedObjects();
//
//  {
//    boost::mutex::scoped_lock lock(objects_mutex); //lock for the stored objects array
//
//    //store objects
//    object_list = msg;
//
//    if (!msg.objects.empty())
//    {
//      //add all objects to the planning scene
//      vector<moveit_msgs::CollisionObject> collision_objects;
//      collision_objects.resize(msg.objects.size());
//      for (unsigned int i = 0; i < collision_objects.size(); i++)
//      {
//        collision_objects[i] = collisionFromSegmentedObject(msg.objects[i], std::to_string(i));
//        unattached_objects.push_back(collision_objects[i].id);
//      }
//
//      planning_scene_interface->addCollisionObjects(collision_objects);
//    }
//  }
//}

bool CollisionSceneManager::attachSimpleGeometry(manipulation_actions::AttachSimpleGeometry::Request &req,
    manipulation_actions::AttachSimpleGeometry::Response &res)
{
  moveit_msgs::CollisionObject obj;
  shape_msgs::SolidPrimitive shape;

  stringstream ss;
  ss << req.name;

  if (req.shape == manipulation_actions::AttachSimpleGeometryRequest::BOX)
  {
    shape.type = shape_msgs::SolidPrimitive::BOX;
    shape.dimensions.resize(3);
    shape.dimensions[shape_msgs::SolidPrimitive::BOX_X] = req.dims[0];
    shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = req.dims[1];
    shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = req.dims[2];
  }
  else if (req.shape == manipulation_actions::AttachSimpleGeometryRequest::CYLINDER)
  {
    shape.type = shape_msgs::SolidPrimitive::CYLINDER;
    shape.dimensions.resize(3);
    shape.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = req.dims[0];
    shape.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = req.dims[1];
  }

  obj.header.frame_id = req.pose.header.frame_id;
  obj.primitives.push_back(shape);
  obj.primitive_poses.push_back(req.pose.pose);
  obj.operation = moveit_msgs::CollisionObject::ADD;

  //create collision object
  if (req.location == manipulation_actions::AttachSimpleGeometryRequest::END_EFFECTOR)
  {
    //check for name collisions
    bool collision = false;
    do
    {
      for (unsigned int i = 0; i < attached_objects.size(); i++)
      {
        if (ss.str() == attached_objects[i])
        {
          ss << "_";
          collision = true;
        }
      }
    } while (collision);
    obj.id = ss.str();
    attached_objects.push_back(obj.id);
  }
  else if (req.location == manipulation_actions::AttachSimpleGeometryRequest::BASE)
  {
    //check for name collisions
    bool collision = false;
    do
    {
      for (unsigned int i = 0; i < base_attached_objects.size(); i ++)
      {
        if (ss.str() == base_attached_objects[i])
        {
          ss << "_";
          collision = true;
        }
      }
    } while (collision);
    obj.id = ss.str();
    base_attached_objects.push_back(obj.id);
  }

  // add object to collision scene
  vector<moveit_msgs::CollisionObject> objs;
  objs.push_back(obj);

  planning_scene_interface->addCollisionObjects(objs);

  // give this time to propagate
  ros::Duration(0.25).sleep();

  if (req.location == manipulation_actions::AttachSimpleGeometryRequest::END_EFFECTOR)
  {
    if (req.use_touch_links)
    {
      arm_group->attachObject(obj.id, arm_group->getEndEffectorLink(), touch_links);
    }
    else
    {
      arm_group->attachObject(obj.id, arm_group->getEndEffectorLink());
    }
    ROS_INFO("Attached object %s to gripper", obj.id.c_str());
  }
  else if (req.location == manipulation_actions::AttachSimpleGeometryRequest::BASE)
  {
    arm_group->attachObject(obj.id, "base_link");
    ROS_INFO("Attached object %s to base_link", obj.id.c_str());
  }

  return true;
}

bool CollisionSceneManager::clearAll(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  clearUnattachedObjects();
  return true;
}

void CollisionSceneManager::clearUnattachedObjects()
{
  unattached_objects.clear();  //clear list of unattached scene object names
  vector<string> previous_objects = planning_scene_interface->getKnownObjectNames();
  if (!attached_objects.empty())
  {
    //don't remove the attached object
    for (int i = 0; i < previous_objects.size(); i ++)
    {
      for (size_t j = 0; j < attached_objects.size(); j ++)
      {
        if (previous_objects[i] == attached_objects[j])
        {
          previous_objects.erase(previous_objects.begin() + i);
          i --;
          break;
        }
      }
    }
  }
  planning_scene_interface->removeCollisionObjects(previous_objects);
}

moveit_msgs::CollisionObject CollisionSceneManager::collisionFromSegmentedObject(
    const rail_manipulation_msgs::SegmentedObject &msg, std::string suffix)
{
  moveit_msgs::CollisionObject obj;

  //create collision object
  obj.header.frame_id = msg.point_cloud.header.frame_id;
  stringstream ss;
  if (msg.recognized)
    ss << msg.name << suffix;
  else
    ss << "object" << suffix;
  //check for name collisions
  for (unsigned int i = 0; i < attached_objects.size(); i ++)
  {
    if (ss.str() == attached_objects[i])
      ss << "_";
  }
  obj.id = ss.str();


  //set object shape
  shape_msgs::SolidPrimitive bounding_volume;
  bounding_volume.type = shape_msgs::SolidPrimitive::BOX;
  bounding_volume.dimensions.resize(3);
  bounding_volume.dimensions[shape_msgs::SolidPrimitive::BOX_X] = msg.bounding_volume.dimensions.x;
  bounding_volume.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = msg.bounding_volume.dimensions.y;
  bounding_volume.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = msg.bounding_volume.dimensions.z;
  obj.primitives.push_back(bounding_volume);
  obj.primitive_poses.push_back(msg.bounding_volume.pose.pose);
  obj.operation = moveit_msgs::CollisionObject::ADD;

  return obj;
}

bool CollisionSceneManager::attachGripper(manipulation_actions::AttachToBase::Request &req,
    manipulation_actions::AttachToBase::Response &res)
{
  vector<moveit_msgs::CollisionObject> objs;
  moveit_msgs::CollisionObject obj = collisionFromSegmentedObject(req.segmented_object, "_gripper");
  attached_objects.push_back(obj.id);
  objs.push_back(obj);

  planning_scene_interface->addCollisionObjects(objs);

  // give this time to propagate
  ros::Duration(0.25).sleep();

  ROS_INFO("Attaching object to end-effector link of the robot.");

  arm_group->attachObject(obj.id, arm_group->getEndEffectorLink(), touch_links);

  return true;
}

bool CollisionSceneManager::attachBase(manipulation_actions::AttachToBase::Request &req,
    manipulation_actions::AttachToBase::Response &res)
{
  vector<moveit_msgs::CollisionObject> objs;
  moveit_msgs::CollisionObject obj = collisionFromSegmentedObject(req.segmented_object, "_base");
  base_attached_objects.push_back(obj.id);
  objs.push_back(obj);

  planning_scene_interface->addCollisionObjects(objs);

  // give this time to propagate
  ros::Duration(0.25).sleep();


  ROS_INFO("Attaching object %s to base link of the robot.", obj.id.c_str());
  arm_group->attachObject(obj.id, "base_link");

  return true;
}

bool CollisionSceneManager::detachBase(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  for (size_t i = 0; i < base_attached_objects.size(); i ++)
  {
    arm_group->detachObject(base_attached_objects[i]);
  }
  planning_scene_interface->removeCollisionObjects(base_attached_objects);
  base_attached_objects.clear();

  return true;
}

bool CollisionSceneManager::detachListFromBase(manipulation_actions::DetachFromBase::Request &req,
    manipulation_actions::DetachFromBase::Response &res)
{
  ROS_INFO("Preparing to detach objects.  The current object names attached to the base are: ");
  for (size_t i = 0; i < base_attached_objects.size(); i ++)
  {
    ROS_INFO("%s", base_attached_objects[i].c_str());
  }
  ROS_INFO("");
  ROS_INFO("You are trying to remove: ");
  for (size_t i = 0; i < req.object_names.size(); i ++)
  {
    ROS_INFO("%s", req.object_names[i].c_str());
  }
  ROS_INFO("");

  vector<string> remove_objects;
  bool all_found = true;
  for (size_t i = 0; i < req.object_names.size(); i ++)
  {
    bool found = false;
    for (size_t j = 0; j < base_attached_objects.size(); j ++)
    {
      if (req.object_names[i] == base_attached_objects[j])
      {
        ROS_INFO("Found object with name %s, detaching it...", req.object_names[i].c_str());
        ROS_INFO("Erasing object name %s from the internal list of base objects...", (*(base_attached_objects.begin() + j)).c_str());
        found = true;
        base_attached_objects.erase(base_attached_objects.begin() + j);
        remove_objects.push_back(req.object_names[i]);
        arm_group->detachObject(req.object_names[i]);
	ros::Duration(0.5).sleep();
        break;
      }
    }
    if (!found)
    {
      ROS_INFO("Could not find collision object with name %s in planning scene!", req.object_names[i].c_str());
    }
    all_found = all_found && found;
  }

  ros::Duration(0.5).sleep();

  if (!remove_objects.empty())
  {
    planning_scene_interface->removeCollisionObjects(remove_objects);
  }

  ros::Duration(0.5).sleep();

  res.result = all_found;
  return true;
}

bool CollisionSceneManager::reattachHeldToBase(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  // Iterate through the objects that are attached to the arm and attach them
  // to the base instead
  for (int i = 0; i < attached_objects.size(); i ++)
  {
    arm_group->detachObject(attached_objects[i]);

    base_attached_objects.push_back(attached_objects[i]);
    arm_group->attachObject(attached_objects[i], "base_link");

    ROS_INFO_STREAM("Scene object " << attached_objects[i] << " detached from the arm and added to base");
  }

  // Clear out the attached arm objects
  attached_objects.clear();

  return true;
}

bool CollisionSceneManager::detachAllObjects(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  for (int i = 0; i < attached_objects.size(); i ++)
  {
    arm_group->detachObject(attached_objects[i]);
  }
  planning_scene_interface->removeCollisionObjects(attached_objects);
  attached_objects.clear();

  return true;
}

bool CollisionSceneManager::attachArbitraryObject(manipulation_actions::AttachArbitraryObject::Request &req,
    manipulation_actions::AttachArbitraryObject::Response &res)
{
  // add an arbitrary object to planning scene for testing (typically this would be done at grasp time)
  vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);
  std::stringstream obj_name;
  obj_name << "arbitrary_";
  shape_msgs::SolidPrimitive shape;
  if (req.challenge_object.object == manipulation_actions::ChallengeObject::BOLT)
  {
    collision_objects[0].header.frame_id = "gripper_link";
    shape.type = shape_msgs::SolidPrimitive::SPHERE;
    shape.dimensions.resize(1);
    shape.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = 0.065;
    obj_name << "bolt";
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    collision_objects[0].primitive_poses.push_back(pose);
  }
  else if (req.challenge_object.object == manipulation_actions::ChallengeObject::SMALL_GEAR)
  {
    collision_objects[0].header.frame_id = "gripper_link";
    shape.type = shape_msgs::SolidPrimitive::SPHERE;
    shape.dimensions.resize(1);
    shape.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = 0.045;
    obj_name << "small_gear";
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    collision_objects[0].primitive_poses.push_back(pose);
  }
  else if (req.challenge_object.object == manipulation_actions::ChallengeObject::LARGE_GEAR)
  {
    collision_objects[0].header.frame_id = "gripper_link";
    shape.type = shape_msgs::SolidPrimitive::SPHERE;
    shape.dimensions.resize(1);
    shape.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = 0.115;
    obj_name << "large_gear";
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    collision_objects[0].primitive_poses.push_back(pose);
  }
  else if (req.challenge_object.object == manipulation_actions::ChallengeObject::GEARBOX_TOP)
  {
    collision_objects[0].header.frame_id = "base_link";
    shape.type = shape_msgs::SolidPrimitive::CYLINDER;
    shape.dimensions.resize(2);
    shape.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.05;
    shape.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.17;
    obj_name << "gearbox_top";
    geometry_msgs::TransformStamped gripper_pose = tf_buffer.lookupTransform("base_link", "gripper_link", ros::Time(0),
        ros::Duration(1.0));
    geometry_msgs::Pose pose;
    pose.position.x = gripper_pose.transform.translation.x;
    pose.position.y = gripper_pose.transform.translation.y;
    pose.position.z = gripper_pose.transform.translation.z;
    pose.orientation.w = 1.0;
    collision_objects[0].primitive_poses.push_back(pose);
  }
  else if (req.challenge_object.object == manipulation_actions::ChallengeObject::GEARBOX_BOTTOM)
  {
    collision_objects[0].header.frame_id = "base_link";
    shape.type = shape_msgs::SolidPrimitive::CYLINDER;
    shape.dimensions.resize(2);
    shape.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.07;
    shape.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.17;
    obj_name << "gearbox_bottom";
    geometry_msgs::TransformStamped gripper_pose = tf_buffer.lookupTransform("base_link", "gripper_link", ros::Time(0),
                                                                             ros::Duration(1.0));
    geometry_msgs::Pose pose;
    pose.position.x = gripper_pose.transform.translation.x;
    pose.position.y = gripper_pose.transform.translation.y;
    pose.position.z = gripper_pose.transform.translation.z;
    pose.orientation.w = 1.0;
    collision_objects[0].primitive_poses.push_back(pose);
  }
  else
  {
    collision_objects[0].header.frame_id = "gripper_link";
    shape.type = shape_msgs::SolidPrimitive::SPHERE;
    shape.dimensions.resize(1);
    shape.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = 0.1;
    obj_name << "object";
  }
  collision_objects[0].id = obj_name.str();
  collision_objects[0].primitives.push_back(shape);
  planning_scene_interface->addCollisionObjects(collision_objects);

  ros::Duration(0.5).sleep();

  arm_group->attachObject(obj_name.str(), "gripper_link", touch_links);
  attached_objects.push_back(obj_name.str());

  return true;
}

bool  CollisionSceneManager::toggleGripperCollisions(manipulation_actions::ToggleGripperCollisions::Request &req,
    manipulation_actions::ToggleGripperCollisions::Response &res)
{
  // Get the planning scene
  moveit_msgs::GetPlanningScene planning_scene_srv;
  planning_scene_srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

  // Set the gripper link names
  std::vector<string> gripper_names;
  gripper_names.emplace_back("gripper_link");
  gripper_names.emplace_back("l_gripper_finger_link");
  gripper_names.emplace_back("r_gripper_finger_link");

  if (!planning_scene_client.call(planning_scene_srv))
  {
    ROS_WARN("Could not get the current planning scene! Not updating gripper collisions...");
    return false;
  }
  else
  {
    // Get the collision matrix
    collision_detection::AllowedCollisionMatrix acm(planning_scene_srv.response.scene.allowed_collision_matrix);

    if (req.object_name == manipulation_actions::ToggleGripperCollisions::Request::ALL_OBJECTS_NAME)
    {
      ROS_INFO("Enabling collisions between gripper and all objects");
    }

    // Determine the list of objects to allow collisions with based on the request
    std::vector<string> collision_objects;
    if (req.object_name == manipulation_actions::ToggleGripperCollisions::Request::OCTOMAP_NAME)
    {
      collision_objects.emplace_back("<octomap>");
    }
    else
    {
      std::vector<string> known_objects = planning_scene_interface->getKnownObjectNames();
      for (size_t i = 0; i < known_objects.size(); i++)
      {
        if (known_objects[i] == req.object_name
            || req.object_name == manipulation_actions::ToggleGripperCollisions::Request::ALL_OBJECTS_NAME)
        {
          collision_objects.emplace_back(known_objects[i]);
        }
      }
    }

    // Set the ACM to the state dictated by the request
    for (size_t i = 0; i < collision_objects.size(); i++)
    {
      ROS_INFO("Collision enabled for %s", collision_objects[i].c_str());
      acm.setEntry(collision_objects[i], gripper_names, req.enable_collisions);
    }

    // Send out the planning scene update
    moveit_msgs::PlanningScene planning_scene_update;
    acm.getMessage(planning_scene_update.allowed_collision_matrix);
    planning_scene_update.is_diff = true;
    planning_scene_publisher.publish(planning_scene_update);

    // Sleep for the publish to go through
    ros::Duration(0.5).sleep();
    return true;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "collision_scene_manager");

  CollisionSceneManager csm;

  ros::spin();

  return EXIT_SUCCESS;
}
