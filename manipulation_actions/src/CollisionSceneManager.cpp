#include <manipulation_actions/CollisionSceneManager.h>
#include <moveit/collision_detection/collision_matrix.h>

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

  attach_closest_server = pnh.advertiseService("attach_closest_object", &CollisionSceneManager::attachClosestObject, this);
  detach_all_server = pnh.advertiseService("detach_objects", &CollisionSceneManager::detachAllObjects, this);
  attach_arbitrary_server= pnh.advertiseService("attach_arbitrary_object", &CollisionSceneManager::attachArbitraryObject, this);
  attach_base_server = pnh.advertiseService("attach_to_base", &CollisionSceneManager::attachBase, this);
  detach_base_server = pnh.advertiseService("detach_all_from_base", &CollisionSceneManager::detachBase, this);
  toggle_gripper_collisions_server = pnh.advertiseService("toggle_gripper_collisions",
      &CollisionSceneManager::toggleGripperCollisions, this);
  planning_scene_client = n.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  planning_scene_publisher = n.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
  objects_subscriber = n.subscribe("rail_segmentation/segmented_objects", 1, &CollisionSceneManager::objectsCallback, this);
}

void CollisionSceneManager::objectsCallback(const rail_manipulation_msgs::SegmentedObjectList &msg)
{
  //remove previously detected collision objects
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

  {
    boost::mutex::scoped_lock lock(objects_mutex); //lock for the stored objects array

    //store objects
    object_list = msg;

    if (!msg.objects.empty())
    {
      //add all objects to the planning scene
      vector<moveit_msgs::CollisionObject> collision_objects;
      collision_objects.resize(msg.objects.size());
      for (unsigned int i = 0; i < collision_objects.size(); i++)
      {
        collision_objects[i] = collisionFromSegmentedObject(msg.objects[i], std::to_string(i));
        unattached_objects.push_back(collision_objects[i].id);
      }

      planning_scene_interface->addCollisionObjects(collision_objects);
    }
  }
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

bool CollisionSceneManager::attachBase(manipulation_actions::AttachToBase::Request &req,
    manipulation_actions::AttachToBase::Response &res)
{
  vector<moveit_msgs::CollisionObject> objs;
  moveit_msgs::CollisionObject obj = collisionFromSegmentedObject(req.segmented_object, "_base");
  base_attached_objects.push_back(obj.id);
  objs.push_back(obj);

  planning_scene_interface->addCollisionObjects(objs);

  // note: we don't actually have to "attach" the objects, as they're in the base frame and will move with the robot
  // as if they're attached

  return true;
}

bool CollisionSceneManager::detachBase(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  planning_scene_interface->removeCollisionObjects(base_attached_objects);
  base_attached_objects.clear();

  return true;
}

bool CollisionSceneManager::attachClosestObject(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  boost::mutex::scoped_lock lock(objects_mutex);  //lock for the stored objects array

  //find the closest point to the gripper pose
  size_t closest = 0;
  if (object_list.objects.size() == 0)
  {
    ROS_INFO("No scene objects to attach.");
    return true;
  }
  else if (object_list.objects.size() > 1)
  {
    // find the closest point
    double min = std::numeric_limits<double>::infinity();
    // check each segmented object
    for (size_t i = 0; i < object_list.objects.size(); i++)
    {
      geometry_msgs::TransformStamped eef_transform = tf_buffer.lookupTransform(
          object_list.objects[i].point_cloud.header.frame_id, "gripper_link", ros::Time(0));
      geometry_msgs::Vector3 &v = eef_transform.transform.translation;
      //convert PointCloud2 to PointCloud to access the data easily
      sensor_msgs::PointCloud cloud;
      sensor_msgs::convertPointCloud2ToPointCloud(object_list.objects[i].point_cloud, cloud);
      // check each point in the cloud
      for (size_t j = 0; j < cloud.points.size(); j++)
      {
        // euclidean distance to the point
        double dist_sqr = pow(cloud.points[j].x - v.x, 2) + pow(cloud.points[j].y - v.y, 2)
            + pow(cloud.points[j].z - v.z, 2);
        if (dist_sqr < min)
        {
          min = dist_sqr;
          closest = i;
        }
      }
    }

    if (min > SCENE_OBJECT_DST_SQR_THRESHOLD)
    {
      ROS_INFO("No scene objects are close enough to the end effector to be attached.");
      return true;
    }
  }

  ROS_INFO("Attaching scene object %lu to gripper.", closest);
  vector<string> touch_links;
  touch_links.emplace_back("r_gripper_finger_link");
  touch_links.emplace_back("l_gripper_finger_link");
  touch_links.emplace_back("gripper_link");
  touch_links.emplace_back("wrist_roll_link");
  touch_links.emplace_back("wrist_flex_link");
  arm_group->attachObject(unattached_objects[closest], arm_group->getEndEffectorLink(), touch_links);
  attached_objects.push_back(unattached_objects[closest]);
  unattached_objects.erase(unattached_objects.begin() + closest);

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
  collision_objects[0].header.frame_id = "gripper_link";
  std::stringstream obj_name;
  obj_name << "arbitrary_";
  shape_msgs::SolidPrimitive shape;
  shape.type = shape_msgs::SolidPrimitive::SPHERE;
  shape.dimensions.resize(1);
  if (req.challenge_object.object == manipulation_actions::ChallengeObject::BOLT)
  {
    shape.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = 0.065;
    obj_name << "bolt";
  }
  else if (req.challenge_object.object == manipulation_actions::ChallengeObject::SMALL_GEAR)
  {
    shape.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = 0.045;
    obj_name << "small_gear";
  }
  else if (req.challenge_object.object == manipulation_actions::ChallengeObject::LARGE_GEAR)
  {
    shape.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = 0.115;
    obj_name << "large_gear";
  }
  else if (req.challenge_object.object == manipulation_actions::ChallengeObject::GEARBOX_TOP)
  {
    shape.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = 0.175;
    obj_name << "gearbox_top";
  }
  else if (req.challenge_object.object == manipulation_actions::ChallengeObject::GEARBOX_BOTTOM)
  {
    shape.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = 0.175;
    obj_name << "gearbox_bottom";
  }
  else
  {
    shape.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = 0.15;
    obj_name << "object";
  }
  collision_objects[0].id = obj_name.str();
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
