#include <manipulation_actions/Placer.h>

using std::ios;
using std::string;
using std::stringstream;
using std::vector;

Placer::Placer() :
    pnh("~"),
    tf_listener(tf_buffer),
    gripper_client("gripper_controller/gripper_action"),
    store_object_server(pnh, "store_object", boost::bind(&Placer::executeStore, this, _1), false)
{
  pnh.param<double>("default_place_height", default_place_height, 0.2);
  pnh.param<bool>("add_object", attach_arbitrary_object, false);
  pnh.param<bool>("debug", debug, true);

  object_place_pose_debug = pnh.advertise<geometry_msgs::PoseStamped>("object_place_debug", 1);
  place_pose_bin_debug = pnh.advertise<geometry_msgs::PoseStamped>("place_bin_debug", 1);
  place_pose_base_debug = pnh.advertise<geometry_msgs::PoseStamped>("place_base_debug", 1);

  attach_arbitrary_object_client =
      n.serviceClient<manipulation_actions::AttachArbitraryObject>("collision_scene_manager/attach_arbitrary_object");
  detach_objects_client = n.serviceClient<std_srvs::Empty>("collision_scene_manager/detach_objects");

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
    // add the largest arbitrary object to planning scene for testing (typically this would be done at grasp time)
    manipulation_actions::AttachArbitraryObject attach_srv;
    if (goal->challenge_object.object == manipulation_actions::ChallengeObject::BOLT)
    {
      attach_srv.request.challenge_object.object = manipulation_actions::ChallengeObject::BOLT;
    }
    else if (goal->challenge_object.object == manipulation_actions::ChallengeObject::SMALL_GEAR)
    {
      attach_srv.request.challenge_object.object = manipulation_actions::ChallengeObject::SMALL_GEAR;
    }
    else if (goal->challenge_object.object == manipulation_actions::ChallengeObject::LARGE_GEAR)
    {
      attach_srv.request.challenge_object.object = manipulation_actions::ChallengeObject::LARGE_GEAR;
    }
    else if (goal->challenge_object.object == manipulation_actions::ChallengeObject::GEARBOX_TOP)
    {
      attach_srv.request.challenge_object.object = manipulation_actions::ChallengeObject::GEARBOX_TOP;
    }
    else if (goal->challenge_object.object == manipulation_actions::ChallengeObject::GEARBOX_BOTTOM)
    {
      attach_srv.request.challenge_object.object = manipulation_actions::ChallengeObject::GEARBOX_BOTTOM;
    }

    if (!attach_arbitrary_object_client.call(attach_srv))
    {
      ROS_INFO("Could not call moveit collision scene manager service!");
    }
  }

  // TODO (enhancement): Consider multiple poses and order them based on which will be most likely to cleanly drop...
  // TODO (enhancement): ...the object (i.e. gripper pointing down)

  vector<ScoredPose> sorted_place_poses;

  geometry_msgs::PoseStamped object_pose;
  geometry_msgs::PoseStamped place_pose_bin;
  geometry_msgs::PoseStamped place_pose_base;
  object_pose.header.frame_id = "kit_frame";
  object_pose.pose.orientation.w = 1.0;
  object_pose.pose.position.z += default_place_height;

  if (goal->challenge_object.object == manipulation_actions::ChallengeObject::BOLT)
  {
    object_pose.pose.position.x += 0.05;
    object_pose.pose.position.y += 0.05;
  }
  else if (goal->challenge_object.object == manipulation_actions::ChallengeObject::SMALL_GEAR
    || goal->challenge_object.object == manipulation_actions::ChallengeObject::LARGE_GEAR)
  {
    object_pose.pose.position.x += 0.05;
    object_pose.pose.position.y -= 0.05;
  }
  else if (goal->challenge_object.object == manipulation_actions::ChallengeObject::GEARBOX_TOP
           || goal->challenge_object.object == manipulation_actions::ChallengeObject::GEARBOX_BOTTOM)
  {
    object_pose.pose.position.x -= 0.05;
  }

  geometry_msgs::TransformStamped object_to_wrist = tf_buffer.lookupTransform("object_frame", "wrist_roll_link",
                                                                              ros::Time(0), ros::Duration(1.0));
  tf2::Transform object_to_wrist_tf;
  tf2::fromMsg(object_to_wrist.transform, object_to_wrist_tf);

  for (int i = 0; i < 8; i ++)
  {
    for (int j = 0; j < 2; j ++)
    {
      geometry_msgs::PoseStamped pose_candidate;
      tf2::Transform place_object_tf;
      tf2::fromMsg(object_pose.pose, place_object_tf);

      // special case objects (large gear doesn't fit in it's compartment unless it's standing upright)
      tf2::Quaternion special_case_adjustment;
      special_case_adjustment.setRPY(0, 0, 0);
      if (goal->challenge_object.object == manipulation_actions::ChallengeObject::LARGE_GEAR)
      {
        special_case_adjustment.setRPY(0, -M_PI_2, 0);
      }

      // optional 180 degree rotation about z-axis to cover all x-axis pose alignments
      tf2::Quaternion initial_adjustment;
      initial_adjustment.setRPY(0, 0, j*M_PI);
      // rotate pose around x-axis to generate candidates (longest axis, which most constrains place)
      tf2::Quaternion adjustment;
      adjustment.setRPY(i * M_PI_4, 0, 0);
      place_object_tf.setRotation(place_object_tf.getRotation()
        * special_case_adjustment * initial_adjustment * adjustment);

      // determine wrist frame pose that will give the desired grasp
      tf2::Transform place_candidate_tf;
      place_candidate_tf = place_object_tf * object_to_wrist_tf;

      // scoring with respect to "downward pointing"
      tf2::Vector3 gravity_vector(0, 0, -1);
      tf2::Matrix3x3 rotation_mat(place_candidate_tf.getRotation());
      tf2::Vector3 x_vector(1, 0, 0);
      tf2::Vector3 pose_x_vector = rotation_mat * x_vector;

      double score = acos(pose_x_vector.dot(gravity_vector));

      pose_candidate.header.frame_id = object_pose.header.frame_id;
      pose_candidate.pose.position.x = place_candidate_tf.getOrigin().x();
      pose_candidate.pose.position.y = place_candidate_tf.getOrigin().y();
      pose_candidate.pose.position.z = place_candidate_tf.getOrigin().z();
      pose_candidate.pose.orientation = tf2::toMsg(place_candidate_tf.getRotation());

      sorted_place_poses.emplace_back(ScoredPose(pose_candidate, score));
    }
  }

  // sort poses
  sort(sorted_place_poses.begin(), sorted_place_poses.end());

  // execute best executable pose
  geometry_msgs::TransformStamped bin_to_base = tf_buffer.lookupTransform("base_link", "kit_frame",
                                                                          ros::Time(0), ros::Duration(1.0));
  bool execution_failed = true;
  for (size_t i = 0; i < sorted_place_poses.size(); i ++)
  {
    place_pose_base.header.frame_id = "base_link";
    tf2::doTransform(sorted_place_poses[i].pose, place_pose_base, bin_to_base);
    place_pose_base.header.frame_id = "base_link";

    place_pose_base_debug.publish(place_pose_base);

    ROS_INFO("Moving to place pose...");
    arm_group->setPlannerId("arm[RRTConnectkConfigDefault]");
    arm_group->setPlanningTime(2.5);
    arm_group->setStartStateToCurrentState();
//    arm_group->setJointValueTarget(place_pose_base);
    arm_group->setPoseTarget(place_pose_base);

    moveit::planning_interface::MoveItErrorCode move_result = arm_group->move();
    std::cout << "MoveIt! error code: " << move_result.val << std::endl;
    if (move_result == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      execution_failed = false;
      break;
    }
  }

  if (execution_failed)
  {
    store_object_server.setAborted(result);
    return;
  }

  //open gripper
  control_msgs::GripperCommandGoal gripper_goal;
  gripper_goal.command.position = 0.1;
  gripper_goal.command.max_effort = 200;
  gripper_client.sendGoal(gripper_goal);
  gripper_client.waitForResult(ros::Duration(5.0));

  // detach collision object
  std_srvs::Empty detach_srv;
  if (!detach_objects_client.call(detach_srv))
  {
    ROS_INFO("Could not call moveit collision scene manager service!");
  }

  store_object_server.setSucceeded(result);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "placer");

  Placer p;

  ros::spin();

  return EXIT_SUCCESS;
}
