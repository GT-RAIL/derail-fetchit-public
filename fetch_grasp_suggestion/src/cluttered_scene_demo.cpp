#include <fetch_grasp_suggestion/cluttered_scene_demo.h>

using std::string;

ClutteredSceneDemo::ClutteredSceneDemo() :
    pnh_("~"),
    cloud_(new pcl::PointCloud<pcl::PointXYZRGB>),
    execute_grasp_client_("/executor/execute_grasp"),
    prepare_robot_client_("/executor/prepare_robot"),
    drop_position_client_("/executor/drop_position"),
    gripper_client_("gripper_controller/gripper_action")
{
  string cloud_topic;
  pnh_.param<string>("cloud_topic", cloud_topic, "head_camera/depth_registered/points");
  pnh_.param<bool>("debug", debug_, true);

  if (debug_)
  {
    debug_publisher_ = pnh_.advertise<geometry_msgs::PoseStamped>("pose_to_execute", 1);
    cloud_publisher_ = pnh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("scene_cloud", 1);
  }

  run_subscriber_ = pnh_.subscribe("run_demo", 1, &ClutteredSceneDemo::runCallback, this);
  cloud_subscriber_ = n_.subscribe(cloud_topic, 1, &ClutteredSceneDemo::cloudCallback, this);

  suggest_grasps_client_ = n_.serviceClient<fetch_grasp_suggestion::SuggestGrasps>("suggester/suggest_grasps_scene");
  rank_grasps_client_ = n_.serviceClient<fetch_grasp_suggestion::PairwiseRank>("suggester/pairwise_rank_scene");
  drop_object_client_ = n_.serviceClient<std_srvs::Empty>("executor/drop_object");
}

void ClutteredSceneDemo::runCallback(const std_msgs::Empty &msg)
{
  if (cloud_->empty())
  {
    ROS_INFO("No point cloud data received yet!");
    return;
  }

  while (true)
  {
    //prepare for grasping
    fetch_grasp_suggestion::PresetMoveGoal preset_move;
    prepare_robot_client_.sendGoal(preset_move);
    prepare_robot_client_.waitForResult(ros::Duration(20.0));
    if (!prepare_robot_client_.getResult()->success)
    {
      ROS_INFO("Failed to reset arm.  Ending demo.");
      return;
    }

    // give head shake time to settle
    ros::Rate sleep_rate(30);
    for (int i = 0; i < 150; i ++)
    {
      sleep_rate.sleep();
      ros::spinOnce();
    }

    geometry_msgs::PoseArray poseList;
    {
      boost::mutex::scoped_lock lock(cloud_mutex_);

      // crop cloud to just the center area
      // TODO (enhancement): the workspace for the demo is hardcoded
      // get sensor_msgs version of cloud for transforming
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      PointCloudManipulation::transformPointCloud(cloud_, base_cloud, "base_link", tf_listener_);

      // crop excess environment
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::CropBox<pcl::PointXYZRGB> crop_box;
      Eigen::Vector4f min_point, max_point;
      min_point[0] = 0.4f;
      min_point[1] = -0.5f;
      min_point[2] = 0.5f;
      max_point[0] = 0.8f;
      max_point[1] = 0.7f;
      max_point[2] = 1.5f;
      crop_box.setMin(min_point);
      crop_box.setMax(max_point);
      crop_box.setInputCloud(base_cloud);
      crop_box.filter(*cropped_cloud);

      // detect table
      pcl::SACSegmentation<pcl::PointXYZRGB> plane_seg;
      pcl::ModelCoefficients::Ptr table_coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices);
      // segmentation parameters
      plane_seg.setOptimizeCoefficients(true);
      plane_seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
      plane_seg.setAxis(Eigen::Vector3f(0, 0, 1));
      plane_seg.setEpsAngle(0.15);
      plane_seg.setMethodType(pcl::SAC_RANSAC);
      plane_seg.setMaxIterations(100);
      plane_seg.setDistanceThreshold(0.01);
      // segmentation
      plane_seg.setInputCloud(cropped_cloud);
      plane_seg.segment(*table_inliers, *table_coefficients);
      // extract table
      pcl::ExtractIndices<pcl::PointXYZRGB> extract_table;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      extract_table.setInputCloud(cropped_cloud);
      extract_table.setIndices(table_inliers);
      extract_table.setNegative(false);
      extract_table.filter(*table_cloud);
      // get table height
      pcl::PointXYZRGB min_table, max_table;
      pcl::getMinMax3D(*table_cloud, min_table, max_table);

      // re-crop above table
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      min_point[2] = max_table.z + .001f;
      crop_box.setMin(min_point);
      crop_box.filter(*table_cropped_cloud);

      // transform cropped clouds back to original frame
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      PointCloudManipulation::transformPointCloud(table_cropped_cloud, objects_cloud, cloud_->header.frame_id,
                                                  tf_listener_);
      PointCloudManipulation::transformPointCloud(cropped_cloud, scene_cloud, cloud_->header.frame_id, tf_listener_);

      // get bounds of "objects" cloud
      pcl::PointXYZRGB min_objects, max_objects;
      pcl::getMinMax3D(*objects_cloud, min_objects, max_objects);

      // pad bounds
      float roi_padding = 0.03f;
      min_point[0] = min_objects.x - roi_padding;
      min_point[1] = min_objects.y - roi_padding;
      min_point[2] = min_objects.z - roi_padding;
      max_point[0] = max_objects.x + roi_padding;
      max_point[1] = max_objects.y + roi_padding;
      max_point[2] = max_objects.z + roi_padding;

      // crop scene cloud to padded bounds
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      crop_box.setMin(min_point);
      crop_box.setMax(max_point);
      crop_box.setInputCloud(scene_cloud);
      crop_box.filter(*roi_cloud);

      cloud_publisher_.publish(roi_cloud);

      //transform back to original frame
      fetch_grasp_suggestion::SuggestGrasps suggest_grasps;
      PointCloudManipulation::toSensorMsgs(roi_cloud, suggest_grasps.request.cloud);

      // get grasp suggestions
      if (!suggest_grasps_client_.call(suggest_grasps))
      {
        ROS_INFO("Call to suggest grasps service failed!");
        return;
      }
      if (suggest_grasps.response.grasp_list.poses.empty())
      {
        ROS_INFO("No grasp suggestions found, stopping execution.");
        return;
      }
      ROS_INFO("Received %lu grasp suggestions.", suggest_grasps.response.grasp_list.poses.size());

      // rank grasp suggestions
      fetch_grasp_suggestion::PairwiseRank pairwise_rank;
      if (!rank_grasps_client_.call(pairwise_rank))
      {
        ROS_INFO("Call to pairwise ranking failed!");
        return;
      }
      poseList = pairwise_rank.response.grasp_list;
      ROS_INFO("Ranked %lu grasps.", poseList.poses.size());

      if (poseList.poses.empty())
      {
        ROS_INFO("Ranking returned no grasps, stoppine execution.");
        return;
      }
    }

    // execute best grasp
    fetch_grasp_suggestion::ExecuteGraspGoal grasp_goal;
    grasp_goal.index = -1;  //signify no-object grasping mode
    grasp_goal.grasp_pose.header.frame_id = poseList.header.frame_id;
    //for (size_t i = 0; i < poseList.poses.size(); i ++)
    size_t i = 0;
    bool finished = false;
    while (!finished)
    {
      ROS_INFO("Showing grasp %lu...", i);

      grasp_goal.grasp_pose.pose = poseList.poses[i];
      if (debug_)
      {
        debug_publisher_.publish(grasp_goal.grasp_pose);
      }

      ROS_INFO("Enter a command (e: execute; q: quit; [ or ]: back or forward): ");
      string input;
      std::cin >> input;

      switch (input[0])
      {
        case ']':
          if (i < poseList.poses.size() - 1)
          {
            i++;
          }
          break;

        case '[':
          if (i > 0)
          {
            i--;
          }
          break;

        case 'q':
          ROS_INFO("Quit requested, ending demo.");
          return;

        case 'e':
          execute_grasp_client_.sendGoal(grasp_goal);
          execute_grasp_client_.waitForResult(ros::Duration(60));
          fetch_grasp_suggestion::ExecuteGraspResultConstPtr execute_grasp_result = execute_grasp_client_.getResult();

          if (execute_grasp_result->success)
          {
            ROS_INFO("Grasp execution complete!");
            finished = true;
          }
          if (i < poseList.poses.size() - 1)
          {
            i++;
          }
          break;
      }

      ros::spinOnce();
      sleep_rate.sleep();
    }

    // move to drop position
    ROS_INFO("Moving object to drop area...");
    drop_position_client_.sendGoal(preset_move);
    drop_position_client_.waitForResult(ros::Duration(20));
    if (!drop_position_client_.getResult()->success)
    {
      ROS_INFO("Planning failed, retrying...");
      drop_position_client_.sendGoal(preset_move);
      drop_position_client_.waitForResult(ros::Duration(20));
      if (!drop_position_client_.getResult()->success)
      {
        ROS_INFO("Could not move to drop area.  Ending demo.");
        return;
      }
    }

    ROS_INFO("Dropping object.");
    control_msgs::GripperCommandGoal gripper_goal;
    gripper_goal.command.position = 0.1;
    gripper_goal.command.max_effort = 200;
    gripper_client_.sendGoal(gripper_goal);
    gripper_client_.waitForResult(ros::Duration(5.0));

    // wait for object to fall out of gripper
    for (int j = 0; j < 90; j ++)
    {
      sleep_rate.sleep();
      ros::spinOnce();
    }
  }
}

void ClutteredSceneDemo::cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
  boost::mutex::scoped_lock lock(cloud_mutex_);

  *cloud_ = *cloud;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cluttered_scene_demo");

  ClutteredSceneDemo csd;

  ros::spin();

  return EXIT_SUCCESS;
}
