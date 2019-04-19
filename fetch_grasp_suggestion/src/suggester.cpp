#include <fetch_grasp_suggestion/suggester.h>

using std::ios;
using std::string;
using std::stringstream;
using std::vector;

Suggester::Suggester() :
    pnh_("~"),
    cloud_(new pcl::PointCloud<pcl::PointXYZRGB>),
    sample_grasps_client_("/rail_agile/sample_grasps"),
    sample_grasps_baseline_client_("/rail_agile/sample_classify_grasps"),
    rank_grasps_object_client_("/grasp_sampler/rank_grasps_object"),
    rank_grasps_scene_client_("/grasp_sampler/rank_grasps_scene"),
    suggest_grasps_server_(pnh_, "get_grasp_suggestions", boost::bind(&Suggester::getGraspSuggestions, this, _1), false)
{
  string segmentation_topic, cloud_topic;
  pnh_.param<string>("segmentation_topic", segmentation_topic, "rail_segmentation/segmented_objects");
  pnh_.param<string>("cloud_topic", cloud_topic, "head_camera/depth_registered/points");
  pnh_.param<string>("file_name", filename_, "grasp_data");
  pnh_.param<double>("min_grasp_depth", min_grasp_depth_, -0.03);
  pnh_.param<double>("max_grasp_depth", max_grasp_depth_, 0.03);

  stringstream ss;
  ss << filename_ << ".csv";
  filename_ = ss.str();

  cloud_received_ = false;

  clear_objects_client_ = n_.serviceClient<std_srvs::Empty>("executor/clear_objects");
  add_object_client_ = n_.serviceClient<fetch_grasp_suggestion::AddObject>("executor/add_object");
  classify_all_client_ = n_.serviceClient<fetch_grasp_suggestion::ClassifyAll>("classify_all");

  grasps_publisher_ = pnh_.advertise<fetch_grasp_suggestion::RankedGraspList>("grasps", 1);
  cloud_subscriber_ = n_.subscribe(cloud_topic, 1, &Suggester::cloudCallback, this);
  objects_subscriber_ = n_.subscribe(segmentation_topic, 1, &Suggester::objectsCallback, this);
  grasp_feedback_subscriber_ = pnh_.subscribe("grasp_feedback", 1, &Suggester::graspFeedbackCallback, this);

  suggest_grasps_service_ = pnh_.advertiseService("suggest_grasps", &Suggester::suggestGraspsCallback, this);
  suggest_grasps_baseline_service_ = pnh_.advertiseService("suggest_grasps_baseline",
                                                           &Suggester::suggestGraspsAgileCallback, this);
  suggest_grasps_random_service_ = pnh_.advertiseService("suggest_grasps_random",
                                                         &Suggester::suggestGraspsRandomCallback, this);
  suggest_grasps_scene_service_ = pnh_.advertiseService("suggest_grasps_scene",
                                                        &Suggester::suggestGraspsSceneCallback, this);
  pairwise_rank_service_ = pnh_.advertiseService("pairwise_rank", &Suggester::pairwiseRankCallback, this);
  pairwise_rank_scene_service_ = pnh_.advertiseService("pairwise_rank_scene",
                                                       &Suggester::pairwiseRankSceneCallback, this);

  suggest_grasps_server_.start();
}

bool Suggester::pairwiseRankCallback(rail_manipulation_msgs::PairwiseRank::Request &req,
    rail_manipulation_msgs::PairwiseRank::Response &res)
{
  boost::mutex::scoped_lock lock(stored_grasp_mutex_);

  if (stored_grasp_list_.grasps.empty())
  {
    ROS_INFO("No grasps to rank!");
    return true;
  }

  //calculate object features if they haven't been calculated yet
  if (object_features_.empty())
  {
    object_features_ = Common::calculateObjectFeatures(stored_object_cloud_);
  }

  struct timeval start;
  gettimeofday(&start, NULL);
  unsigned long long start_time = start.tv_usec + (unsigned long long)start.tv_sec * 1000000;

  fetch_grasp_suggestion::ClassifyAll classify;
  classify.request.object_features = object_features_;
  classify.request.grasp_list = stored_grasp_list_;
  if (!classify_all_client_.call(classify))
  {
    ROS_INFO("Failed to call classify all service!");
    return false;
  }

  struct timeval end;
  gettimeofday(&end, NULL);
  unsigned long long end_time = end.tv_usec + (unsigned long long)end.tv_sec * 1000000;
  std::cout << "Classifier runtime:" << std::endl;
  std::cout << end_time - start_time << std::endl;

  res.grasp_list = classify.response.grasp_list;

  return true;
}

bool Suggester::pairwiseRankSceneCallback(rail_manipulation_msgs::PairwiseRank::Request &req,
    rail_manipulation_msgs::PairwiseRank::Response &res)
{
  boost::mutex::scoped_lock lock(stored_grasp_mutex_);

  if (stored_grasp_list_.grasps.empty())
  {
    ROS_INFO("No grasps to rank!");
    return true;
  }

  fetch_grasp_suggestion::ClassifyAll classify;
  classify.request.grasp_list.grasps.resize(stored_grasp_list_.grasps.size());
  classify.request.object_features.clear();

  //calculate local features at each grasp point
  for (size_t i = 0; i < classify.request.grasp_list.grasps.size(); i ++)
  {
    classify.request.grasp_list.grasps[i].pose = stored_grasp_list_.grasps[i].pose;
    classify.request.grasp_list.grasps[i].heuristics =
        Common::calculateLocalFeatures(stored_scene_cloud_, stored_grasp_list_.grasps[i].pose.pose.position);
    for (size_t j = 0; j < stored_grasp_list_.grasps[i].heuristics.size(); j ++)
    {
      classify.request.grasp_list.grasps[i].heuristics.push_back(stored_grasp_list_.grasps[i].heuristics[j]);
    }
  }

  if (!classify_all_client_.call(classify))
  {
    ROS_INFO("Failed to call classify all service!");
    return false;
  }

  res.grasp_list = classify.response.grasp_list;

  return true;
}

void Suggester::graspFeedbackCallback(const rail_manipulation_msgs::GraspFeedback &grasp_feedback)
{
  boost::mutex::scoped_lock lock(stored_grasp_mutex_);

  //calculate object features if they haven't been calculated yet
  if (object_features_.empty())
  {
    object_features_ = Common::calculateObjectFeatures(stored_object_cloud_);
  }

  //do nothing if this grasp has already been selected (prevent duplicate training instances)
  if (find(selected_grasps_.begin(), selected_grasps_.end(), grasp_feedback.index_selected) != selected_grasps_.end())
  {
    return;
  }

  //generate a positive and negative example for each pair of the selected grasp and any seen but unselected grasps
  file_.open(filename_.c_str(), ios::out | ios::app);
  for (int i = 0; i <= grasp_feedback.indices_considered.size(); i ++)
  {
    if (i != grasp_feedback.index_selected
        && find(selected_grasps_.begin(),selected_grasps_.end(), i) == selected_grasps_.end())
    {
      //log positive training example
      file_ << Common::createTrainingInstance(object_features_,
                                              stored_grasp_list_.grasps[grasp_feedback.index_selected].heuristics,
                                              stored_grasp_list_.grasps[i].heuristics, true) << "\n";

      //log negative training example
      file_ << Common::createTrainingInstance(object_features_,
                                              stored_grasp_list_.grasps[grasp_feedback.index_selected].heuristics,
                                              stored_grasp_list_.grasps[i].heuristics, false) << "\n";
    }
  }
  file_.close();

  selected_grasps_.push_back(grasp_feedback.index_selected);
}

bool Suggester::suggestGraspsCallback(rail_manipulation_msgs::SuggestGrasps::Request &req,
    rail_manipulation_msgs::SuggestGrasps::Response &res)
{
  boost::mutex::scoped_lock cloud_lock(cloud_mutex_);
  boost::mutex::scoped_lock grasp_lock(stored_grasp_mutex_);

  stored_grasp_list_.grasps.clear();
  stored_grasp_list_.object_index = 0;
  object_features_.clear();
  selected_grasps_.clear();

  stored_object_cloud_ = req.cloud;

  //check that we have all required data
  if (!cloud_received_)
  {
    ROS_INFO("No point cloud data received!");
    return false;
  }

  //save frames for lots of upcoming point cloud transforming
  string environment_source_frame = cloud_->header.frame_id;
  string object_source_frame = stored_object_cloud_.header.frame_id;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  geometry_msgs::PoseArray sampled_grasps;
  SampleGraspCandidates(stored_object_cloud_, object_source_frame, environment_source_frame,
                        sampled_grasps, cropped_cloud);

  ROS_INFO("Sampling complete.");

  if (!sampled_grasps.poses.empty())
  {
    ROS_INFO("Ranking grasp candidates...");
    rankCandidates(cropped_cloud, stored_object_cloud_, sampled_grasps, object_source_frame, stored_grasp_list_);

    // remove any grasps with fingers in collision with object
    for (int i = static_cast<int>(stored_grasp_list_.grasps.size()) - 1; i >= 0; i --)
    {
      if (isInCollision(stored_grasp_list_.grasps[i].pose, stored_object_cloud_, false))
      {
        stored_grasp_list_.grasps.erase(stored_grasp_list_.grasps.begin() + i);
      }
    }

    ROS_INFO("%lu grasps remain after collision checking", stored_grasp_list_.grasps.size());
    // iteratively calculate grasp depth
    for (int i = 0; i < stored_grasp_list_.grasps.size(); i ++)
    {
      double depth_lower_bound = min_grasp_depth_;
      double depth_upper_bound = max_grasp_depth_;
      double current_depth = max_grasp_depth_;
      geometry_msgs::PoseStamped test_pose = stored_grasp_list_.grasps[i].pose;

      // check max grasp depth first
      test_pose.pose = adjustGraspDepth(stored_grasp_list_.grasps[i].pose.pose, current_depth);
      if (isInCollision(test_pose, cloud_, true))
      {
        geometry_msgs::Pose adjustedPose;
        // binary search to set grasp pose
        for (int j = 0; j < 5; j++)
        {
          current_depth = (depth_lower_bound + depth_upper_bound) / 2.0;
          adjustedPose = adjustGraspDepth(stored_grasp_list_.grasps[i].pose.pose, current_depth);
          test_pose.pose.position.x = adjustedPose.position.x;
          test_pose.pose.position.y = adjustedPose.position.y;
          test_pose.pose.position.z = adjustedPose.position.z;
          if (isInCollision(test_pose, cloud_, true))
          {
            depth_upper_bound = current_depth;
          }
          else
          {
            depth_lower_bound = current_depth;
          }
        }
      }
      stored_grasp_list_.grasps[i].pose.pose.position.x = test_pose.pose.position.x;
      stored_grasp_list_.grasps[i].pose.pose.position.y = test_pose.pose.position.y;
      stored_grasp_list_.grasps[i].pose.pose.position.z = test_pose.pose.position.z;
    }

    if (!stored_grasp_list_.grasps.empty())
    {
      res.grasp_list.header.frame_id = stored_grasp_list_.grasps[0].pose.header.frame_id;
      for (size_t i = 0; i < stored_grasp_list_.grasps.size(); i ++)
      {
        res.grasp_list.poses.push_back(stored_grasp_list_.grasps[i].pose.pose);
      }
    }
  }
  return true;
}

bool Suggester::suggestGraspsSceneCallback(rail_manipulation_msgs::SuggestGrasps::Request &req,
    rail_manipulation_msgs::SuggestGrasps::Response &res)
{
  boost::mutex::scoped_lock cloud_lock(cloud_mutex_);
  boost::mutex::scoped_lock grasp_lock(stored_grasp_mutex_);

  stored_grasp_list_.grasps.clear();
  stored_grasp_list_.object_index = 0;
  object_features_.clear();
  selected_grasps_.clear();

  stored_scene_cloud_ = req.cloud;

  geometry_msgs::PoseArray sampled_grasps;
  SampleGraspCandidatesScene(stored_scene_cloud_, sampled_grasps);

  ROS_INFO("Sampling complete.");

  if (!sampled_grasps.poses.empty())
  {
    ROS_INFO("Ranking grasp candidates...");
    rankCandidatesScene(stored_scene_cloud_, sampled_grasps, stored_grasp_list_);

    // remove any grasps with fingers in collision with object
    for (int i = static_cast<int>(stored_grasp_list_.grasps.size()) - 1; i >= 0; i --)
    {
      geometry_msgs::PoseStamped collision_test_pose = stored_grasp_list_.grasps[i].pose;
      // adjust grasp depth for just fingertips touching
      //TODO (enhancement): this is hardcoded and fetch-specific
      collision_test_pose.pose = adjustGraspDepth(stored_grasp_list_.grasps[i].pose.pose, -0.02);
      if (isInCollision(collision_test_pose, stored_scene_cloud_, false))
      {
        stored_grasp_list_.grasps.erase(stored_grasp_list_.grasps.begin() + i);
      }
    }

    ROS_INFO("%lu grasps remain after collision checking", stored_grasp_list_.grasps.size());
    // iteratively calculate grasp depth
    for (int i = 0; i < stored_grasp_list_.grasps.size(); i ++)
    {
      double depth_lower_bound = min_grasp_depth_;
      double depth_upper_bound = max_grasp_depth_;
      double current_depth = max_grasp_depth_;
      geometry_msgs::PoseStamped test_pose = stored_grasp_list_.grasps[i].pose;

      // check max grasp depth first
      test_pose.pose = adjustGraspDepth(stored_grasp_list_.grasps[i].pose.pose, current_depth);
      if (isInCollision(test_pose, cloud_, true))
      {
        geometry_msgs::Pose adjustedPose;
        // binary search to set grasp pose
        for (int j = 0; j < 5; j++)
        {
          current_depth = (depth_lower_bound + depth_upper_bound) / 2.0;
          adjustedPose = adjustGraspDepth(stored_grasp_list_.grasps[i].pose.pose, current_depth);
          test_pose.pose.position.x = adjustedPose.position.x;
          test_pose.pose.position.y = adjustedPose.position.y;
          test_pose.pose.position.z = adjustedPose.position.z;
          if (isInCollision(test_pose, cloud_, true))
          {
            depth_upper_bound = current_depth;
          }
          else
          {
            depth_lower_bound = current_depth;
          }
        }
      }
      stored_grasp_list_.grasps[i].pose.pose.position.x = test_pose.pose.position.x;
      stored_grasp_list_.grasps[i].pose.pose.position.y = test_pose.pose.position.y;
      stored_grasp_list_.grasps[i].pose.pose.position.z = test_pose.pose.position.z;
    }

    if (!stored_grasp_list_.grasps.empty())
    {
      res.grasp_list.header.frame_id = stored_grasp_list_.grasps[0].pose.header.frame_id;
      for (size_t i = 0; i < stored_grasp_list_.grasps.size(); i ++)
      {
        res.grasp_list.poses.push_back(stored_grasp_list_.grasps[i].pose.pose);
      }
    }
  }
  return true;
}

// Note: This is test code to use ONLY agile to calculate grasps.
// It should only be used for baseline testing, not actual use of this package!
bool Suggester::suggestGraspsAgileCallback(rail_manipulation_msgs::SuggestGrasps::Request &req,
    rail_manipulation_msgs::SuggestGrasps::Response &res)
{
  boost::mutex::scoped_lock cloud_lock(cloud_mutex_);
  boost::mutex::scoped_lock grasp_lock(stored_grasp_mutex_);

  stored_grasp_list_.grasps.clear();
  stored_grasp_list_.object_index = 0;
  object_features_.clear();
  selected_grasps_.clear();

  stored_object_cloud_ = req.cloud;

  //check that we have all required data
  if (!cloud_received_)
  {
    ROS_INFO("No point cloud data received!");
    return false;
  }

  //save frames for lots of upcoming point cloud transforming
  string environment_source_frame = cloud_->header.frame_id;
  string object_source_frame = stored_object_cloud_.header.frame_id;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  geometry_msgs::PoseArray sampled_grasps;
  SampleGraspCandidates(stored_object_cloud_, object_source_frame, environment_source_frame,
                        sampled_grasps, cropped_cloud, true);

  ROS_INFO("Grasp calculation complete.");

  if (!sampled_grasps.poses.empty())
  {
    ROS_INFO("Converting to executable poses...");

    fetch_grasp_suggestion::RankedGraspList grasp_list;
    grasp_list.grasps.resize(sampled_grasps.poses.size());
    for (size_t i = 0; i < sampled_grasps.poses.size(); i ++)
    {
      geometry_msgs::PoseStamped grasp;
      grasp_list.grasps[i].pose.header = sampled_grasps.header;
      grasp_list.grasps[i].pose.pose = sampled_grasps.poses[i];
    }

    //rotate each pose by 90 degree roll to align AGILE's results with gripper's coordinate frame
    tf::Quaternion rotation_adjustment = tf::createQuaternionFromRPY(M_PI / 2.0, 0, 0);
    for (size_t i = 0; i < grasp_list.grasps.size(); i++)
    {
      tf::Quaternion q;
      tf::quaternionMsgToTF(grasp_list.grasps[i].pose.pose.orientation, q);
      tf::Quaternion adjusted_q = q * rotation_adjustment;
      tf::quaternionTFToMsg(adjusted_q, grasp_list.grasps[i].pose.pose.orientation);
    }
    stored_grasp_list_ = grasp_list;

    // remove any grasps with fingers in collision with object
    for (int i = static_cast<int>(stored_grasp_list_.grasps.size()) - 1; i >= 0; i --)
    {
      if (isInCollision(stored_grasp_list_.grasps[i].pose, stored_object_cloud_, false))
      {
        stored_grasp_list_.grasps.erase(stored_grasp_list_.grasps.begin() + i);
      }
    }

    for (int i = 0; i < stored_grasp_list_.grasps.size(); i ++)
    {
      double depth_lower_bound = min_grasp_depth_;
      double depth_upper_bound = max_grasp_depth_;
      double current_depth = max_grasp_depth_;
      geometry_msgs::PoseStamped test_pose = stored_grasp_list_.grasps[i].pose;

      // check max grasp depth first
      test_pose.pose = adjustGraspDepth(stored_grasp_list_.grasps[i].pose.pose, current_depth);
      if (isInCollision(test_pose, cloud_, true))
      {
        geometry_msgs::Pose adjustedPose;
        // binary search to set grasp pose
        for (int j = 0; j < 5; j++)
        {
          current_depth = (depth_lower_bound + depth_upper_bound) / 2.0;
          adjustedPose = adjustGraspDepth(stored_grasp_list_.grasps[i].pose.pose, current_depth);
          test_pose.pose.position.x = adjustedPose.position.x;
          test_pose.pose.position.y = adjustedPose.position.y;
          test_pose.pose.position.z = adjustedPose.position.z;
          if (isInCollision(test_pose, cloud_, true))
          {
            depth_upper_bound = current_depth;
          }
          else
          {
            depth_lower_bound = current_depth;
          }
        }
      }
      stored_grasp_list_.grasps[i].pose.pose.position.x = test_pose.pose.position.x;
      stored_grasp_list_.grasps[i].pose.pose.position.y = test_pose.pose.position.y;
      stored_grasp_list_.grasps[i].pose.pose.position.z = test_pose.pose.position.z;
    }

    if (!stored_grasp_list_.grasps.empty())
    {
      res.grasp_list.header.frame_id = stored_grasp_list_.grasps[0].pose.header.frame_id;
      for (size_t i = 0; i < stored_grasp_list_.grasps.size(); i ++)
      {
        res.grasp_list.poses.push_back(stored_grasp_list_.grasps[i].pose.pose);
      }
    }
  }
  return true;
}

// Note: This is test code to use ONLY random antipodal sampling to calculate grasps.
// It should only be used for baseline testing, not actual use of this package!
bool Suggester::suggestGraspsRandomCallback(rail_manipulation_msgs::SuggestGrasps::Request &req,
    rail_manipulation_msgs::SuggestGrasps::Response &res)
{
  boost::mutex::scoped_lock cloud_lock(cloud_mutex_);
  boost::mutex::scoped_lock grasp_lock(stored_grasp_mutex_);

  stored_grasp_list_.grasps.clear();
  stored_grasp_list_.object_index = 0;
  object_features_.clear();
  selected_grasps_.clear();

  stored_object_cloud_ = req.cloud;

  //check that we have all required data
  if (!cloud_received_)
  {
    ROS_INFO("No point cloud data received!");
    return false;
  }

  //save frames for lots of upcoming point cloud transforming
  string environment_source_frame = cloud_->header.frame_id;
  string object_source_frame = stored_object_cloud_.header.frame_id;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  geometry_msgs::PoseArray sampled_grasps;
  SampleGraspCandidates(stored_object_cloud_, object_source_frame, environment_source_frame,
                        sampled_grasps, cropped_cloud);

  ROS_INFO("Sampling complete.");

  if (!sampled_grasps.poses.empty())
  {
    ROS_INFO("Converting %lu antipodal poses to executable poses...", sampled_grasps.poses.size());

    fetch_grasp_suggestion::RankedGraspList grasp_list;
    //randomly order grasps, take the first 30
    vector<int> indices;
    for (size_t i = 0; i < sampled_grasps.poses.size(); i ++)
    {
      indices.push_back(i);
    }
    std::random_shuffle(indices.begin(), indices.end());
    if (sampled_grasps.poses.size() > 30)
    {
      grasp_list.grasps.resize(30);
      ROS_INFO("Taking 30 random grasps.");
    }
    else
    {
      grasp_list.grasps.resize(sampled_grasps.poses.size());
    }

    //extract sampled grasps
    for (unsigned int i = 0; i < grasp_list.grasps.size(); i ++)
    {
      grasp_list.grasps[i].pose.header = sampled_grasps.header;
      grasp_list.grasps[i].pose.pose = sampled_grasps.poses[indices[i]];
    }

    //rotate each pose by 90 degree roll to align AGILE's results with gripper's coordinate frame
    tf::Quaternion rotation_adjustment = tf::createQuaternionFromRPY(M_PI / 2.0, 0, 0);
    for (size_t i = 0; i < grasp_list.grasps.size(); i++)
    {
      tf::Quaternion q;
      tf::quaternionMsgToTF(grasp_list.grasps[i].pose.pose.orientation, q);
      tf::Quaternion adjusted_q = q * rotation_adjustment;
      tf::quaternionTFToMsg(adjusted_q, grasp_list.grasps[i].pose.pose.orientation);
    }
    stored_grasp_list_ = grasp_list;

    // remove any grasps with fingers in collision with object
    for (int i = static_cast<int>(stored_grasp_list_.grasps.size()) - 1; i >= 0; i --)
    {
      if (isInCollision(stored_grasp_list_.grasps[i].pose, stored_object_cloud_, false))
      {
        stored_grasp_list_.grasps.erase(stored_grasp_list_.grasps.begin() + i);
      }
    }

    for (int i = 0; i < stored_grasp_list_.grasps.size(); i ++)
    {
      double depth_lower_bound = min_grasp_depth_;
      double depth_upper_bound = max_grasp_depth_;
      double current_depth = max_grasp_depth_;
      geometry_msgs::PoseStamped test_pose = stored_grasp_list_.grasps[i].pose;

      // check max grasp depth first
      test_pose.pose = adjustGraspDepth(stored_grasp_list_.grasps[i].pose.pose, current_depth);
      if (isInCollision(test_pose, cloud_, true))
      {
        geometry_msgs::Pose adjustedPose;
        // binary search to set grasp pose
        for (int j = 0; j < 5; j++)
        {
          current_depth = (depth_lower_bound + depth_upper_bound) / 2.0;
          adjustedPose = adjustGraspDepth(stored_grasp_list_.grasps[i].pose.pose, current_depth);
          test_pose.pose.position.x = adjustedPose.position.x;
          test_pose.pose.position.y = adjustedPose.position.y;
          test_pose.pose.position.z = adjustedPose.position.z;
          if (isInCollision(test_pose, cloud_, true))
          {
            depth_upper_bound = current_depth;
          }
          else
          {
            depth_lower_bound = current_depth;
          }
        }
      }
      stored_grasp_list_.grasps[i].pose.pose.position.x = test_pose.pose.position.x;
      stored_grasp_list_.grasps[i].pose.pose.position.y = test_pose.pose.position.y;
      stored_grasp_list_.grasps[i].pose.pose.position.z = test_pose.pose.position.z;
    }

    if (!stored_grasp_list_.grasps.empty())
    {
      res.grasp_list.header.frame_id = stored_grasp_list_.grasps[0].pose.header.frame_id;
      for (size_t i = 0; i < stored_grasp_list_.grasps.size(); i ++)
      {
        res.grasp_list.poses.push_back(stored_grasp_list_.grasps[i].pose.pose);
      }
    }
  }
  return true;
}

void Suggester::cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg)
{
  boost::mutex::scoped_lock lock(cloud_mutex_);

  *cloud_ = *msg;

  cloud_received_ = true;
}

void Suggester::getGraspSuggestions(const fetch_grasp_suggestion::SuggestGraspsGoalConstPtr &goal)
{
  boost::mutex::scoped_lock cloud_lock(cloud_mutex_);
  boost::mutex::scoped_lock object_lock(object_list_mutex_);

  fetch_grasp_suggestion::SuggestGraspsFeedback feedback;
  fetch_grasp_suggestion::SuggestGraspsResult result;

  //check that we have all required data
  if (!cloud_received_)
  {
    ROS_INFO("No point cloud data received!");
    suggest_grasps_server_.setSucceeded(result);
    return;
  }
  if (goal->object_index >= object_list_.objects.size() || goal->object_index < 0)
  {
    ROS_INFO("Object index out of array bounds!");
    suggest_grasps_server_.setSucceeded(result);
    return;
  }

  rail_manipulation_msgs::SegmentedObject object = object_list_.objects[goal->object_index];

  //save frames for lots of upcoming point cloud transforming
  string environment_source_frame = cloud_->header.frame_id;
  string object_source_frame = object.point_cloud.header.frame_id;

  feedback.message = "Sampling grasp candidates...";
  suggest_grasps_server_.publishFeedback(feedback);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  geometry_msgs::PoseArray sampled_grasps;
  SampleGraspCandidates(object.point_cloud, object_source_frame, environment_source_frame,
                        sampled_grasps, cropped_cloud);

  if (!sampled_grasps.poses.empty())
  {
    feedback.message = "Ranking grasps...";
    suggest_grasps_server_.publishFeedback(feedback);

    rankCandidates(cropped_cloud, object.point_cloud, sampled_grasps, object_source_frame, result.grasp_list);

    // remove any grasps with fingers in collision with object
    for (int i = static_cast<int>(result.grasp_list.grasps.size()) - 1; i >= 0; i --)
    {
      if (isInCollision(result.grasp_list.grasps[i].pose, object.point_cloud, false))
      {
        result.grasp_list.grasps.erase(result.grasp_list.grasps.begin() + i);
      }
    }

    grasps_publisher_.publish(result.grasp_list);
  }
  else
  {
    feedback.message = "No grasp candidates found!";
    suggest_grasps_server_.publishFeedback(feedback);
  }

  result.grasp_list.object_index = goal->object_index;
  suggest_grasps_server_.setSucceeded(result);
}

void Suggester::SampleGraspCandidates(sensor_msgs::PointCloud2 object, string object_source_frame,
    string environment_source_frame, geometry_msgs::PoseArray &grasps_out,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out)
{
  return SampleGraspCandidates(object, object_source_frame, environment_source_frame, grasps_out, cloud_out, false);
}

void Suggester::SampleGraspCandidates(sensor_msgs::PointCloud2 object, string object_source_frame,
    string environment_source_frame, geometry_msgs::PoseArray &grasps_out,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, bool agile_only)
{
  //get a pcl version of the object point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(object, *temp_cloud);
  pcl::fromPCLPointCloud2(*temp_cloud, *object_cloud);

  //transform object cloud to camera frame to get new crop box dimensions
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (object_cloud->header.frame_id != environment_source_frame)
  {
    pcl_ros::transformPointCloud(environment_source_frame, ros::Time(0), *object_cloud, object_source_frame,
                                 *transformed_cloud, tf_listener_);
    transformed_cloud->header.frame_id = environment_source_frame;
  }
  else
  {
    *transformed_cloud = *object_cloud;
  }

  //calculate workspace bounds in new coordinate frame
  pcl::PointXYZRGB min_workspace_point, max_workspace_point;
  pcl::getMinMax3D(*transformed_cloud, min_workspace_point, max_workspace_point);

  //crop cloud based on specified object
  double cloud_padding = 0.03;
  pcl::CropBox<pcl::PointXYZRGB> crop_box;
  Eigen::Vector4f min_point, max_point;
  min_point[0] = static_cast<float>(min_workspace_point.x - cloud_padding);
  min_point[1] = static_cast<float>(min_workspace_point.y - cloud_padding);
  min_point[2] = static_cast<float>(min_workspace_point.z - cloud_padding);
  max_point[0] = static_cast<float>(max_workspace_point.x + cloud_padding);
  max_point[1] = static_cast<float>(max_workspace_point.y + cloud_padding);
  max_point[2] = static_cast<float>(max_workspace_point.z + cloud_padding);
  crop_box.setMin(min_point);
  crop_box.setMax(max_point);
  crop_box.setInputCloud(cloud_);
  crop_box.filter(*cloud_out);

  rail_grasp_calculation_msgs::SampleGraspsGoal sample_goal;
  pcl::toPCLPointCloud2(*cloud_out, *temp_cloud);
  pcl_conversions::fromPCL(*temp_cloud, sample_goal.cloud);

  sample_goal.workspace.mode = rail_grasp_calculation_msgs::Workspace::WORKSPACE_VOLUME;
  sample_goal.workspace.x_min = min_point[0];
  sample_goal.workspace.y_min = min_point[1];
  sample_goal.workspace.z_min = min_point[2];
  sample_goal.workspace.x_max = max_point[0];
  sample_goal.workspace.y_max = max_point[1];
  sample_goal.workspace.z_max = max_point[2];

  if (agile_only)
  {
    sample_grasps_baseline_client_.sendGoal(sample_goal);
    sample_grasps_baseline_client_.waitForResult(ros::Duration(10.0));
    grasps_out = sample_grasps_baseline_client_.getResult()->graspList;
    return;
  }

  sample_grasps_client_.sendGoal(sample_goal);
  sample_grasps_client_.waitForResult(ros::Duration(10.0));
  grasps_out = sample_grasps_client_.getResult()->graspList;
}

void Suggester::SampleGraspCandidatesScene(sensor_msgs::PointCloud2 cloud, geometry_msgs::PoseArray &grasps_out)
{
  //get a pcl version of the point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(cloud, *temp_cloud);
  pcl::fromPCLPointCloud2(*temp_cloud, *scene_cloud);

  //calculate workspace bounds in new coordinate frame
  pcl::PointXYZRGB min_point, max_point;
  pcl::getMinMax3D(*scene_cloud, min_point, max_point);

  rail_grasp_calculation_msgs::SampleGraspsGoal sample_goal;
  pcl::toPCLPointCloud2(*scene_cloud, *temp_cloud);
  pcl_conversions::fromPCL(*temp_cloud, sample_goal.cloud);

  sample_goal.workspace.mode = rail_grasp_calculation_msgs::Workspace::WORKSPACE_VOLUME;
  sample_goal.workspace.x_min = min_point.x;
  sample_goal.workspace.y_min = min_point.y;
  sample_goal.workspace.z_min = min_point.z;
  sample_goal.workspace.x_max = max_point.x;
  sample_goal.workspace.y_max = max_point.y;
  sample_goal.workspace.z_max = max_point.z;

  sample_grasps_client_.sendGoal(sample_goal);
  sample_grasps_client_.waitForResult(ros::Duration(30.0));
  grasps_out = sample_grasps_client_.getResult()->graspList;
}

void Suggester::rankCandidates(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, sensor_msgs::PointCloud2 object_cloud,
    geometry_msgs::PoseArray &grasps, string object_source_frame,
    fetch_grasp_suggestion::RankedGraspList &ranked_grasps)
{
  //transform environment cloud back to object frame
  sensor_msgs::PointCloud2 environment_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2);
  if (cloud_in->header.frame_id != object_source_frame)
  {
    pcl_ros::transformPointCloud(object_source_frame, ros::Time(0), *cloud_in, object_source_frame, *transformed_cloud,
                                 tf_listener_);
    transformed_cloud->header.frame_id = object_source_frame;
    pcl::toPCLPointCloud2(*transformed_cloud, *temp_cloud);
  }
  else
  {
    pcl::toPCLPointCloud2(*cloud_in, *temp_cloud);
  }
  pcl_conversions::fromPCL(*temp_cloud, environment_cloud);

  rail_grasp_calculation_msgs::RankGraspsGoal rank_goal;
  rank_goal.sceneCloud = environment_cloud;
  rank_goal.segmentedCloud = object_cloud;
  rank_goal.graspList = grasps;
  rank_grasps_object_client_.sendGoal(rank_goal);

  rank_grasps_object_client_.waitForResult(ros::Duration(10.0));
  rail_grasp_calculation_msgs::RankGraspsResultConstPtr rank_result = rank_grasps_object_client_.getResult();

  ranked_grasps.grasps.clear();
  if (!rank_result->graspList.poses.empty())
  {
    ranked_grasps.grasps.resize(rank_result->graspList.poses.size());
    for (size_t i = 0; i < rank_result->graspList.poses.size(); i++)
    {
      ranked_grasps.grasps[i].pose.header.frame_id = rank_result->graspList.header.frame_id;
      ranked_grasps.grasps[i].pose.pose = rank_result->graspList.poses[i];
      ranked_grasps.grasps[i].heuristics = rank_result->heuristicList[i].heuristics;
    }

    //rotate each pose by 90 degree roll to align AGILE's results with gripper's coordinate frame
    tf::Quaternion rotation_adjustment = tf::createQuaternionFromRPY(M_PI / 2.0, 0, 0);
    for (size_t i = 0; i < ranked_grasps.grasps.size(); i++)
    {
      tf::Quaternion q;
      tf::quaternionMsgToTF(ranked_grasps.grasps[i].pose.pose.orientation, q);
      tf::Quaternion adjusted_q = q * rotation_adjustment;
      tf::quaternionTFToMsg(adjusted_q, ranked_grasps.grasps[i].pose.pose.orientation);
    }
  }
  else
  {
    ROS_INFO("Didn't receive any ranked grasps...");
  }
}

void Suggester::rankCandidatesScene(sensor_msgs::PointCloud2 cloud, geometry_msgs::PoseArray &grasps,
    fetch_grasp_suggestion::RankedGraspList &ranked_grasps)
{
  rail_grasp_calculation_msgs::RankGraspsGoal rank_goal;
  rank_goal.sceneCloud = cloud;
  rank_goal.graspList = grasps;
  rank_grasps_scene_client_.sendGoal(rank_goal);

  rank_grasps_scene_client_.waitForResult(ros::Duration(10.0));
  rail_grasp_calculation_msgs::RankGraspsResultConstPtr rank_result = rank_grasps_scene_client_.getResult();

  ranked_grasps.grasps.clear();
  if (!rank_result->graspList.poses.empty())
  {
    ranked_grasps.grasps.resize(rank_result->graspList.poses.size());
    for (size_t i = 0; i < rank_result->graspList.poses.size(); i++)
    {
      ranked_grasps.grasps[i].pose.header.frame_id = rank_result->graspList.header.frame_id;
      ranked_grasps.grasps[i].pose.pose = rank_result->graspList.poses[i];
      ranked_grasps.grasps[i].heuristics = rank_result->heuristicList[i].heuristics;
    }

    //rotate each pose by 90 degree roll to align AGILE's results with gripper's coordinate frame
    tf::Quaternion rotation_adjustment = tf::createQuaternionFromRPY(M_PI / 2.0, 0, 0);
    for (size_t i = 0; i < ranked_grasps.grasps.size(); i++)
    {
      tf::Quaternion q;
      tf::quaternionMsgToTF(ranked_grasps.grasps[i].pose.pose.orientation, q);
      tf::Quaternion adjusted_q = q * rotation_adjustment;
      tf::quaternionTFToMsg(adjusted_q, ranked_grasps.grasps[i].pose.pose.orientation);
    }
  }
  else
  {
    ROS_INFO("Didn't receive any ranked grasps...");
  }
}

geometry_msgs::Pose Suggester::adjustGraspDepth(geometry_msgs::Pose grasp_pose, double distance)
{
  geometry_msgs::Pose result;
  result.orientation.x = grasp_pose.orientation.x;
  result.orientation.y = grasp_pose.orientation.y;
  result.orientation.z = grasp_pose.orientation.z;
  result.orientation.w = grasp_pose.orientation.w;

  geometry_msgs::Transform transform;
  Eigen::Affine3d transform_matrix;
  transform.rotation = grasp_pose.orientation;
  transform.translation.x = grasp_pose.position.x;
  transform.translation.y = grasp_pose.position.y;
  transform.translation.z = grasp_pose.position.z;
  tf::transformMsgToEigen(transform, transform_matrix);

  Eigen::Vector3d transform_point, transformed_point;
  transform_point[0] = distance;
  transform_point[1] = 0;
  transform_point[2] = 0;
  transformed_point = transform_matrix*transform_point;
  tf::pointEigenToMsg(transformed_point, result.position);

  return result;
}

bool Suggester::isInCollision(geometry_msgs::PoseStamped grasp, sensor_msgs::PointCloud2 cloud, bool check_palm)
{
  // convert to pcl point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(cloud, *temp_cloud);
  pcl::fromPCLPointCloud2(*temp_cloud, *cloud_pcl);

  return isInCollision(grasp, cloud_pcl, check_palm);
}

bool Suggester::isInCollision(geometry_msgs::PoseStamped grasp, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    bool check_palm)
{
  //TODO(enhancement): finger tip size, shape, and position are all hardcoded for Fetch
  pcl::CropBox<pcl::PointXYZRGB> crop_box;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr collision_points(new pcl::PointCloud<pcl::PointXYZRGB>);
  Eigen::Vector4f min_point, max_point;
  crop_box.setInputCloud(cloud);

  geometry_msgs::PoseStamped check_grasp = grasp;
  if (grasp.header.frame_id != cloud->header.frame_id)
  {
    // transform grasp pose to point cloud frame
    tf_listener_.transformPose(cloud->header.frame_id, ros::Time(0), grasp, cloud->header.frame_id, check_grasp);
    check_grasp.header.frame_id = cloud->header.frame_id;
  }

  geometry_msgs::Transform transform;
  Eigen::Affine3d transform_matrix;
  transform.rotation = check_grasp.pose.orientation;
  transform.translation.x = check_grasp.pose.position.x;
  transform.translation.y = check_grasp.pose.position.y;
  transform.translation.z = check_grasp.pose.position.z;
  tf::transformMsgToEigen(transform, transform_matrix);

  Eigen::Vector3d transform_point;
  Eigen::Vector3d transformed_point;
  tf::Quaternion rotation_tf;
  double r, p, y;
  Eigen::Vector3f rotation;
  geometry_msgs::PoseStamped test_pose;
  visualization_msgs::Marker test_marker;

  // left finger
  min_point[0] = -0.029f;
  max_point[0] = 0.029f;
  min_point[1] = 0;
  max_point[1] = 0.014f;
  min_point[2] = -0.013f;
  max_point[2] = 0.013f;
  crop_box.setMin(min_point);
  crop_box.setMax(max_point);

  transform_point[0] = 0;
  transform_point[1] = -0.065f;
  transform_point[2] = 0;
  transformed_point = transform_matrix*transform_point;
  crop_box.setTranslation(transformed_point.cast<float>());

  tf::quaternionMsgToTF(check_grasp.pose.orientation, rotation_tf);
  tf::Matrix3x3(rotation_tf).getRPY(r, p, y);
  rotation[0] = static_cast<float>(r);
  rotation[1] = static_cast<float>(p);
  rotation[2] = static_cast<float>(y);
  crop_box.setRotation(rotation);

  crop_box.filter(*collision_points);
  if (!collision_points->points.empty())
    return true;

  // right finger
  min_point[1] = -0.014f;
  max_point[1] = 0;
  crop_box.setMin(min_point);
  crop_box.setMax(max_point);

  transform_point[1] = 0.065f;
  transformed_point = transform_matrix*transform_point;
  crop_box.setTranslation(transformed_point.cast<float>());

  crop_box.filter(*collision_points);
  if (!collision_points->points.empty())
    return true;

  if (check_palm)
  {
    min_point[0] = 0;
    max_point[0] = 0.137f;
    min_point[1] = -0.059f;
    max_point[1] = 0.059f;
    min_point[2] = -0.035f;
    max_point[2] = 0.035f;
    crop_box.setMin(min_point);
    crop_box.setMax(max_point);

    transform_point[0] = -0.166;
    transform_point[1] = 0;
    transform_point[2] = 0;
    transformed_point = transform_matrix*transform_point;
    crop_box.setTranslation(transformed_point.cast<float>());

    crop_box.filter(*collision_points);
    if (!collision_points->points.empty())
      return true;
  }

  return false;
}

void Suggester::objectsCallback(const rail_manipulation_msgs::SegmentedObjectList &list)
{
  boost::mutex::scoped_lock lock(object_list_mutex_);

  object_list_ = list;

  //set up MoveIt collision objects
  std_srvs::Empty clear;
  clear_objects_client_.call(clear);

  fetch_grasp_suggestion::AddObject add;
  for (size_t i = 0; i < list.objects.size(); i ++)
  {
    add.request.centroids.push_back(list.objects[i].centroid);
    add.request.point_clouds.push_back(list.objects[i].point_cloud);
    add.request.indices.push_back(i);
  }
  add_object_client_.call(add);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "suggester");

  Suggester s;

  ros::spin();

  return EXIT_SUCCESS;
}
