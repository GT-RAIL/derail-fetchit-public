#include <fetch_grasp_suggestion/selector.h>

using std::string;
using std::stringstream;
using std::vector;
using std::ios;

Selector::Selector() :
    pnh_("~"),
    execute_grasp_client_("/executor/execute_grasp"),
    execute_selected_grasp_server_(pnh_, "execute_selected_grasp",
                                   boost::bind(&Selector::executeSelectedGrasp, this, _1), false)
{
  string segmentation_topic, grasps_topic;
  pnh_.param<string>("segmentation_topic", segmentation_topic, "rail_segmentation/segmented_objects");
  pnh_.param<string>("grasps_topic", grasps_topic, "suggester/grasps");
  pnh_.param<string>("file_name", filename_, "grasp_data");

  stringstream ss;
  ss << filename_ << ".csv";
  filename_ = ss.str();

  im_server_.reset( new interactive_markers::InteractiveMarkerServer("grasp_selector", "grasp_selector_server", false));
  ros::Duration(0.1).sleep();
  im_server_->applyChanges();

  resetGraspData();

  grasps_subscriber_ = n_.subscribe(grasps_topic, 1, &Selector::graspsCallback, this);
  objects_subscriber_ = n_.subscribe(segmentation_topic, 1, &Selector::objectsCallback, this);

  cycle_grasps_server_ = pnh_.advertiseService("cycle_grasps", &Selector::cycleGraspsCallback, this);

  execute_selected_grasp_server_.start();
}

void Selector::executeSelectedGrasp(const fetch_grasp_suggestion::ExecuteSelectedGraspGoalConstPtr &goal)
{
  boost::mutex::scoped_lock lock(grasp_list_mutex_);

  fetch_grasp_suggestion::ExecuteSelectedGraspResult result;

  logTrainingInstances(grasp_index_);

  fetch_grasp_suggestion::ExecuteGraspGoal grasp_goal;
  grasp_goal.index = grasp_list_.object_index;
  grasp_goal.grasp_pose = grasp_list_.grasps[grasp_index_].pose;

  //skip execution for faster data collection
  execute_grasp_client_.sendGoal(grasp_goal);
  execute_grasp_client_.waitForResult(ros::Duration(60));
  fetch_grasp_suggestion::ExecuteGraspResultConstPtr execute_grasp_result = execute_grasp_client_.getResult();

  result.success = execute_grasp_result->success;
  if (!result.success)
  {
    result.error_code = execute_grasp_result->error_code;
    result.failure_point = execute_grasp_result->failure_point;
  }

  execute_selected_grasp_server_.setSucceeded(result);
}

void Selector::logTrainingInstances(int selected_index)
{
  //do nothing if this grasp has already been selected (prevent duplicate training instances)
  if (find(selected_grasps_.begin(), selected_grasps_.end(), selected_index) != selected_grasps_.end())
    return;

  //generate a positive and negative example for each pair of the selected grasp and any seen but unselected grasps
  file_.open(filename_.c_str(), ios::out | ios::app);
  for (int i = 0; i <= max_index_seen_; i ++)
  {
    if (i != selected_index && find(selected_grasps_.begin(), selected_grasps_.end(), i) == selected_grasps_.end())
    {
      //log positive training example
      file_ << Common::createTrainingInstance(object_features_, grasp_list_.grasps[selected_index].heuristics,
                                              grasp_list_.grasps[i].heuristics, true) << "\n";

      //log negative training example
      file_ << Common::createTrainingInstance(object_features_, grasp_list_.grasps[selected_index].heuristics,
                                              grasp_list_.grasps[i].heuristics, false) << "\n";
    }
  }
  file_.close();

  selected_grasps_.push_back(selected_index);
}

void Selector::objectsCallback(const rail_manipulation_msgs::SegmentedObjectList &list)
{
  boost::mutex::scoped_lock lock(grasp_list_mutex_);

  object_list_ = list;
  grasp_list_.grasps.clear();

  resetGraspData();

  updateMarker();
}

void Selector::graspsCallback(const fetch_grasp_suggestion::RankedGraspList &grasp_list)
{
  boost::mutex::scoped_lock lock(grasp_list_mutex_);

  this->grasp_list_ = grasp_list;

  resetGraspData();

  object_features_ = Common::calculateObjectFeatures(object_list_.objects[grasp_list.object_index].point_cloud);

  updateMarker();
}

void Selector::resetGraspData()
{
  grasp_index_ = 0;
  max_index_seen_ = 0;
  selected_grasps_.clear();
  object_features_.clear();
}

void Selector::updateMarker()
{
  if (grasp_list_.grasps.empty())
  {
    im_server_->clear();
  }
  else
  {
    geometry_msgs::PoseStamped pose = grasp_list_.grasps[grasp_index_].pose;

    visualization_msgs::InteractiveMarker gripper_marker;
    if (im_server_->get("gripper", gripper_marker))
    {
      //update gripper marker pose
      im_server_->setPose("gripper", pose.pose, pose.header);
    }
    else
    {
      //create new gripper marker
      gripper_marker = makeGripperMarker(pose);
      im_server_->insert(gripper_marker);
    }
  }

  im_server_->applyChanges();
}

bool Selector::cycleGraspsCallback(fetch_grasp_suggestion::CycleGrasps::Request &req,
  fetch_grasp_suggestion::CycleGrasps::Response &res)
{
  boost::mutex::scoped_lock lock(grasp_list_mutex_);

  if (req.forward)
  {
    if (grasp_index_ < grasp_list_.grasps.size() - 1)
      cycleGraspsForward();
  }
  else
  {
    if (grasp_index_ > 0)
      cycleGraspsBackward();
  }

  res.grasp = grasp_list_.grasps[grasp_index_].pose;
  res.index = grasp_index_;

  return true;
}

void Selector::cycleGraspsForward()
{
  grasp_index_ ++;
  max_index_seen_ = std::max(max_index_seen_, grasp_index_);
  updateMarker();
}

void Selector::cycleGraspsBackward()
{
  grasp_index_ --;
  updateMarker();
}

visualization_msgs::InteractiveMarker Selector::makeGripperMarker(geometry_msgs::PoseStamped pose)
{
  visualization_msgs::InteractiveMarker i_marker;
  i_marker.header.frame_id = pose.header.frame_id;

  i_marker.pose = pose.pose;

  i_marker.scale = 0.2;

  i_marker.name = "gripper";
  i_marker.description = "gripper goal pose";

  //gripper mesh marker
  visualization_msgs::Marker gripper_base = createGripperMeshMarker(
      0, 0, 0,
      0, 0, 0, 1,
      "package://fetch_description/meshes/gripper_link.dae");
  visualization_msgs::Marker gripper_right_finger = createGripperMeshMarker(
//      0, 0.11685, 0,
      0, 0.05, 0,
      0, 0, 0, 1,
      "package://fetch_description/meshes/r_gripper_finger_link.STL");
  visualization_msgs::Marker gripper_left_finger = createGripperMeshMarker(
//      0, -0.11685, 0,
      0, -0.05, 0,
      0, 0, 0, 1,
      "package://fetch_description/meshes/l_gripper_finger_link.STL");

  visualization_msgs::InteractiveMarkerControl gripper_control;
  gripper_control.markers.push_back(gripper_base);
  gripper_control.markers.push_back(gripper_right_finger);
  gripper_control.markers.push_back(gripper_left_finger);
  gripper_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  gripper_control.name = "gripper_control";
  gripper_control.always_visible = true;

  i_marker.controls.push_back(gripper_control);

  return i_marker;
}

visualization_msgs::Marker Selector::createGripperMeshMarker(double x, double y, double z,double rx, double ry,
  double rz, double rw, string mesh_location)
{
  visualization_msgs::Marker marker;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = rx;
  marker.pose.orientation.y = ry;
  marker.pose.orientation.z = rz;
  marker.pose.orientation.w = rw;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = mesh_location;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 0.65;
  marker.color.g = 0.0;
  marker.color.b = 0.65;
  marker.color.a = 1.0;

  return marker;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "selector");

  Selector s;

  ros::spin();

  return EXIT_SUCCESS;
}
