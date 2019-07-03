#include <rail_semantic_grasping/semantic_grasp_suggestion.h>
#include <rail_semantic_grasping/SegmentSemanticObjects.h>

using namespace std;
using namespace rail::semantic_grasping;

SemanticGraspSuggestion::SemanticGraspSuggestion():
    private_node_("~"), tf_listener_(tf_buffer_)
//    execute_grasp_client_("/executor/execute_grasp")
{
  private_node_.param<string>("object_semantic_segmentation_topic", object_semantic_segmentation_topic_,
                              "object_semantic_segmentation_node/semantic_objects");
  private_node_.param("min_distance_to_part", min_distance_to_part_, 0.001);

  grasp_publisher_ = private_node_.advertise<geometry_msgs::PoseStamped>("grasp", 1);

//  heuristic_subscriber_ = private_node_.subscribe("grasp_object_heuristic", 1, &SemanticGraspSuggestion::getHeuristicGraspsCallback, this);
//  objects_subscriber_ = node_.subscribe(segmentation_topic, 1, &SemanticGraspSuggestion::objectsCallback, this);

  get_semantic_grasps_ = private_node_.advertiseService("get_semantic_grasps", &SemanticGraspSuggestion::getSemanticGraspsCallback, this);

  segment_semantic_objects_client_ = node_.serviceClient<rail_semantic_grasping::SegmentSemanticObjects>("object_semantic_segmentation_node/segment");
  // retrieve grasp suggestions from fetch_grasp_suggestion, which does all the low-level computation
  suggest_heuristic_grasps_client_ = node_.serviceClient<fetch_grasp_suggestion::SuggestGrasps>("suggester/suggest_grasps");
}

bool SemanticGraspSuggestion::getSemanticGraspsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  // retrieve segmented semantic objects
  rail_semantic_grasping::SemanticObjectListConstPtr objects_msg =
      ros::topic::waitForMessage<rail_semantic_grasping::SemanticObjectList>(object_semantic_segmentation_topic_,
                                                                             node_, ros::Duration(10));
  if (objects_msg == nullptr)
  {
    ROS_INFO("No semantic object received before timeout.");
    return false;
  } else if (objects_msg->objects.empty())
  {
    ROS_INFO("No semantic object received.");
    return false;
  }

  // Important: assume there is just one semantic object segmented
  rail_semantic_grasping::SemanticObject semantic_object = objects_msg->objects[0];

  // get grasp suggestions
  fetch_grasp_suggestion::SuggestGrasps suggest_grasps;
  suggest_grasps.request.cloud = semantic_object.point_cloud;
  if (!suggest_heuristic_grasps_client_.call(suggest_grasps))
  {
    ROS_INFO("Call to suggest grasps service failed!");
    return false;
  }
  if (suggest_grasps.response.grasp_list.poses.empty())
  {
    ROS_INFO("No grasp suggestions found, stopping execution.");
    return false;
  }
  ROS_INFO("Received %lu grasp suggestions.", suggest_grasps.response.grasp_list.poses.size());
  geometry_msgs::PoseArray grasp_list = suggest_grasps.response.grasp_list;

  // ToDo: locate grasp
  // Test**************************************


  // visualize grasps
  size_t i = 0;
  while(true)
  {
    ROS_INFO("Showing grasp %lu...", i);

    geometry_msgs::PoseStamped grasp;
    grasp.header.frame_id = grasp_list.header.frame_id;
    grasp.pose = grasp_list.poses[i];
    grasp_publisher_.publish(grasp);

    // iterate through parts to find the closest part
    double min_sqr_dst = std::numeric_limits<double>::max();
    int closest_part_index = 0;
    for (size_t pi = 0; pi < semantic_object.parts.size(); ++pi)
    {
      // convert part pc from ros msg to pcl format
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr part_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(semantic_object.parts[pi].point_cloud, *part_pc);

      // transform the grasp pose so that it's in same frame as part_pc
      geometry_msgs::PoseStamped transformed_grasp_pose;
      if (grasp.header.frame_id != part_pc->header.frame_id)
      {
        transformed_grasp_pose.header.stamp = ros::Time(0);
        transformed_grasp_pose.header.frame_id = part_pc->header.frame_id;
        geometry_msgs::TransformStamped grasp_transform = tf_buffer_.lookupTransform(part_pc->header.frame_id,
                                                                                     grasp.header.frame_id,
                                                                                     ros::Time(0),
                                                                                     ros::Duration(1.0));
        tf2::doTransform(grasp, transformed_grasp_pose, grasp_transform);
      } else
      {
        transformed_grasp_pose = grasp;
      }

      // find distance between the position of the grasp and the point cloud of the part
      pcl::PointXYZRGB grasp_position;
      grasp_position.x = transformed_grasp_pose.pose.position.x;
      grasp_position.y = transformed_grasp_pose.pose.position.y;
      grasp_position.z = transformed_grasp_pose.pose.position.z;

      pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
      kdtree.setInputCloud(part_pc);
      vector<int> indices;
      vector<float> sqr_dsts;
      kdtree.nearestKSearch(grasp_position, 1, indices, sqr_dsts);

      // ToDo: maybe use average of top k
      if (sqr_dsts[0] < min_sqr_dst)
      {
        min_sqr_dst = sqr_dsts[0];
        closest_part_index = pi;
      }
    }

    ROS_INFO("min sqr dist: %f", min_sqr_dst);
    if (min_sqr_dst > min_distance_to_part_)
    {
      ROS_INFO("This grasp is far from any part");
    } else
    {
      ROS_INFO("This grasp is on part with affordance [%s]", semantic_object.parts[closest_part_index].affordance.c_str());
    }

    ROS_INFO("Enter a command (q: quit; [ or ]: back or forward): ");
    string input;
    std::cin >> input;

    switch (input[0])
    {
      case ']':
        if (i < grasp_list.poses.size() - 1)
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
        return true;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_suggestion");

  SemanticGraspSuggestion gs;

  ros::spin();

  return EXIT_SUCCESS;
}
