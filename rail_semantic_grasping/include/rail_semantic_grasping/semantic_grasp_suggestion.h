#ifndef RAIL_SEMANTIC_GRASPING_GRASP_SUGGESTION_H_
#define RAIL_SEMANTIC_GRASPING_GRASP_SUGGESTION_H_

// Boost
#include <boost/thread/mutex.hpp>

// PCL
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

// ROS
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <std_srvs/Empty.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

//#include <actionlib/client/simple_action_client.h>
//#include <actionlib/server/simple_action_server.h>
//#include <fetch_grasp_suggestion/ExecuteGraspAction.h>
//#include <fetch_grasp_suggestion/PairwiseRank.h>
//#include <fetch_grasp_suggestion/RankedGraspList.h>
#include <fetch_grasp_suggestion/SuggestGrasps.h>
#include <rail_semantic_grasping/SemanticObjectList.h>
#include <rail_semantic_grasping/SemanticObject.h>
#include <rail_semantic_grasping/SemanticGrasp.h>
#include <rail_semantic_grasping/SegmentSemanticObjects.h>

// tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace rail
{
  namespace semantic_grasping
  {

/**
 * @brief Calculate and execute pairwise classified grasps for a segmented object in one autonomous step
 */
    class SemanticGraspSuggestion
    {

    public:

      /**
       * @brief Initialize ROS messages, services, and actions, required for grasp suggestion testing
       */
      SemanticGraspSuggestion();

    private:

      /**
        * @brief Retrieve grasp suggestion for a given object, with grasps ranked by a linear combination of heuristics
        * @param msg
        */
      bool getSemanticGraspsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

      bool visualizeSemanticGraspsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

      std::string object_semantic_segmentation_topic_;
      double min_distance_to_part_;
      int num_sampled_grasps_;

      ros::NodeHandle node_, private_node_;

      // topics
      ros::Publisher grasp_publisher_, objects_publisher_;

      // service clients
      ros::ServiceClient suggest_heuristic_grasps_client_;
      ros::ServiceClient segment_semantic_objects_client_;

      // service
      ros::ServiceServer get_semantic_grasps_, visualize_semantic_grasps_;

      // tf
      tf2_ros::Buffer tf_buffer_;
      tf2_ros::TransformListener tf_listener_;
    };

  }
}

#endif
