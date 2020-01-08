/*!
 * \file Segmenter.cpp
 * \brief The main segmentation node object.
 *
 * The segmenter is responsible for segmenting clusters from a point cloud topic. Visualization and data latched topics
 * are published after each request. A persistent array of objects is maintained internally.
 *
 * \author Russell Toris, WPI - russell.toris@gmail.com
 * \author David Kent, GT - dekent@gatech.edu
 * \date January 12, 2016
 */

// RAIL Segmentation
#include "rail_semantic_grasping/object_semantic_segmentation.h"

using namespace std;
using namespace rail::semantic_grasping;

ObjectSemanticSegmentation::ObjectSemanticSegmentation() : private_node_("~"), tf2_(tf_buffer_)
{
  // flag for the first point cloud coming in
  first_pc_in_ = false;

  // params
  private_node_.param("use_affordance_segmentation", use_affordance_segmentation_, true);
  private_node_.param<string>("point_cloud_topic", point_cloud_topic_, "/head_camera/depth_registered/points");
  private_node_.param<string>("camera_color_topic", camera_color_topic_, "/head_camera/rgb/image_rect_color");
  private_node_.param<string>("camera_depth_topic", camera_depth_topic_, "/head_camera/depth/image_rect");
  private_node_.param("label_markers", label_markers_, false);
  private_node_.param("min_affordance_pixels", min_affordance_pixels_, 0);
  private_node_.param<string>("geometric_segmentation_frame", geometric_segmentation_frame_, "base_link");
  private_node_.param("cylinder_segmentation_normal_k", cylinder_segmentation_normal_k_, 200);
  private_node_.param("cylinder_segmentation_normal_distance_weight", cylinder_segmentation_normal_distance_weight_,
                      0.1);
  private_node_.param("cylinder_segmentation_max_iteration", cylinder_segmentation_max_iteration_, 10000);
  private_node_.param("cylinder_segmentation_distance_threshold_ratio", cylinder_segmentation_distance_threshold_ratio_,
                      0.8);
  private_node_.param("cylinder_segmentation_cluster_tolerance", cylinder_segmentation_cluster_tolerance_, 0.01);
  private_node_.param("cylinder_segmentation_min_cluster_size", cylinder_segmentation_min_cluster_size_, 30);
  private_node_.param("cylinder_segmentation_max_cluster_size", cylinder_segmentation_max_cluster_size_, 10000);

  // setup subscribers
  point_cloud_sub_ = node_.subscribe(point_cloud_topic_, 1, &ObjectSemanticSegmentation::pointCloudCallback, this);
  color_image_sub_ = node_.subscribe(camera_color_topic_, 1, &ObjectSemanticSegmentation::colorImageCallback, this);
  depth_image_sub_ = node_.subscribe(camera_depth_topic_, 1, &ObjectSemanticSegmentation::depthImageCallback, this);

  // setup publishers
  semantic_objects_pub_ = private_node_.advertise<rail_semantic_grasping::SemanticObjectList>(
      "semantic_objects", 1, true);
  markers_pub_ = private_node_.advertise<visualization_msgs::MarkerArray>("markers", 1, true);
  // table_marker_pub_ = private_node_.advertise<visualization_msgs::Marker>("table_marker", 1, true);

  // setup service clients
  if (use_affordance_segmentation_)
  {
    detect_part_affordances_client_ =
        node_.serviceClient<rail_part_affordance_detection::DetectAffordances>("rail_part_affordance_detection/detect");
  }
  segment_objects_client_ =
      node_.serviceClient<rail_manipulation_msgs::SegmentObjects>("rail_segmentation/segment_objects");
  segment_objects_from_point_cloud_client_ =
      node_.serviceClient<rail_manipulation_msgs::SegmentObjectsFromPointCloud>(
          "rail_segmentation/segment_objects_from_point_cloud");
  calculate_features_client_ =
      node_.serviceClient<rail_manipulation_msgs::ProcessSegmentedObjects>("rail_segmentation/calculate_features");

  // setup service servers
  segment_srv_ = private_node_.advertiseService("segment", &ObjectSemanticSegmentation::segmentCallback, this);
  segment_objects_srv_ = private_node_.advertiseService("segment_objects",
                                                        &ObjectSemanticSegmentation::segmentObjectsCallback, this);
  clear_srv_ = private_node_.advertiseService("clear", &ObjectSemanticSegmentation::clearCallback, this);

  // debug
  debug_pc_pub_ = private_node_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("debug_pc", 1, true);
  debug_pc_pub_2_ = private_node_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("debug_pc_2", 1, true);
  debug_pc_pub_3_ = private_node_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("debug_pc_3", 1, true);
  debug_pose_pub_ = private_node_.advertise<geometry_msgs::PoseStamped>("debug_pose", 1, true);
  debug_img_pub_ = private_node_.advertise<sensor_msgs::Image>("debug_img", 1, true);

  // define affordance segmentation constants
  if (use_affordance_segmentation_)
  {
    // define affordance labels
    idx_to_affordance[0] = "background";
    idx_to_affordance[1] = "contain";
    idx_to_affordance[2] = "cut";
    idx_to_affordance[3] = "display";
    idx_to_affordance[4] = "engine";
    idx_to_affordance[5] = "grasp";
    idx_to_affordance[6] = "hit";
    idx_to_affordance[7] = "pound";
    idx_to_affordance[8] = "support";
    idx_to_affordance[9] = "w_grasp";

    // define object class labels
    idx_to_object_class[0] = "background";
    idx_to_object_class[1] = "bowl";
    idx_to_object_class[2] = "tvm";
    idx_to_object_class[3] = "pan";
    idx_to_object_class[4] = "hammer";
    idx_to_object_class[5] = "knife";
    idx_to_object_class[6] = "cup";
    idx_to_object_class[7] = "drill";
    idx_to_object_class[8] = "racket";
    idx_to_object_class[9] = "spatula";
    idx_to_object_class[10] = "bottle";

  }
}

void ObjectSemanticSegmentation::pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc)
{
  // lock for the point cloud
  boost::mutex::scoped_lock lock(pc_mutex_);
  // simply store the latest point cloud
  first_pc_in_ = true;
  pc_ = pc;
}

void ObjectSemanticSegmentation::colorImageCallback(const sensor_msgs::ImageConstPtr &color_img)
{
  boost::mutex::scoped_lock lock(color_img_mutex_);
  first_color_in_ = true;
  color_img_ = color_img;
}

void ObjectSemanticSegmentation::depthImageCallback(const sensor_msgs::ImageConstPtr &depth_img)
{
  boost::mutex::scoped_lock lock(depth_img_mutex_);
  first_depth_in_ = true;
  depth_img_ = depth_img;
}

bool ObjectSemanticSegmentation::clearCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  // lock for the messages
  boost::mutex::scoped_lock lock(msg_mutex_);
  // empty the list
  object_list_.objects.clear();
  object_list_.cleared = true;
  // set header information
  object_list_.header.seq++;
  object_list_.header.stamp = ros::Time::now();
  // republish
  semantic_objects_pub_.publish(object_list_);
  // delete markers
  for (size_t i = 0; i < markers_.markers.size(); i++)
  {
    markers_.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  if (label_markers_)
  {
    for (size_t i = 0; i < text_markers_.markers.size(); i++)
    {
      text_markers_.markers[i].action = visualization_msgs::Marker::DELETE;
    }
  }
  if (label_markers_)
  {
    visualization_msgs::MarkerArray marker_list;
    marker_list.markers.reserve(markers_.markers.size() + text_markers_.markers.size());
    marker_list.markers.insert(marker_list.markers.end(), markers_.markers.begin(), markers_.markers.end());
    marker_list.markers.insert(marker_list.markers.end(), text_markers_.markers.begin(), text_markers_.markers.end());
    markers_pub_.publish(marker_list);
  } else
  {
    markers_pub_.publish(markers_);
  }
  markers_.markers.clear();
  if (label_markers_)
  {
    text_markers_.markers.clear();
  }
  return true;
}

bool ObjectSemanticSegmentation::segmentCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  rail_semantic_grasping::SemanticObjectList objects;
  if (use_affordance_segmentation_)
  {
    return segmentObjects(objects);
  }
  else
  {
    return segmentObjectsWithoutAffordance(objects);
  }
}

bool ObjectSemanticSegmentation::segmentObjectsCallback(rail_semantic_grasping::SegmentSemanticObjectsRequest &req,
                                                        rail_semantic_grasping::SegmentSemanticObjectsResponse &res)
{
  if (use_affordance_segmentation_)
  {
    return segmentObjects(res.semantic_objects);
  }
  else
  {
    return segmentObjectsWithoutAffordance(res.semantic_objects);
  }
}

bool ObjectSemanticSegmentation::segmentObjectsWithoutAffordance(rail_semantic_grasping::SemanticObjectList &objects)
{
  // check if we have a point cloud first
  {
    boost::mutex::scoped_lock lock(pc_mutex_);
    if (!first_pc_in_)
    {
      ROS_WARN("No point cloud received yet. Ignoring segmentation request.");
      return false;
    }
  }
  sensor_msgs::ImagePtr rgb_img(new sensor_msgs::Image);
  {
    boost::mutex::scoped_lock lock(color_img_mutex_);
    if (!first_color_in_)
    {
      ROS_WARN("No color image received yet. Ignoring segmentation request.");
      return false;
    } else
    {
      *rgb_img = *color_img_;
    }
  }
  sensor_msgs::ImagePtr dep_img(new sensor_msgs::Image);
  {
    boost::mutex::scoped_lock lock(depth_img_mutex_);
    if (!first_depth_in_)
    {
      ROS_WARN("No depth image received yet. Ignoring segmentation request.");
      return false;
    } else
    {
      *dep_img = *depth_img_;
    }
  }

  // clear the objects first
  std_srvs::Empty empty;
  this->clearCallback(empty.request, empty.response);

  rail_manipulation_msgs::SegmentObjectsFromPointCloud segment_objects_srv;
  sensor_msgs::PointCloud2 provide_pc;
  pcl::toROSMsg(*pc_, provide_pc);
  ROS_INFO("check pc input %d", provide_pc.width);
  segment_objects_srv.request.point_cloud = provide_pc;
  if (!segment_objects_from_point_cloud_client_.call(segment_objects_srv))
  {
    ROS_INFO("Could not segment objects! Aborting.");
    return false;
  } else
  {
    ROS_INFO("object segmentation succeeded");
  }

  // let the user choose which point cloud is the target object
  ROS_INFO("Input object id according to visualization of /rail_segmentation/markers");
  int object_id;
  std::cin >> object_id;
  ROS_INFO("Segmented object id: %d!", object_id);
  if (object_id > segment_objects_srv.response.segmented_objects.objects.size()-1)
  {
    ROS_INFO("Input id exceeds max id! Aborting.");
    return false;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(segment_objects_srv.response.segmented_objects.objects[object_id].point_cloud, *object_pc);

  // check if we need to transform to a different frame
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_object_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (object_pc->header.frame_id != geometric_segmentation_frame_)
  {
    pcl_ros::transformPointCloud(geometric_segmentation_frame_, ros::Time(0), *object_pc, object_pc->header.frame_id,
                                 *transformed_object_pc, tf_);
    transformed_object_pc->header.frame_id = geometric_segmentation_frame_;
    transformed_object_pc->header.seq = object_pc->header.seq;
    transformed_object_pc->header.stamp = object_pc->header.stamp;
  } else
  {
    pcl::copyPointCloud(*object_pc, *transformed_object_pc);
  }

  rail_semantic_grasping::SemanticObject semantic_object;
  sensor_msgs::PointCloud2 object_pc_msg;
  pcl::toROSMsg(*transformed_object_pc, object_pc_msg);
  semantic_object.point_cloud = object_pc_msg;
  semantic_object.image_indices = segment_objects_srv.response.segmented_objects.objects[object_id].image_indices;
  semantic_object.color_image = *rgb_img;
  semantic_object.depth_image = *dep_img;

  // Note: add a psuedo object part for storing the material
  rail_semantic_grasping::SemanticPart semantic_part;
  semantic_object.parts.push_back(semantic_part);

  objects.objects.push_back(semantic_object);

  // compute features
  for (size_t oi = 0; oi < objects.objects.size(); ++oi)
  {
    // compute features
    rail_manipulation_msgs::SegmentedObject input_object;
    input_object.point_cloud = objects.objects[oi].point_cloud;
    rail_manipulation_msgs::ProcessSegmentedObjects process_objects;
    process_objects.request.segmented_objects.objects.push_back(input_object);
    if (!calculate_features_client_.call(process_objects))
    {
      ROS_INFO("Could not call service to calculate segmented object features!");
      return false;
    }
    objects.objects[oi].centroid = process_objects.response.segmented_objects.objects[0].centroid;
    objects.objects[oi].center = process_objects.response.segmented_objects.objects[0].center;
    objects.objects[oi].bounding_volume = process_objects.response.segmented_objects.objects[0].bounding_volume;
    objects.objects[oi].width = process_objects.response.segmented_objects.objects[0].width;
    objects.objects[oi].depth = process_objects.response.segmented_objects.objects[0].depth;
    objects.objects[oi].height = process_objects.response.segmented_objects.objects[0].height;
    objects.objects[oi].rgb = process_objects.response.segmented_objects.objects[0].rgb;
    objects.objects[oi].cielab = process_objects.response.segmented_objects.objects[0].cielab;
    objects.objects[oi].orientation = process_objects.response.segmented_objects.objects[0].orientation;
    objects.objects[oi].marker = process_objects.response.segmented_objects.objects[0].marker;

    markers_.markers.push_back(objects.objects[oi].marker);
  }
  markers_pub_.publish(markers_);

  // collect object part material
  for (size_t oi = 0; oi < objects.objects.size(); ++oi)
  {
    // collect object part material
    ROS_INFO("");
    ROS_INFO("The object has material (metal, ceramic, plastic, glass, wood, stone, paper): ");
    string material;
    std::cin >> material;
    ROS_INFO("This object has material %s!", material.c_str());
    objects.objects[oi].parts[0].material = material;
  }

  // Update object list and publish it
  objects.header.seq++;
  objects.header.stamp = ros::Time::now();
  objects.header.frame_id = geometric_segmentation_frame_;
  objects.cleared = false;
  object_list_ = objects;
  semantic_objects_pub_.publish(object_list_);

  return true;
}

bool ObjectSemanticSegmentation::segmentObjects(rail_semantic_grasping::SemanticObjectList &objects)
{
  // check if we have a point cloud first
  {
    boost::mutex::scoped_lock lock(pc_mutex_);
    if (!first_pc_in_)
    {
      ROS_WARN("No point cloud received yet. Ignoring segmentation request.");
      return false;
    }
  }
  sensor_msgs::ImagePtr rgb_img(new sensor_msgs::Image);
  {
    boost::mutex::scoped_lock lock(color_img_mutex_);
    if (!first_color_in_)
    {
      ROS_WARN("No color image received yet. Ignoring segmentation request.");
      return false;
    } else
    {
      *rgb_img = *color_img_;
    }
  }
  sensor_msgs::ImagePtr dep_img(new sensor_msgs::Image);
  {
    boost::mutex::scoped_lock lock(depth_img_mutex_);
    if (!first_depth_in_)
    {
      ROS_WARN("No depth image received yet. Ignoring segmentation request.");
      return false;
    } else
    {
      *dep_img = *depth_img_;
    }
  }

  // clear the objects first
  std_srvs::Empty empty;
  this->clearCallback(empty.request, empty.response);

  // call part affordance detection
  rail_part_affordance_detection::DetectAffordances detect_affordances_srv;
  if (!detect_part_affordances_client_.call(detect_affordances_srv))
  {
    ROS_INFO("Could not detect part affordances! Aborting.");
    return false;
  } else if (detect_affordances_srv.response.object_part_affordance_list.empty())
  {
    ROS_INFO("No affordance detected! Aborting.");
    return false;
  } else
  {
    ROS_INFO("affordance detection succeeded");
  }

  // Important: doesn't have significant effect, so just copy for now
  // transform input point cloud (depth frame) to the frame that the affordance detection uses (rgb frame)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
  {
    boost::mutex::scoped_lock lock(pc_mutex_);
    pcl::copyPointCloud(*pc_, *transformed_pc);
  }
//  std::string affordance_frame_id;
//  if (!detect_affordances_srv.response.object_part_affordance_list.empty())
//  {
//    {
//      boost::mutex::scoped_lock lock(pc_mutex_);
//      // perform the copy/transform using TF
//      affordance_frame_id = detect_affordances_srv.response.object_part_affordance_list[0].header.frame_id;
//      pcl_ros::transformPointCloud(affordance_frame_id,
//                                   ros::Time(0), *pc_, pc_->header.frame_id, *transformed_pc, tf_);
//      transformed_pc->header.frame_id = affordance_frame_id;
//      transformed_pc->header.seq = pc_->header.seq;
//      transformed_pc->header.stamp = pc_->header.stamp;
//    }
//  }

  // call rail_segmentation and get the list of objects
  rail_manipulation_msgs::SegmentObjectsFromPointCloud segment_objects_srv;
  sensor_msgs::PointCloud2 provide_pc;
  pcl::toROSMsg(*transformed_pc, provide_pc);
  ROS_INFO("check pc input %d", provide_pc.width);
  segment_objects_srv.request.point_cloud = provide_pc;
  if (!segment_objects_from_point_cloud_client_.call(segment_objects_srv))
  {
    ROS_INFO("Could not segment objects! Aborting.");
    return false;
  } else
  {
    ROS_INFO("object segmentation succeeded");
  }

  // match detected objects (from part affordance detection) with segmented objects (from segmentation)
  std::map<int, int> detected_to_segmented_object;
  std::map<int, int> match_distance;
  for (size_t i = 0; i < segment_objects_srv.response.segmented_objects.objects.size(); ++i)
  {
    // compute center of the segmented object
    vector<int> segmented_indices = segment_objects_srv.response.segmented_objects.objects[i].image_indices;
    int row_sum = 0;
    int col_sum = 0;
    for (size_t pi = 0; pi < segmented_indices.size(); ++pi)
    {
      int row = segmented_indices[pi] / transformed_pc->width;
      int col = segmented_indices[pi] - (row * transformed_pc->width);
      row_sum += row;
      col_sum += col;
    }
//    ROS_INFO("sums: %d, %d", row_sum, col_sum);
    int row_avg = int(row_sum / double(segmented_indices.size()));
    int col_avg = int(col_sum / double(segmented_indices.size()));
//    ROS_INFO("size %zu", segmented_indices.size());
//    ROS_INFO("width %d", transformed_pc->width);
//    ROS_INFO("Segmented object No.%zu has image coordinate %d, %d", i, col_avg, row_avg);

    for (size_t j = 0; j < detect_affordances_srv.response.object_part_affordance_list.size(); ++j)
    {
      uint16_t col_min = detect_affordances_srv.response.object_part_affordance_list[j].bounding_box[0];
      uint16_t row_min = detect_affordances_srv.response.object_part_affordance_list[j].bounding_box[1];
      uint16_t col_max = detect_affordances_srv.response.object_part_affordance_list[j].bounding_box[2];
      uint16_t row_max = detect_affordances_srv.response.object_part_affordance_list[j].bounding_box[3];

      // check if the center of the segmented object is in the bounding box of the detected object
      if (row_avg < row_max && row_avg > row_min && col_avg < col_max && col_avg > col_min)
      {
        // match the closest pair of detected bounding box center and segmented object center
        int row_center = ((int) row_max + (int) row_min) / 2;
        int col_center = ((int) col_max + (int) col_min) / 2;
        int distance =
            (row_avg - row_center) * (row_avg - row_center) + (col_avg - col_center) * (col_avg - col_center);
        if (detected_to_segmented_object.count(j) == 1)
        {
          if (distance < match_distance[j])
          {
            detected_to_segmented_object[j] = i;
            match_distance[j] = distance;
          }
        } else
        {
          detected_to_segmented_object[j] = i;
          match_distance[j] = distance;
        }
      }
    }
  }
  if (detected_to_segmented_object.empty())
  {
    ROS_INFO("Cannot match detected object parts with segmented object! Aborting.");
    return false;
  }
  for (map<int,int>::iterator it=detected_to_segmented_object.begin(); it!=detected_to_segmented_object.end(); ++it)
  {
    ROS_INFO("Detected object %d matches to segmented object %d", it->first, it->second);
  }

  // segment each object into parts based on affordances of parts
  // lock for the messages
  boost::mutex::scoped_lock lock(msg_mutex_);
  for (size_t oi = 0; oi < detect_affordances_srv.response.object_part_affordance_list.size(); oi++)
  {
    // match the detected object to the segmented object
    vector<int> segmented_indices;
    if (detected_to_segmented_object.count(oi) == 1)
    {
      segmented_indices = segment_objects_srv.response.segmented_objects.objects[detected_to_segmented_object.at(oi)].image_indices;
    }

    rail_part_affordance_detection::ObjectPartAffordance object_affordances;
    object_affordances = detect_affordances_srv.response.object_part_affordance_list[oi];
    std::string object_class = idx_to_object_class[object_affordances.object_class];
    ROS_INFO("");
    ROS_INFO("Detected object No.%zu is %s", oi, object_class.c_str());

    // iterate through affordance mask of the whole image and find number of unique affordances and their numbers of
    // occurances
    map<int, int> unique_affordances;
    for (size_t pi = 0; pi < object_affordances.affordance_mask.size(); pi++)
    {
      if (object_affordances.affordance_mask[pi] == 0) continue;
      unique_affordances[object_affordances.affordance_mask[pi]]++;
    }

    // construct a semantic object
    rail_semantic_grasping::SemanticObject semantic_object;
    semantic_object.name = object_class;
    // also combine pc of parts in the loop
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_object_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    vector<int> combined_object_image_indices;

    // for each affordance, get the corresponding part
    for (map<int, int>::iterator aff_it=unique_affordances.begin(); aff_it!=unique_affordances.end(); ++aff_it)
    {
      int aff_idx = aff_it->first;
      // ignore background
      if (aff_idx == 0)
      {
        continue;
      }
      // filter out affordances that have small number of occurances
      if (aff_it->second < min_affordance_pixels_)
      {
        continue;
      }

      rail_semantic_grasping::SemanticPart semantic_part;
      string cluster_affordance = idx_to_affordance[aff_idx];
      ROS_INFO("Affordance %s has %d supports", cluster_affordance.c_str(), aff_it->second);

      // extract the point cloud based on the segmentation mask
      // create unorganized point cloud
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
      //ROS_INFO("width %d and height %d and is_dense %d", transformed_pc->width, transformed_pc->height, transformed_pc->is_dense);
      for (size_t pi = 0; pi < object_affordances.affordance_mask.size(); pi++)
      {
        if (object_affordances.affordance_mask[pi] == aff_idx)
        {
          if (pcl_isfinite(transformed_pc->points[pi].x) & pcl_isfinite(transformed_pc->points[pi].y) & pcl_isfinite(transformed_pc->points[pi].z))
          {
            // use the segmented object to filter out points
            if (find(segmented_indices.begin(), segmented_indices.end(), pi) != segmented_indices.end())
            {
              cluster->points.push_back(transformed_pc->points[pi]);
              combined_object_image_indices.push_back(pi);
            }
          }
        }
      }

      if (cluster->points.empty())
      {
        continue;
      }

      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      cluster->header.frame_id = transformed_pc->header.frame_id;

      // check if we need to transform to a different frame
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
      if (cluster->header.frame_id != geometric_segmentation_frame_)
      {
        pcl_ros::transformPointCloud(geometric_segmentation_frame_, ros::Time(0), *cluster, cluster->header.frame_id,
                                     *transformed_cluster, tf_);
        transformed_cluster->header.frame_id = geometric_segmentation_frame_;
        transformed_cluster->header.seq = cluster->header.seq;
        transformed_cluster->header.stamp = cluster->header.stamp;
      } else
      {
        pcl::copyPointCloud(*cluster, *transformed_cluster);
      }

      // add pc of this part to the combined object
      // the += operator should take care of width, height, is_dense, and header
      *combined_object_pc += *transformed_cluster;

      sensor_msgs::PointCloud2 part_pc;
      pcl::toROSMsg(*transformed_cluster, part_pc);
      semantic_part.point_cloud = part_pc;
      // semantic_part.image_indices = cluster_indices;
      semantic_part.affordance = cluster_affordance;
      semantic_object.parts.push_back(semantic_part);
    }

    // Combine semantic parts to get the semantic object
    combined_object_pc->header.frame_id = geometric_segmentation_frame_; // need to be set, otherwise will be empty
    debug_pc_pub_.publish(combined_object_pc);
    sensor_msgs::PointCloud2 combined_object_pc_msg;
    pcl::toROSMsg(*combined_object_pc, combined_object_pc_msg);
    semantic_object.point_cloud = combined_object_pc_msg;

    semantic_object.image_indices = combined_object_image_indices;
    semantic_object.color_image = *rgb_img;
    semantic_object.depth_image = *dep_img;

    objects.objects.push_back(semantic_object);
  }

  // ToDo: process cluster
  // 1. cluster based on connected region
  // 2. detect handle
  // 3. detect opening

  // furthur segment the object based on geometric features
  this->segmentObjectsGeometric(objects);

  // compute features
  for (size_t oi = 0; oi < objects.objects.size(); ++oi)
  {
    string object_class = objects.objects[oi].name;
    for (size_t pi = 0; pi < objects.objects[oi].parts.size(); ++pi)
    {
      string part_affordance = objects.objects[oi].parts[pi].affordance;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr part_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(objects.objects[oi].parts[pi].point_cloud, *part_pc);
      // ToDo: after parts are segmented, compute other features. also make the computation a seperate function.
      // ToDo: This can be achieved by calling calculate_features service in rail_segmentation
      // compute centroid of part
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*part_pc, centroid);
      objects.objects[oi].parts[pi].centroid.x = centroid[0];
      objects.objects[oi].parts[pi].centroid.y = centroid[1];
      objects.objects[oi].parts[pi].centroid.z = centroid[2];

      // create visualization marker
      pcl::PCLPointCloud2::Ptr converted(new pcl::PCLPointCloud2);
      pcl::toPCLPointCloud2(*part_pc, *converted);
      // set namespace (e.g., obj_0/cup/contain)
      stringstream marker_ns;
      marker_ns << "obj_" << oi << "/" << object_class << "/" << part_affordance;

      visualization_msgs::Marker marker, text_marker;
      marker = this->createMarker(converted, marker_ns.str());
      markers_.markers.push_back(marker);
      text_marker = this->createTextMarker(part_affordance, marker.header, objects.objects[oi].parts[pi].centroid, marker_ns.str());
      text_markers_.markers.push_back(text_marker);
      objects.objects[oi].parts[pi].marker = marker;
      objects.objects[oi].parts[pi].text_marker = text_marker;
    }

    // compute features
    rail_manipulation_msgs::SegmentedObject input_object;
    input_object.point_cloud = objects.objects[oi].point_cloud;
    rail_manipulation_msgs::ProcessSegmentedObjects process_objects;
    process_objects.request.segmented_objects.objects.push_back(input_object);
    if (!calculate_features_client_.call(process_objects))
    {
      ROS_INFO("Could not call service to calculate segmented object features!");
      return false;
    }
    objects.objects[oi].centroid = process_objects.response.segmented_objects.objects[0].centroid;
    objects.objects[oi].center = process_objects.response.segmented_objects.objects[0].center;
    objects.objects[oi].bounding_volume = process_objects.response.segmented_objects.objects[0].bounding_volume;
    objects.objects[oi].width = process_objects.response.segmented_objects.objects[0].width;
    objects.objects[oi].depth = process_objects.response.segmented_objects.objects[0].depth;
    objects.objects[oi].height = process_objects.response.segmented_objects.objects[0].height;
    objects.objects[oi].rgb = process_objects.response.segmented_objects.objects[0].rgb;
    objects.objects[oi].cielab = process_objects.response.segmented_objects.objects[0].cielab;
    objects.objects[oi].orientation = process_objects.response.segmented_objects.objects[0].orientation;
    objects.objects[oi].marker = process_objects.response.segmented_objects.objects[0].marker;
  }

  // publish markers for each object
  if (label_markers_)
  {
    visualization_msgs::MarkerArray marker_list;
    marker_list.markers.reserve(markers_.markers.size() + text_markers_.markers.size());
    marker_list.markers.insert(marker_list.markers.end(), markers_.markers.begin(), markers_.markers.end());
    marker_list.markers.insert(marker_list.markers.end(), text_markers_.markers.begin(), text_markers_.markers.end());
    markers_pub_.publish(marker_list);
  } else
  {
    markers_pub_.publish(markers_);
  }

  // collect object part material
  for (size_t oi = 0; oi < objects.objects.size(); ++oi)
  {
    string object_class = objects.objects[oi].name;
    for (size_t pi = 0; pi < objects.objects[oi].parts.size(); ++pi)
    {
      string part_affordance = objects.objects[oi].parts[pi].affordance;

      stringstream marker_ns;
      marker_ns << "obj_" << oi << "/" << object_class << "/" << part_affordance;

      // collect object part material
      ROS_INFO("");
      ROS_INFO("%s has material (metal, ceramic, plastic, glass, wood, stone, paper): ", marker_ns.str().c_str());
      string material;
      std::cin >> material;
      ROS_INFO("This part has material %s!", material.c_str());

      objects.objects[oi].parts[pi].material = material;
    }
  }

  // Update object list and publish it
  objects.header.seq++;
  objects.header.stamp = ros::Time::now();
  objects.header.frame_id = geometric_segmentation_frame_;
  objects.cleared = false;
  object_list_ = objects;
  semantic_objects_pub_.publish(object_list_);

  return true;
}

bool ObjectSemanticSegmentation::segmentObjectsGeometric(rail_semantic_grasping::SemanticObjectList &objects)
{
  for (size_t i = 0; i < objects.objects.size(); ++i)
  {
    vector<rail_semantic_grasping::SemanticPart> new_parts;
    if (objects.objects[i].name == "cup")
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(objects.objects[i].point_cloud, *object_pc);

      // segment opening from body
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr opening_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr body_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
      this->extractOpening(object_pc, body_pc, opening_pc);

      // segment body and handle
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr handle_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_body_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
      bool handle_segmented = this->extractHandle(body_pc, new_body_pc, handle_pc);
      if (handle_segmented)
      {
        body_pc = new_body_pc;
      }

      // assign newly segmented parts to the semantic object
      rail_semantic_grasping::SemanticPart body_part;
      pcl::toROSMsg(*body_pc, body_part.point_cloud);
      body_part.affordance = "body";
      new_parts.push_back(body_part);
      debug_pc_pub_.publish(body_pc);

      rail_semantic_grasping::SemanticPart opening_part;
      pcl::toROSMsg(*opening_pc, opening_part.point_cloud);
      opening_part.affordance = "opening";
      new_parts.push_back(opening_part);
      debug_pc_pub_2_.publish(opening_pc);

      if (handle_segmented)
      {
        rail_semantic_grasping::SemanticPart handle_part;
        pcl::toROSMsg(*handle_pc, handle_part.point_cloud);
        handle_part.affordance = "handle";
        new_parts.push_back(handle_part);
        debug_pc_pub_3_.publish(handle_pc);
      }
    }
    if (!new_parts.empty())
    {
      objects.objects[i].parts = new_parts;
    }
  }
  return true;
}

bool ObjectSemanticSegmentation::extractHandle(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &object,
                                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr &body,
                                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr &handle)
{
  // get dimension of the segmented object
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*object, min_pt, max_pt);
  double width = max_pt[0] - min_pt[0];

  // segment the cylinder and the remaining point cloud should have the handle
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmenter;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr cylinder_indices(new pcl::PointIndices);

  // Estimate point normals
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(object);
  // estimate a point's normal using its K nearest neighbors
  normal_estimator.setKSearch(cylinder_segmentation_normal_k_);
  // another option: normal_estimator.setRadiusSearch(0.03); // 3cm
  normal_estimator.compute(*cloud_normals);

  // create the segmentation object for cylinder segmentation and set all the parameters
  segmenter.setOptimizeCoefficients(true);
  segmenter.setModelType(pcl::SACMODEL_CYLINDER);
  segmenter.setMethodType(pcl::SAC_RANSAC);
  segmenter.setNormalDistanceWeight(cylinder_segmentation_normal_distance_weight_);
  segmenter.setMaxIterations(cylinder_segmentation_max_iteration_);
  segmenter.setDistanceThreshold(width * cylinder_segmentation_distance_threshold_ratio_);
  segmenter.setRadiusLimits(0, width);
  segmenter.setInputCloud(object);
  segmenter.setInputNormals(cloud_normals);
  segmenter.segment(*cylinder_indices, *coefficients_cylinder);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinder_pc(new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr handle_pc(new pcl::PointCloud<pcl::PointXYZRGB> ());

  // extract the cylinder point cloud and the remaining point cloud
  extract.setInputCloud(object);
  extract.setIndices(cylinder_indices);
  extract.setNegative(false);
  extract.filter(*cylinder_pc);
  extract.setNegative(true);
  extract.filter(*handle_pc);
  if (cylinder_pc->points.empty() or handle_pc->points.empty())
  {
    ROS_INFO("Cannot find the cylinder or the handle.");
    return false;
  }

  // further refine the point cloud of the handle. The largest cluster should correspond to the handle
  vector<pcl::PointIndices> clusters;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr cluster_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  cluster_tree->setInputCloud(handle_pc);
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> cluster_extractor;
  cluster_extractor.setClusterTolerance(cylinder_segmentation_cluster_tolerance_);
  cluster_extractor.setMinClusterSize(cylinder_segmentation_min_cluster_size_);
  cluster_extractor.setMaxClusterSize(cylinder_segmentation_max_cluster_size_);
  cluster_extractor.setSearchMethod(cluster_tree);
  cluster_extractor.setInputCloud(handle_pc);
  cluster_extractor.extract(clusters);

  size_t max_cluster_size=0;
  int max_cluster_index=0;
  if (!clusters.empty())
  {
    for (size_t ci = 0; ci < clusters.size(); ++ci)
    {
      if (clusters[ci].indices.size() > max_cluster_size)
      {
        max_cluster_size = clusters[ci].indices.size();
        max_cluster_index = ci;
      }
    }
  } else
  {
    ROS_INFO("Cannot cluster remaining point cloud that are not part of the cylinder to find the handle.");
    return false;
  }

  // extract the largest cluster that corresponds to the handle
  pcl::PointIndices::Ptr handle_indices(new pcl::PointIndices);
  *handle_indices = clusters[max_cluster_index];
  extract.setInputCloud(handle_pc);
  extract.setIndices(handle_indices);
  extract.setNegative(false);
  extract.filter(*handle_pc);

  // add point cloud that are not part of the handle back to the cylinder point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr remain_pc(new pcl::PointCloud<pcl::PointXYZRGB> ());
  extract.setNegative(true);
  extract.filter(*remain_pc);
  // add pc of this part to the combined object
  // the += operator should take care of width, height, is_dense, and header
  *cylinder_pc += *remain_pc;

  body = cylinder_pc;
  handle = handle_pc;
  return true;
}

bool ObjectSemanticSegmentation::extractOpening(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &object,
                                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr &body,
                                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr &opening)
{
  // The following part is used to segment a pc into two parts based on height
  // calculate the axis-aligned bounding box
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*object, min_pt, max_pt);
  double max_z = max_pt[2];
  //      double width = max_pt[0] - min_pt[0];
  //      double depth = max_pt[1] - min_pt[1];
  //      double height = max_pt[2] - min_pt[2];
  //      geometry_msgs::PoseStamped center;
  //      center.header.frame_id = geometric_segmentation_frame_;
  //      center.header.stamp = ros::Time(0);
  //      center.pose.position.x = (max_pt[0] + min_pt[0]) / 2.0;
  //      center.pose.position.y = (max_pt[1] + min_pt[1]) / 2.0;
  //      center.pose.position.z = max_pt[2];
  //      debug_pose_pub_.publish(center);

  // filter out pc that are not less than max_z - 0.05
  double height_constraint = max_z - 0.02;
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr bounds (new pcl::ConditionAnd<pcl::PointXYZRGB>());
  bounds->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LE, height_constraint)));

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr remaining_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ConditionalRemoval<pcl::PointXYZRGB> removal(true);
  removal.setCondition(bounds);
  removal.setInputCloud(object);
  removal.filter(*remaining_pc);
  const pcl::IndicesConstPtr &filter_indices = removal.getRemovedIndices();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(object);
  extract.setIndices(filter_indices);
  extract.filter(*filtered_pc);

  body = remaining_pc;
  opening = filtered_pc;
  return true;
}

visualization_msgs::Marker ObjectSemanticSegmentation::createMarker(const pcl::PCLPointCloud2::ConstPtr &pc,
    const std::string &marker_namespace) const
{
  visualization_msgs::Marker marker;
  // set header field
  marker.header.frame_id = pc->header.frame_id;

  // set marker namespace
  marker.ns = marker_namespace;

  // default position
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // default scale
  marker.scale.x = MARKER_SCALE;
  marker.scale.y = MARKER_SCALE;
  marker.scale.z = MARKER_SCALE;

  // set the type of marker and our color of choice
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.color.a = 1.0;

  // downsample point cloud for visualization
  pcl::PCLPointCloud2 downsampled;
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
  voxel_grid.setInputCloud(pc);
  voxel_grid.setLeafSize(DOWNSAMPLE_LEAF_SIZE, DOWNSAMPLE_LEAF_SIZE, DOWNSAMPLE_LEAF_SIZE);
  voxel_grid.filter(downsampled);

  // convert to an easy to use point cloud message
  sensor_msgs::PointCloud2 pc2_msg;
  pcl_conversions::fromPCL(downsampled, pc2_msg);
  sensor_msgs::PointCloud pc_msg;
  sensor_msgs::convertPointCloud2ToPointCloud(pc2_msg, pc_msg);

  // place in the marker message
  marker.points.resize(pc_msg.points.size());
  // int r = 0, g = 0, b = 0;
  for (size_t j = 0; j < pc_msg.points.size(); j++)
  {
    marker.points[j].x = pc_msg.points[j].x;
    marker.points[j].y = pc_msg.points[j].y;
    marker.points[j].z = pc_msg.points[j].z;

//    // use average RGB
//    uint32_t rgb = *reinterpret_cast<int *>(&pc_msg.channels[0].values[j]);
//    r += (int) ((rgb >> 16) & 0x0000ff);
//    g += (int) ((rgb >> 8) & 0x0000ff);
//    b += (int) ((rgb) & 0x0000ff);
  }

  // set average RGB
//  marker.color.r = ((float) r / (float) pc_msg.points.size()) / 255.0;
//  marker.color.g = ((float) g / (float) pc_msg.points.size()) / 255.0;
//  marker.color.b = ((float) b / (float) pc_msg.points.size()) / 255.0;
  marker.color.r = rand() / double(RAND_MAX);
  marker.color.g = rand() / double(RAND_MAX);
  marker.color.b = rand() / double(RAND_MAX);
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::Marker ObjectSemanticSegmentation::createTextMarker(const std::string &label, const std_msgs::Header &header,
    const geometry_msgs::Point &position, const std::string &marker_namespace) const
{
  // Create a text marker to label the current marker
  visualization_msgs::Marker text_marker;
  text_marker.header = header;
  text_marker.ns = marker_namespace;
  // part marker has id 0, label has id 1
  text_marker.id = 1;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;

  text_marker.pose.position.x = position.x - 0.05;
  text_marker.pose.position.y = position.y;
  text_marker.pose.position.z = position.z + 0.02;

  text_marker.scale.x = .03;
  text_marker.scale.y = .03;
  text_marker.scale.z = .03;

  text_marker.color.r = 1;
  text_marker.color.g = 1;
  text_marker.color.b = 1;
  text_marker.color.a = 1;

  text_marker.text = label;

  return text_marker;
}
