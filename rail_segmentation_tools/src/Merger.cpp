#include "rail_segmentation_tools/Merger.h"

using std::vector;

//constant definitions (to use in functions with reference parameters, e.g. param())
const double Merger::DEFAULT_MERGE_DST;
const double Merger::DEFAULT_COLOR_DELTA;

Merger::Merger() : pn("~")
{
  // grab any parameters we need
  pn.param("merge_dst", merge_dst, DEFAULT_MERGE_DST);
  pn.param("color_delta", color_delta, DEFAULT_COLOR_DELTA);
  pn.param("republish_segmented_objects", republish_segmented_objects, true);
  merge_dst = pow(merge_dst, 2);
  color_delta = pow(color_delta, 2);

  // setup publishers/subscribers we need
  segmented_objects_pub =
      n.advertise<rail_manipulation_msgs::SegmentedObjectList>("rail_segmentation/segmented_objects", 1, true);
  markers_pub = pn.advertise<visualization_msgs::MarkerArray>("markers", 1, true);
  calculate_featuers_client =
      n.serviceClient<rail_manipulation_msgs::ProcessSegmentedObjects>("rail_segmentation/calculate_features");
  merge_srv = pn.advertiseService("merge_objects", &Merger::mergeCallback, this);
}

bool Merger::mergeCallback(rail_manipulation_msgs::ProcessSegmentedObjects::Request &req,
    rail_manipulation_msgs::ProcessSegmentedObjects::Response &res)
{
  rail_manipulation_msgs::SegmentedObjectList input_list;
  input_list.header = req.segmented_objects.header;
  input_list.cleared = req.segmented_objects.cleared;
  input_list.objects = req.segmented_objects.objects;
  vector<rail_manipulation_msgs::SegmentedObject> merged_objects;

  for (size_t i = 0; i < input_list.objects.size(); i ++)
  {
    std::cout << input_list.objects[i].cielab[1] << ", " << input_list.objects[i].cielab[2] << std::endl;
  }

  bool merging = true;
  while (merging && input_list.objects.size() > 1)
  {
    std::cout << "Checking for merges over input list size: " << input_list.objects.size() << std::endl;
    merging = false;
    for (size_t i = 0; i < input_list.objects.size(); i++)
    {
      if (i == input_list.objects.size() - 1)
      {
        merged_objects.push_back(input_list.objects[i]);
        break;
      }
      bool merged = false;
      for (size_t j = i + 1; j < input_list.objects.size(); j++)
      {
        // color check
        if (pow(input_list.objects[i].cielab[1] - input_list.objects[j].cielab[1], 2)
            + pow(input_list.objects[i].cielab[2] - input_list.objects[j].cielab[2], 2) < color_delta)
        {
          // point cloud distance check for merge

          // convert to pcl point clouds
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj1_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj2_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
          pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2);
          pcl_conversions::toPCL(input_list.objects[i].point_cloud, *temp_cloud);
          pcl::fromPCLPointCloud2(*temp_cloud, *obj1_cloud);
          pcl_conversions::toPCL(input_list.objects[j].point_cloud, *temp_cloud);
          pcl::fromPCLPointCloud2(*temp_cloud, *obj2_cloud);

          // search for minimum distance
          double min_sqr_dst = std::numeric_limits<double>::max();
          pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
          kdtree.setInputCloud(obj1_cloud);
          for (size_t k = 0; k < obj2_cloud->points.size(); k++)
          {
            vector<int> indices;
            vector<float> sqr_dsts;
            kdtree.nearestKSearch(obj2_cloud->points[k], 1, indices, sqr_dsts);
            if (sqr_dsts[0] < min_sqr_dst)
            {
              min_sqr_dst = sqr_dsts[0];
            }
          }

          if (min_sqr_dst < merge_dst)
          {
            std::cout << "Merging: " << i << " - " << j << std::endl;

            rail_manipulation_msgs::SegmentedObject merged_object;

            // merge object point clouds
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            *merged_cloud = *obj1_cloud + *obj2_cloud;

            // fill in message data that doesn't need recalculating
            pcl::toPCLPointCloud2(*merged_cloud, *temp_cloud);
            pcl_conversions::fromPCL(*temp_cloud, merged_object.point_cloud);
            // TODO (enhancement): currently this does not tie in with recognition, so merges will be unrecognized
            merged_object.recognized = false;

            rail_manipulation_msgs::ProcessSegmentedObjects process_objects;
            process_objects.request.segmented_objects.objects.push_back(merged_object);
            if (!calculate_featuers_client.call(process_objects))
            {
              ROS_INFO("Could not call service to recalculate merged segmented object features!");
              return false;
            }
            merged_object = process_objects.response.segmented_objects.objects[0];

            merged_objects.push_back(merged_object);

            input_list.objects.erase(input_list.objects.begin() + j);

            merged = true;
            merging = true;
            break;
          }
        }
      }

      if (!merged)
      {
        merged_objects.push_back(input_list.objects[i]);
      }
    }

    input_list.objects = merged_objects;
    merged_objects.clear();
  }

  res.segmented_objects = input_list;

  if (republish_segmented_objects)
  {
    segmented_objects_pub.publish(res.segmented_objects);
  }

  visualization_msgs::MarkerArray markers;
  for (size_t i = 0; i < res.segmented_objects.objects.size(); i ++)
  {
    markers.markers.push_back(res.segmented_objects.objects[i].marker);
  }
  markers_pub.publish(markers);

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "merger");

  Merger m;

  ros::spin();

  return EXIT_SUCCESS;
}
