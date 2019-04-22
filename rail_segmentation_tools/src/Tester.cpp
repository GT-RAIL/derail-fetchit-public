#include "rail_segmentation_tools/Tester.h"

Tester::Tester()
{
  // setup publishers/subscribers we need
  segment_client = n.serviceClient<rail_manipulation_msgs::SegmentObjects>("rail_segmentation/segment_objects");
  merge_client = n.serviceClient<rail_manipulation_msgs::ProcessSegmentedObjects>("merger/merge_objects");
}

void Tester::testAll()
{
  rail_manipulation_msgs::SegmentObjects segment;
  ROS_INFO("Getting initial object segmentation");
  if (!segment_client.call(segment))
  {
    ROS_INFO("Couldn't call rail_segmentation!  Aborting tests.");
    return;
  }

  ROS_INFO("Running merge objects service...");

  rail_manipulation_msgs::ProcessSegmentedObjects process;
  process.request.segmented_objects = segment.response.segmented_objects;
  if (!merge_client.call(process))
  {
    ROS_INFO("Failed to call merge client!  Skipping merge test.");
  }
  else
  {
    ROS_INFO("Merge test complete.");
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rail_segmentation_tools_tester");

  Tester tester;

  ros::spinOnce();

  tester.testAll();

  ROS_INFO("rail_segmentation_tools testing complete, exiting.");

  return EXIT_SUCCESS;
}
