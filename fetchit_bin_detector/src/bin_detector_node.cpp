#include <ros/ros.h>
#include "fetchit_bin_detector/BinDetector.h"


int main(int argc, char** argv){
    ros::init(argc, argv, "bin_detector_node");
    ros::NodeHandle detector_nh, pnh("~");

    // get some launch args
    std::string segmentation_node = "/rail_segmentation";
    std::string segmentation_frame = "base_link";
    bool visualize = false;
    std::string icp_node = "/kit_template_matcher_node";

    pnh.getParam("segmentation_node", segmentation_node);
    pnh.getParam("segmentation_frame", segmentation_frame);
    pnh.getParam("visualize", visualize);
    pnh.getParam("kit_icp_node", icp_node);

    BinDetector bin_detector(detector_nh,segmentation_node,segmentation_frame,icp_node,visualize);

    try{
        ros::Rate loop_rate(100);
        while (ros::ok())
        {
            ros::spinOnce();
            bin_detector.publish_bin_tf();
            loop_rate.sleep();
        }
    }catch(std::runtime_error& e){
        ROS_ERROR("bin_detector_node exception: %s", e.what());
        return -1;
    }

    return 0;
}
