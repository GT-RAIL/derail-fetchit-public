#include <ros/ros.h>
#include "fetchit_bin_detector/BinDetector.h"


int main(int argc, char** argv){
    ros::init(argc, argv, "bin_detector_node");
    ros::NodeHandle detector_nh;

    // get some launch args
    std::string segmentation_service = "/segmenter/segment_objects";
    std::string segmentation_frame = "base_link";
    bool visualize = false;

    detector_nh.getParam("/bin_detector_node/segmentation_service", segmentation_service);
    detector_nh.getParam("/bin_detector_node/segmentation_frame", segmentation_frame);
    detector_nh.getParam("/bin_detector_node/visualize", visualize);

    BinDetector bin_detector(detector_nh,segmentation_service,segmentation_frame,visualize);

    try{
        ros::spin();
    }catch(std::runtime_error& e){
        ROS_ERROR("bin_detector_node exception: %s", e.what());
        return -1;
    }

    return 0;
}