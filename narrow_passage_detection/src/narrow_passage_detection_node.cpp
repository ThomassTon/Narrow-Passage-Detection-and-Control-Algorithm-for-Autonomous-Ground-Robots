#include <ros/ros.h>
#include "Narrowpassagedetection.hpp"
#include <grid_map_cv/GridMapCvConverter.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "narrow_passage_detection");
    ROS_INFO("narrow_passage_detection\n\n\n\n\n\n\n");

    ros::NodeHandle nodeHandle("~");
    narrow_passage_detection::Narrowpassagedetection det(nodeHandle);

    // Spin
    // ros::AsyncSpinner spinner(nodeHandle.param("num_callback_threads", 1));  // Use n threads
    // spinner.start();
    ros::spin();
    // ros::waitForShutdown();
    return 0;
}