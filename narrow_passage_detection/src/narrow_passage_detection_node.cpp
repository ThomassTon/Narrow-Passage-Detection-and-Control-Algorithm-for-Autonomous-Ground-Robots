#include <ros/ros.h>
//#include "elevation_mapping/ElevationMapping.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "narrow_passage_detection");
    ROS_INFO("narrow_passage_detection\n\n\n\n\n\n\n");

    ros::NodeHandle nodeHandle("~");
//    elevation_mapping::ElevationMapping elevationMap(nodeHandle);

    // Spin
    ros::AsyncSpinner spinner(nodeHandle.param("num_callback_threads", 1));  // Use n threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}