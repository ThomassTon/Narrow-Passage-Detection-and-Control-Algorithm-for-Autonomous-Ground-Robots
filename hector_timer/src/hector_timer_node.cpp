#include <ros/ros.h>
#include "hector_timer.hpp"
// #include <grid_map_cv/GridMapCvConverter.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "hector_timer");
    // ROS_INFO("narrow_passage_detection\n\n\n\n\n\n\n");

    ros::NodeHandle nodeHandle("~");
    hector_timer::HectorTimer det(nodeHandle);
    // narrow_passage_detection::Narrowpassagedetection det(nodeHandle);

    // Spin
    // ros::AsyncSpinner spinner(nodeHandle.param("num_callback_threads", 1));  // Use n threads
    // spinner.start();
     ros::AsyncSpinner spinner_1(1,&hector_timer::queue_1);
     spinner_1.start();

     ros::AsyncSpinner spinner_2(1,&hector_timer::queue_2);
     spinner_2.start();


    // ros::AsyncSpinner spinner_3(1,&narrow_passage_detection::queue_3);
    // spinner_3.start();
//    ros::waitForShutdown();
     ros::spin();
    return 0;
}