#include <ros/ros.h>
#include "NarrowPassageController.hpp"
#include <grid_map_cv/GridMapCvConverter.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "narrow_passage_controller");
    // ROS_INFO("narrow_passage_detection\n\n\n\n\n\n\n");

    ros::NodeHandle nodeHandle("~");
    narrow_passgae_controller::NarrowPassageController det(nodeHandle);

    // Spin
    // ros::AsyncSpinner spinner(nodeHandle.param("num_callback_threads", 1));  // Use n threads
    // spinner.start();
    // ros::AsyncSpinner spinner_1(1,&narrow_passgae_controller::queue_1);
    // spinner_1.start();

    // ros::AsyncSpinner spinner_2(1,&narrow_passgae_controller::queue_2);
    // spinner_2.start();


    // ros::AsyncSpinner spinner_3(1,&narrow_passage_detection::queue_3);
    // spinner_3.start();
    while(ros::ok())
    {
        ros::spin();
    }


    ros::shutdown();
    return 0;
}