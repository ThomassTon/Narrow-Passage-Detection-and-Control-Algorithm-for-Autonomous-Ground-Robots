#ifndef NARROW_PASSAGE_CONTROLLER_HPP
#define NARROW_PASSAGE_CONTROLLER_HPP

// Grid Map
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/ProcessFile.h>
#include <grid_map_msgs/SetGridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
// ROS
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>


//  file output
#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>

#include <cmath>

#include <vector>

#include <algorithm>

#include <geometry_msgs/Twist.h>
#include <narrow_passage_detection_msgs/NarrowPassage.h>
#include <narrow_passage_detection_msgs/NarrowPassageController.h>


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>

namespace narrow_passgae_controller
{
    struct robot_range{
        grid_map::Position position;
        // double angle;
        double distance = 0;
    };
    ros::CallbackQueue queue_1;
    ros::CallbackQueue queue_2;
    class NarrowPassageController
    {
    protected:
        ros::Subscriber narrow_passage_sub;
        ros::Publisher  speed_pub;
        ros::Publisher  lqr_params_pub;
        ros::Subscriber stateSubscriber;
        ros::Subscriber map_sub;

        void narrow_passage_messageCallback(const narrow_passage_detection_msgs::NarrowPassage msg);
        void speed_publisher(const narrow_passage_detection_msgs::NarrowPassage msg);
        void lqr_params_publisher(const narrow_passage_detection_msgs::NarrowPassage msg);
        void stateCallback(const nav_msgs::Odometry odom_state);
        void updateRobotState(const nav_msgs::Odometry odom_state);
        void predicteRobotState(geometry_msgs::Pose &predict_pose);
        void map_messageCallback2(const nav_msgs::OccupancyGrid& msg);
        void predict_distance(const geometry_msgs::Pose robot_pose);
        void create_robot_range(std::vector<robot_range> robot, const geometry_msgs::Pose robot_pose, const double  length, const double width);
        void obsticke_distance(std::vector<robot_range> robot, grid_map::GridMap map);
        double compute_distance(grid_map::Position pos1, grid_map::Position pos2);
        static bool compareByDistance(robot_range &a, robot_range &b);
        void get_min_distance(double &right, double &left, double &front, double &back);
        geometry_msgs::PoseStamped pose;
        geometry_msgs::Vector3Stamped velocity_linear;
        geometry_msgs::Vector3Stamped velocity_angular;
        grid_map::GridMap occupancy_map;
        std::vector<robot_range> robot_right;
        std::vector<robot_range> robot_left;
        std::vector<robot_range> robot_front;
        std::vector<robot_range> robot_back;
        double right_min_distance;
        double left_min_distance;
        double front_min_distance;
        double back_min_distance;


        bool get_map = false;

        double dt;
    public:
        NarrowPassageController(ros::NodeHandle& nodeHandle);
        ros::NodeHandle nh;
        nav_msgs::Odometry latest_odom_;
    };
    

    
    
} // namespace narrow_passgae_controller


#endif