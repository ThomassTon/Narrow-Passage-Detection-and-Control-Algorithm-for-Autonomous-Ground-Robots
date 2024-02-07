#ifndef NARROW_PASSAGE_DETECTION_
#define NARROW_PASSAGE_DETECTION_

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

#include <cmath>

#include <vector>

#include <algorithm>

#include <geometry_msgs/Twist.h>


namespace narrow_passage_detection {
            ros::CallbackQueue queue_1;
        ros::CallbackQueue queue_2;
    class Narrowpassagedetection
    {
    protected:
        struct dis_buffer_type{
            double distance;
            grid_map::Index index;
            grid_map::Position position;
        };
        struct ray_buffer_type{
            double angle;
            double distance;
            grid_map::Index index;
            grid_map::Position position;
        };

        struct passage_width_buffer_type{
            double wide;
            grid_map::Index index1;
            grid_map::Index index2;
            grid_map::Position position1;
            grid_map::Position position2;
        };

        struct Point {
            double x, y;
        };

        void map_messageCallback(const grid_map_msgs::GridMap& msg);
        void map_messageCallback2(const nav_msgs::OccupancyGrid& msg);
        void pose_messageCallback(const nav_msgs::Odometry& pos_msg);
        void vel_messageCallback(const geometry_msgs::Twist& vel_msg);
        void setupTimers();
        void mapUpdateTimerCallback(const ros::TimerEvent& timerEvent);
        void narrowmap_pub();
        bool generate_output();
        void computegradient();
        void convert_from_gradient();
        void create_ray();
        bool is_obstacle(const passage_width_buffer_type& a);
        void ray_detection(double x, double y, double angle,grid_map::Position robot_position);
        double calculateDistance(const grid_map::Position &A, const grid_map::Position& B);
        static bool compareByDis(const dis_buffer_type& a, const dis_buffer_type& b);
        static bool compareByWidth(const passage_width_buffer_type&a ,const passage_width_buffer_type&b);
        static bool compareByPose(const ray_buffer_type &a, const ray_buffer_type &b);
        bool compute_angle_diff(double angle_robot, double angle2);
        void compute_passage_width();
        void mark_narrow_passage(const passage_width_buffer_type&a);
        bool isPointOnSegment(const grid_map::Position A, const grid_map::Position B, const grid_map::Position C);
        bool isPointOnSegment(const grid_map::Position A, const grid_map::Position B);

        void classification(std::vector<ray_buffer_type> &buffer1, std::vector<ray_buffer_type> &buffer2, const std::vector<ray_buffer_type> &data_);

        ros::Subscriber map_sub;
        ros::Subscriber map_sub2;

        ros::Subscriber pose_sub;
        ros::Subscriber vel_sub;
        grid_map::GridMap elevationmap;
        grid_map::GridMap occupancy_map;
        ros::Duration maxduration;
        ros::Timer mapUpdateTimer_;
        bool getmap = false;
        ros::Publisher map_pub;
        ros::Publisher width_pub;
        cv::Mat input_img;
        grid_map::GridMap outputmap;
        cv::Mat gradient, direction;
        nav_msgs::Odometry pose_msg;
        geometry_msgs::Twist vel_msg;
        grid_map::Matrix grid_data;
        double roll, pitch, yaw;
        bool tan90 = false;
        bool backward = false;

        
        std::vector<dis_buffer_type> dis_buffer;


        
        std::vector<ray_buffer_type> ray_buffer;

        std::vector<ray_buffer_type> test_buffer;

        template<typename Type_, int NChannels_>
        bool addLayerFromImage(const cv::Mat& image, const std::string& layer,
                                grid_map::GridMap& gridMap, const float lowerValue = 0.0,
                                const float upperValue = 1.0, const double alphaThreshold = 0.5)
        {
            if (gridMap.getSize()(0) != image.rows || gridMap.getSize()(1) != image.cols) {
            std::cerr << "Image size does not correspond to grid map size!" << std::endl;
            return false;
            }

            bool isColor = false;
            if (image.channels() >= 3) isColor = true;
            bool hasAlpha = false;
            if (image.channels() >= 4) hasAlpha = true;

            cv::Mat imageMono;
            if (isColor && !hasAlpha) {
            cv::cvtColor(image, imageMono, CV_BGR2GRAY);
            } else if (isColor && hasAlpha) {
            cv::cvtColor(image, imageMono, CV_BGRA2GRAY);
            } else if (!isColor && !hasAlpha){
            imageMono = image;
            } else {
            std::cerr << "Something went wrong when adding grid map layer form image!" << std::endl;
            return false;
            }

            const float mapValueDifference = upperValue - lowerValue;

            float maxImageValue;
            if (std::is_same<Type_, float>::value || std::is_same<Type_, double>::value) {
            maxImageValue = 1.0;
            } else if (std::is_same<Type_, unsigned short>::value || std::is_same<Type_, unsigned char>::value) {
            maxImageValue = (float)std::numeric_limits<Type_>::max();
            } else {
            std::cerr << "This image type is not supported." << std::endl;
            return false;
            }

            const Type_ alphaTreshold = (Type_)(alphaThreshold * maxImageValue);

            gridMap.add(layer);
            grid_map::Matrix& data = gridMap[layer];

            for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
                const grid_map::Index gridMapIndex = *iterator;
                const grid_map::Index imageIndex = iterator.getUnwrappedIndex();
                
                // Check for alpha layer.
                if (hasAlpha) {
                    const Type_ alpha = image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[NChannels_ - 1];
                    if (alpha < alphaTreshold) continue;
                }

                // Compute value.
                const Type_ imageValue = imageMono.at<Type_>(imageIndex(0), imageIndex(1));
                float mapValue = lowerValue + mapValueDifference * ((float) imageValue / maxImageValue);
                if(mapValue < 0.01)
                {
                    mapValue = NAN;
                }
                
                data(gridMapIndex(0), gridMapIndex(1)) = mapValue;
            }

            return true;
        }


    public:
        Narrowpassagedetection(ros::NodeHandle& nodeHandle);
        // ~Narrowpassagedetection();
        // void messageCallback(const ::GridMap& map);
        ros::NodeHandle nh;
        void initialize();

    };
    

    
}

#endif