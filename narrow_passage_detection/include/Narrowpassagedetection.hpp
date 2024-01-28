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
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
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


namespace narrow_passage_detection {
    class Narrowpassagedetection
    {
    protected:
        struct dis_buffer_type{
            double distance;
            int x;
            int y;
        };

        struct Point {
            double x, y;
        };

        void map_messageCallback(const grid_map_msgs::GridMap& msg);
        void pose_messageCallback(const nav_msgs::Odometry& pos_msg);
        void setupTimers();
        void mapUpdateTimerCallback(const ros::TimerEvent& timerEvent);
        void initialize();
        void narrowmap_pub();
        bool generate_output();
        void computegradient();
        void convert_from_gradient();
        void create_ray();
        void ray_detection(double k, double b, int angle,grid_map::Position robot_position);
        double calculateDistance(const Point &A, const Point& B);
        static bool compareByDis(const dis_buffer_type& a, const dis_buffer_type& b);

        ros::Subscriber map_sub;
        ros::Subscriber pose_sub;
        grid_map::GridMap elevationmap;
        ros::Duration maxduration;
        ros::Timer mapUpdateTimer_;
        bool getmap = false;
        ros::Publisher map_pub;
        cv::Mat input_img;
        grid_map::GridMap outputmap;
        cv::Mat gradient, direction;
        nav_msgs::Odometry pose_msg;
        grid_map::Matrix grid_data;
        double roll, pitch, yaw;
        
        std::vector<dis_buffer_type> dis_buffer;
        std::vector<dis_buffer_type> ray_buffer;

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
        // void messageCallback(const grid_map_msgs::GridMap& map);
        ros::NodeHandle nh;
    };
    

    
}

#endif