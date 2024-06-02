#ifndef NARROW_PASSAGE_CONTROLLER_HPP
#define NARROW_PASSAGE_CONTROLLER_HPP

// Grid Map
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/ProcessFile.h>
#include <grid_map_msgs/SetGridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
// ROS
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//  file output
#include <fstream>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

#include <cmath>

#include <vector>

#include <algorithm>

#include <geometry_msgs/Twist.h>
#include <narrow_passage_detection_msgs/NarrowPassage.h>
#include <narrow_passage_detection_msgs/NarrowPassageController.h>
#include <narrow_passage_detection_msgs/NarrowPassageDetection.h>


#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace narrow_passgae_controller
{
struct robot_range {
  grid_map::Position position;
  // double angle;
  double distance = 0;
};
struct robot_ladar {
  double left_distance;
  double right_distance;
  double front_distance;
  double back_distance;
};
ros::CallbackQueue queue_1;
ros::CallbackQueue queue_2;
class NarrowPassageController
{
protected:
  ros::Subscriber approach_point_sub;
  ros::Publisher speed_pub;
  ros::Publisher lqr_params_pub;
  ros::Subscriber stateSubscriber;
  ros::Subscriber map_sub;
  ros::Subscriber controllerTypeSwitch;
  ros::Publisher smoothPathPublisher;
  ros::Publisher approachedPublisher;

  void approach_point_messageCallback( const narrow_passage_detection_msgs::NarrowPassage msg );
  void stateCallback( const nav_msgs::Odometry odom_state );
  void map_messageCallback2( const grid_map_msgs::GridMap &msg);
  double compute_distance( grid_map::Position pos1, grid_map::Position pos2 );
  static bool compareByDistance( robot_range &a, robot_range &b );
  bool path_to_approach( geometry_msgs::Pose start, geometry_msgs::Pose end, geometry_msgs::Pose mid );
  void controllerTypeSwitchCallback(const narrow_passage_detection_msgs::NarrowPassageDetection &msg);

  bool endpoint_approached( geometry_msgs::Pose end );
  void reset();
  bool check_path_collision(nav_msgs::Path circle);

  double constrainAngle_mpi_pi( double x )
  {
    x = fmod( x + M_PI, 2.0 * M_PI );
    if ( x < 0 )
      x += 2.0 * M_PI;
    return x - M_PI;
  }
  geometry_msgs::PoseStamped pose;
  geometry_msgs::Pose robot_pose;
  geometry_msgs::Pose end_point;
  geometry_msgs::Pose extend_point;
  geometry_msgs::Pose mid_point;

  geometry_msgs::Vector3Stamped velocity_linear;
  geometry_msgs::Vector3Stamped velocity_angular;
  grid_map::GridMap elevation_map;

  double right_min_distance;
  double left_min_distance;
  double front_min_distance;
  double back_min_distance;

  bool get_elevation_map = false;
  bool get_map = false;
  bool approached_endpoint= false;
  bool approached_extendpoint= false;
  bool lookahead_detected= false;

  double dt;

public:
  NarrowPassageController( ros::NodeHandle &nodeHandle );
  ros::NodeHandle nh;
  nav_msgs::Odometry latest_odom_;
};

} // namespace narrow_passgae_controller

#endif