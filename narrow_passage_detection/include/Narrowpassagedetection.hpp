#ifndef NARROW_PASSAGE_DETECTION_
#define NARROW_PASSAGE_DETECTION_

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
#include <nav_msgs/OccupancyGrid.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
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


namespace narrow_passage_detection
{
ros::CallbackQueue queue_1;
ros::CallbackQueue queue_2;
ros::CallbackQueue queue_3;

class Narrowpassagedetection
{
protected:
  struct dis_buffer_type {
    double distance;
    grid_map::Index index;
    grid_map::Position position;
  };
  struct ray_buffer_type {
    double angle;
    double distance;
    grid_map::Index index;
    grid_map::Position position;
  };

  struct point_info {
    grid_map::Position position;
    int side;

    point_info( grid_map::Position pos, int s ) : position( pos ), side( s ) { }
  };

  struct passage_width_buffer_type {
    double wide;
    grid_map::Index index1;
    grid_map::Index index2;
    grid_map::Position position1;
    grid_map::Position position2;
  };

  struct Point {
    double x, y;
  };

  void map_messageCallback( const grid_map_msgs::GridMap &msg );
  void map_messageCallback2( const nav_msgs::OccupancyGrid &msg );
  void path_messageCallback( const nav_msgs::Path &msg );
  void pose_messageCallback( const nav_msgs::Odometry &pos_msg );
  void vel_messageCallback( const geometry_msgs::Twist &vel_msg );
  void narrowmap_pub( grid_map::GridMap map );
  void endpoint_approaced_messageCallback(const narrow_passage_detection_msgs::NarrowPassageController &msg);


  bool generate_output2( double pos_x, double pos_y, double yaw_, grid_map::GridMap map,
                         geometry_msgs::Pose &pos, int index );

  void create_ray2( double pos_x, double pos_y, double yaw_, grid_map::GridMap map,
                    std::vector<grid_map::Position> &pos_buffer );
  bool is_obstacle( const passage_width_buffer_type &a, grid_map::GridMap map );
  bool lookahead_detection();
  bool robot_detection();
  void ray_detection2( double x, double y, double angle, grid_map::Position robot_position,
                       grid_map::GridMap map );
  void extend_point_publisher( geometry_msgs::Pose mid_pos,geometry_msgs::Pose end_pos , geometry_msgs::Pose extend_pos);
  void adjust_map(grid_map::GridMap &map);

  double calculateDistance( const grid_map::Position &A, const grid_map::Position &B );
  static bool compareByDis( const dis_buffer_type &a, const dis_buffer_type &b );
  static bool compareByWidth( const passage_width_buffer_type &a, const passage_width_buffer_type &b );
  static bool compareByPose( const ray_buffer_type &a, const ray_buffer_type &b );
  bool compute_passage_width2( grid_map::GridMap map, grid_map::Position center, double yaw,
                               std::vector<grid_map::Position> pos_buffer, geometry_msgs::Pose &pos, int index );

  void mark_narrow_passage( grid_map::Position pos1, grid_map::Position pos2 );
  bool isPointOnSegment( const grid_map::Position A, const grid_map::Position B,
                         const grid_map::Position C );
  bool isPointOnSegment( const grid_map::Position A, const grid_map::Position B, float max = 0.99999 );
  int get_path_index( const nav_msgs::Path path_msg, const float distance = 0.7 );
  geometry_msgs::Pose extend_point( geometry_msgs::Pose pose, float distance,  bool extend_or_approach );
  bool finde_intersection_point( std::vector<passage_width_buffer_type> width_buffer,
                                 nav_msgs::Path &msg, geometry_msgs::Pose &pos );
  bool is_on_path( std::vector<passage_width_buffer_type> width_buffer, nav_msgs::Path &msg,
                   geometry_msgs::Pose &pos );
  bool approach_distance( nav_msgs::Odometry robot_pose_msg, geometry_msgs::Pose mid_pose,
                          float &distance, nav_msgs::Path path_msg );


  // void detect_passage2(const nav_msgs::Path::poses poses_);
  int lookahead_detection_count=0;
  ros::Subscriber map_sub;
  ros::Subscriber map_sub2;
  ros::Subscriber path_sub;
  ros::Subscriber endpoint_approached;


  ros::Subscriber pose_sub;
  ros::Subscriber vel_sub;
  grid_map::GridMap elevationmap;
  grid_map::GridMap elevationmap_;
  grid_map::GridMap occupancy_map;
  ros::Duration maxduration;
  ros::Timer mapUpdateTimer_;
  bool getmap = false;
  ros::Publisher map_pub;
  ros::Publisher width_pub;
  ros::Publisher extend_point_pub;
  ros::Publisher approach_distacne_pub;
  cv::Mat input_img;
  grid_map::GridMap outputmap;
  grid_map::GridMap outputmap2;
  geometry_msgs::Pose robot_pose;
  cv::Mat gradient, direction;
  nav_msgs::Odometry robot_pose_msg;
  geometry_msgs::Twist vel_msg;
  grid_map::Matrix grid_data;
  nav_msgs::Path path_msg;
  double robot_roll, robot_pitch, robot_yaw;
  bool tan90 = false;
  bool backward = false;
  bool get_path = false;
  geometry_msgs::Pose mid_pose;
  bool lookahead_narrow_passage_dectected = false;
  bool extended_point = false;

  std::vector<dis_buffer_type> dis_buffer;

  std::vector<passage_width_buffer_type> width_buffer;

  std::vector<ray_buffer_type> ray_buffer;

  std::vector<ray_buffer_type> test_buffer;
  double constrainAngle_mpi_pi( double x )
  {
    x = fmod( x + M_PI, 2.0 * M_PI );
    if ( x < 0 )
      x += 2.0 * M_PI;
    return x - M_PI;
  }

  template<typename Type_, int NChannels_>
  bool addLayerFromImage( const cv::Mat &image, const std::string &layer,
                          grid_map::GridMap &gridMap, const float lowerValue = 0.0,
                          const float upperValue = 1.0, const double alphaThreshold = 0.5 )
  {
    if ( gridMap.getSize()( 0 ) != image.rows || gridMap.getSize()( 1 ) != image.cols ) {
      std::cerr << "Image size does not correspond to grid map size!" << std::endl;
      return false;
    }

    bool isColor = false;
    if ( image.channels() >= 3 )
      isColor = true;
    bool hasAlpha = false;
    if ( image.channels() >= 4 )
      hasAlpha = true;

    cv::Mat imageMono;
    if ( isColor && !hasAlpha ) {
      cv::cvtColor( image, imageMono, CV_BGR2GRAY );
    } else if ( isColor && hasAlpha ) {
      cv::cvtColor( image, imageMono, CV_BGRA2GRAY );
    } else if ( !isColor && !hasAlpha ) {
      imageMono = image;
    } else {
      std::cerr << "Something went wrong when adding grid map layer form image!" << std::endl;
      return false;
    }

    const float mapValueDifference = upperValue - lowerValue;

    float maxImageValue;
    if ( std::is_same<Type_, float>::value || std::is_same<Type_, double>::value ) {
      maxImageValue = 1.0;
    } else if ( std::is_same<Type_, unsigned short>::value ||
                std::is_same<Type_, unsigned char>::value ) {
      maxImageValue = (float)std::numeric_limits<Type_>::max();
    } else {
      std::cerr << "This image type is not supported." << std::endl;
      return false;
    }

    const Type_ alphaTreshold = (Type_)( alphaThreshold * maxImageValue );

    gridMap.add( layer );
    grid_map::Matrix &data = gridMap[layer];

    for ( grid_map::GridMapIterator iterator( gridMap ); !iterator.isPastEnd(); ++iterator ) {
      const grid_map::Index gridMapIndex = *iterator;
      const grid_map::Index imageIndex = iterator.getUnwrappedIndex();

      // Check for alpha layer.
      if ( hasAlpha ) {
        const Type_ alpha =
            image.at<cv::Vec<Type_, NChannels_>>( imageIndex( 0 ), imageIndex( 1 ) )[NChannels_ - 1];
        if ( alpha < alphaTreshold )
          continue;
      }

      // Compute value.
      const Type_ imageValue = imageMono.at<Type_>( imageIndex( 0 ), imageIndex( 1 ) );
      float mapValue = lowerValue + mapValueDifference * ( (float)imageValue / maxImageValue );
      if ( mapValue < 0.01 ) {
        mapValue = NAN;
      }

      data( gridMapIndex( 0 ), gridMapIndex( 1 ) ) = mapValue;
    }

    return true;
  }

public:
  Narrowpassagedetection( ros::NodeHandle &nodeHandle );
  // ~Narrowpassagedetection();
  // void messageCallback(const ::GridMap& map);
  ros::NodeHandle nh;
};

} // namespace narrow_passage_detection

#endif