#ifndef MPC_H
#define MPC_H

#include <vehicle_controller/LqrControllerParamsConfig.h>
#include <vehicle_controller/controller.h>

#include <narrow_passage_detection_msgs/NarrowPassageController.h>
#include <vehicle_controller/ekf.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/ProcessFile.h>
#include <grid_map_msgs/SetGridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <exception>  
#include <iostream>

struct robot_range {
  grid_map::Position position;
  // double angle;
  double distance = 0;

  robot_range( grid_map::Position pos, double dis ) : position( pos ), distance( dis ) { }
};
struct robot_ladar {
  double left_distance;
  double right_distance;
  double front_distance;
  double back_distance;
};
struct Point_ {
    double x, y;
};

struct Vector_ {
    Point_ start;
    Point_ end;
};
class MPC_Controller : public Controller
{
public:
  MPC_Controller( ros::NodeHandle &nh_ );
  ~MPC_Controller() override;

  //   bool configure() override;

  inline std::string getName() override { return "MPC"; }
  ros::NodeHandle nh_dr_params;
  ros::Subscriber stateSubscriber;
  ros::Subscriber map_sub;
  void map_messageCallback2( const nav_msgs::OccupancyGrid &msg );
  grid_map::GridMap occupancy_map;
  bool get_map = false;
  void predict_distance( const geometry_msgs::Pose robot_pose );
  void create_robot_range( const geometry_msgs::Pose robot_pose);
  void obsticke_distance( std::vector<robot_range> &robot, grid_map::GridMap map );
  double compute_distance( grid_map::Position pos1, grid_map::Position pos2 );
  static bool compareByDistance( robot_range &a, robot_range &b );
  void get_min_distance( robot_ladar &rl );
  double crossProduct(const Vector_& AB, const Point_& C);
  double pd_controller(const double e, double &last_e, const double p, const double d);
  void collision_detection(double &linear_vel, double &angluar_vel, double dt);
    double width = 0.52; //0.52
  double length = 0.72;   //0.72
  geometry_msgs::PoseStamped pose;
  geometry_msgs::Vector3Stamped velocity_linear;
  geometry_msgs::Vector3Stamped velocity_angular;
  std::vector<robot_range> robot_right;
  std::vector<robot_range> robot_left;
  std::vector<robot_range> robot_front;
  std::vector<robot_range> robot_back;
  std::vector<robot_range> robot_middle;
  double right_min_distance;
  double left_min_distance;
  double front_min_distance;
  double back_min_distance;
  double last_error_front=0;
  double last_error_back=0;
  double last_error_mid=0;
protected:
  void computeMoveCmd() override;
  void reset() override;
  void stateCallback( const nav_msgs::Odometry odom_state );
};

#endif // LQR_CONTROLLER_H
