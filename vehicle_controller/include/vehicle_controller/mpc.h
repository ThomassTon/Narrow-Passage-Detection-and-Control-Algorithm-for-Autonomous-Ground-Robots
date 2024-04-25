#ifndef MPC_H
#define MPC_H

#include <vehicle_controller/MPCParamsConfig.h>
#include <vehicle_controller/controller.h>

#include <narrow_passage_detection_msgs/NarrowPassageController.h>
#include <vehicle_controller/ekf.h>

#include <exception>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/ProcessFile.h>
#include <grid_map_msgs/SetGridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <iostream>

struct robot_range {
  grid_map::Position position;
  // double angle;
  double distance = 0;

  robot_range( grid_map::Position pos, double dis ) : position( pos ), distance( dis ) { }
};

struct cmd_combo{
  double linear_vel;
  double angle_vel;

  double reward;
  double min_distance;
  double angle_diff;
  double dis_diff;
    cmd_combo(double linear, double angle, double r) :linear_vel(linear), angle_vel(angle), reward(r){};
    cmd_combo(double linear, double angle, double r, double m) :linear_vel(linear), angle_vel(angle), reward(r), min_distance(m){};

  cmd_combo(double linear, double angle, double r, double min, double d, double an) :linear_vel(linear), angle_vel(angle), reward(r), min_distance(min), angle_diff(an), dis_diff(d){}
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

struct dis_buffer_type {
  double distance;
  grid_map::Index index;
  grid_map::Position position;
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
  void controllerParamsCallback(vehicle_controller::MPCParamsConfig & config, uint32_t level);

  // bool configure() override;
  dynamic_reconfigure::Server<vehicle_controller::MPCParamsConfig> * dr_controller_params_server;

  inline std::string getName() override { return "MPC"; }
  ros::NodeHandle nh_dr_paramsss;
  ros::Subscriber stateSubscriber;
  ros::Subscriber map_sub;
  void map_messageCallback2( const grid_map_msgs::GridMap &msg );
  grid_map::GridMap occupancy_map;
  grid_map::GridMap dist_map;
  grid_map::GridMap map;


  bool get_map = false;
  void predict_distance( const geometry_msgs::Pose robot_pose );
  void predict_position( const geometry_msgs::Pose robot_pose, double linear_vel, double angluar_vel,geometry_msgs::Pose &predict_pose );
  void create_robot_range( const geometry_msgs::Pose robot_pose );
  void obsticke_distance( std::vector<robot_range> &robot, geometry_msgs::Pose robot_pose );
  double obsticke_distance( geometry_msgs::Pose robot_pose);

  double compute_distance( grid_map::Position pos1, grid_map::Position pos2 );
  static bool compareByDistance( robot_range &a, robot_range &b );
  double get_min_distance();
  double crossProduct( const Vector_ &AB, const Point_ &C );
  double pd_controller( double &last_e_front, double &last_e_back, const double p, const double d );
  double pd_controller2( geometry_msgs::Pose clost_pose, double &last_e, const double p,
                         const double d );
  bool collision_detection(const geometry_msgs::Pose robot_pose );
  double calc_local_path(geometry_msgs::Pose &lookahead);
  int calcClosestPoint();

  double ray_detection( double angle, grid_map::Position robot_position, grid_map::GridMap map );
  bool isPointOnSegment( const grid_map::Position A, const grid_map::Position B );
  static bool compareByDis( const dis_buffer_type &a, const dis_buffer_type &b );
  static bool compareByReward( const cmd_combo &a, const cmd_combo &b );

  bool compute_cmd(double &linear_vel, double & angluar_vel);
  double width = 0.53;  // 0.52
  double length = 0.75; // 0.72
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
  double last_error_front = 0;
  double last_error_back = 0;
  double last_error_mid = 0;

  double p;
  double d;
  double lookahead;
  double p2;
  double d2;
  double dt_;
  double w_l;
  double w_a;
  double w_l_c;
  double w_a_c;
  double w_min;

  geometry_msgs::PointStamped closest_point;
  double rot_vel_dir, lin_vel_dir;
  double local_path_radius;
  double alignment_angle;

  double lqr_y_error, lqr_x_error;
  double lqr_angle_error;

  geometry_msgs::Twist lqr_last_cmd;
  double lqr_last_y_error;
  double lqr_last_angle_error;

  std::vector<dis_buffer_type> dis_buffer;

  double angluar_array[21]={0,0.05,0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45,0.5 ,-0.05, -0.1, -0.15, -0.2, -0.25, -0.3, -0.35, -0.4, -0.45, -0.5};
  double linear_array[5]={0.1, 0.1, 0.1, 0.15, 0.2};

protected:
  void computeMoveCmd() override;
  void reset() override;
  void stateCallback( const nav_msgs::Odometry odom_state );
};

#endif // LQR_CONTROLLER_H
