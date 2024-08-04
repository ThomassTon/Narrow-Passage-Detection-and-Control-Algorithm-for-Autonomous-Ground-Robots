#include <vehicle_controller/mpc.h>

#include "iostream"
MPC_Controller::MPC_Controller( ros::NodeHandle &nh_ )
    : Controller( nh_ ), nh_dr_paramsss( "~/mpc_params" )
{
  // LQR parameters
  map_sub = nh_.subscribe( "/move_base_lite_node/debug_planning_map", 1,
                           &MPC_Controller::map_messageCallback2, this );
  // map_sub2 = nh_.subscribe( "/narrow_passage_map", 1,
  //                           &MPC_Controller::map_messageCallback22, this );
  // smoothPathPublisher = nh_.advertise<nav_msgs::Path>( "smooth_path22", 1, true );
  endpoint_approaced = nh_.subscribe( "/narrow_passage_controller_node/endpoint_approached", 1,
                                      &MPC_Controller::endpoint_approaced_messageCallback, this );
  smoothPath_sub = nh_.subscribe( "/narrow_passage_controller_node/smooth_path_circle", 1,
                                  &MPC_Controller::smoothPath_messageCallback, this );
  controllerTypeSwitch = nh.subscribe( "/narrow_passage_detected", 1, &MPC_Controller::controllerTypeSwitchCallback, this );

  switch_to_smoothpath = true;
  get_smoothpath = false;
  // lookahead = 0.4;  // stateSubscriber = nh.subscribe( "/odom", 1, &MPC_Controller::stateCallback, this,
  // ros::TransportHints().tcpNoDelay( true ) );
  //   lqr_params_narrow = nh_dr_params.subscribe("/lqr_params_narrow",1, &Lqr_Controller::lqr_params_callback,this);
  dr_controller_params_server =
      new dynamic_reconfigure::Server<vehicle_controller::MPCParamsConfig>( nh_dr_paramsss );
  dr_controller_params_server->setCallback(
      boost::bind( &MPC_Controller::controllerParamsCallback, this, _1, _2 ) );
  // follow_path_server_.reset(new actionlib::ActionServer<move_base_lite_msgs::FollowPathAction>(nh_, "/controller/follow_path2",
  //                                                                                              boost::bind(&Controller::followPathGoalCallback, this, _1),
  //                                                                                              boost::bind(&Controller::followPathPreemptCallback, this, _1),
  //                                                                                              false));
  // follow_path_server_->start();

  // ROS_INFO("CONTROLLER INIT!!!!!!!!!!!!!!!!!!!!!!!!!11\n\n\n\n\n\n\n");
  
  // std::cout<<"start to use mpc !!!!!!!!!!!\n\n\n\n\n\n";
}

MPC_Controller::~MPC_Controller()
{
  if ( dr_controller_params_server ) {
    nh_dr_paramsss.shutdown();
    dr_controller_params_server->clearCallback();
  }
}

void MPC_Controller::reset()
{
  Controller::reset();
  //   lqr_y_error = 0;
  //   lqr_angle_error = 0;
  //   state = INACTIVE;
  //   current = 0;
}
// bool MPC_Controller::configure()
// {
//   Controller::configure();

//   dr_controller_params_server = new dynamic_reconfigure::Server<vehicle_controller::MPCParamsConfig>(nh_dr_paramsss);
//   dr_controller_params_server->setCallback(boost::bind(&MPC_Controller::controllerParamsCallback,
//   this, _1, _2)); return true;
// }
void MPC_Controller::controllerParamsCallback( vehicle_controller::MPCParamsConfig &config,
                                               uint32_t level )
{
  lookahead = config.lookahead_distance;
  lookahead_angle = config.lookahead_angle_distance;

  p = config.P;
  d = config.D;
  p2 = config.P2;
  d2 = config.D2;
  dt_ = config.dt;
  w_a = config.w_a;
  w_l = config.w_l;
  w_l_c = config.w_l_c;
  w_a_c = config.w_a_c;
  w_min = config.w_min;
  dt_c = config.dt_c;

  std::cout<<"look ahead  "<<lookahead<<"\n\n\n\n\n";
  // update_boundingbox_size();
  // lqr_r = config.R;
}
void MPC_Controller::controllerTypeSwitchCallback(const narrow_passage_detection_msgs::NarrowPassageDetection &msg){
  if(msg.narrow_passage_detected){
    switch_to_smoothpath = true;
  }
  else{
      switch_to_smoothpath = false;

    }    // ROS_INFO_STREAM("controlstype: "<<controller_type_<<"\n\n\n\n\n\n\n\n\n");
}

void MPC_Controller::computeMoveCmd()
{
  // std::cout << "x:  " << robot_control_state.pose.position.x << "\n";
  // std::cout<<"compute mpc cmd!!!!!!!!!!!!!\n\n\n\n\n\n";
  ros::Time current_time = ros::Time::now();
  ros::Duration interval = current_time - last_time;
  // ROS_INFO("Function called, interval: %f seconds", interval.toSec());
  geometry_msgs::Twist cmd;
  double linear_vel = 0.0;
  double angular_vel = 0.0;

  // cmd.linear.x = 0.1;
  cmd.angular.z = 0.0;
  // compute_cmd( linear_vel, angular_vel );

  if ( switch_to_smoothpath == true && get_smoothpath ==true ) {
    current_path_ = adjust_path_;
  }
  else{
    current_path_ = current_path;
  }
  // get_smoothpath=false;
  // optimal_path( robot_control_state.pose, 1.0 );

  // compute_cmd2( linear_vel, angular_vel );

  /*--------------------------------------------------------*/
  compute_cmd( linear_vel, angular_vel );
  smoothPathPublisher_narrow.publish( current_path_ );
  /*  -----------------------------------------------------------------*/
  cmd.angular.z = angular_vel;
  cmd.linear.x = linear_vel;
  // send cmd to control interface
  vehicle_control_interface_->executeTwist( cmd, robot_control_state, yaw, pitch, roll );
  last_time = ros::Time::now();
}

void MPC_Controller::stateCallback( const nav_msgs::Odometry odom_state )
{
  //   updateRobotState( odom_state );
  //   geometry_msgs::Pose predict_pose;
  //   predicteRobotState(predict_pose, 0.0, 0.0);
}

void MPC_Controller::endpoint_approaced_messageCallback(const narrow_passage_detection_msgs::NarrowPassageController &msg )
{
  if ( msg.approached_endpoint ) {
    get_smoothpath = false;
  }
}

void MPC_Controller::map_messageCallback2( const grid_map_msgs::GridMap &msg )
{
  grid_map::GridMapRosConverter::fromMessage( msg, occupancy_map ); // distance_transform occupancy
  get_debugging_map = true;

}

void MPC_Controller::smoothPath_messageCallback( const nav_msgs::Path &msg )
{
  // ROS_INFO("get the new path call back !!!!!!!!!\n\n\n\n\n\n\n\n");
  ros::Time time_sub = msg.header.stamp;
  ros::Time time_now = ros::Time::now();
  ros::Duration timd_diff = time_now - time_sub;
  // std::cout<<"time_diss"<<timd_diff.toSec()<<"\n\n\n\n\n";
  if(timd_diff.toSec()<3.0){
    get_smoothpath = true;
    adjust_path_ = msg;
  }
}


void MPC_Controller::appro_integral(double &x, double &y, double dt, double yaw, double linear_vel, double angluar_vel){
  double theta = yaw;
  
  // for(double t=0.1; t<dt; t+=0.1){
    
  //   x += cos(theta_)*0.1*linear_vel;
  //   y += sin(theta_)*0.1*linear_vel;
  //   theta += t*angluar_vel;
  // }
  double x_1 = cos(yaw)*dt*linear_vel;
  double y_1 = sin(yaw)*dt*linear_vel;
  double x_2 = cos(yaw+angluar_vel*dt)*dt*linear_vel;
  double y_2 = sin(yaw+angluar_vel*dt)*dt*linear_vel;

  x += (x_1+x_2)/2.0;
  y += (y_1+y_2)/2.0;
  // std::cout<<"failed !!!!!!!!!!!!!!!!!!!!!!!1\n\n\n\n\n\n\n\n\n";
}

void MPC_Controller::predict_position( const geometry_msgs::Pose robot_pose, double linear_vel,
                                       double angluar_vel, geometry_msgs::Pose &predict_pose , const double dt)
{
  double roll_, pitch_, yaw_;
  tf::Quaternion q( robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z,
                    robot_pose.orientation.w );
  tf::Matrix3x3 m( q );
  m.getRPY( roll_, pitch_, yaw_ );
  double theta = angluar_vel * dt + yaw_;
  // double linear_vel_ = linear_vel; // robot_control_state.velocity_linear.x;
  // std::cout<<"liner_vel:  "<<linear_vel_<<"\n\n\n";
  // double x = robot_pose.position.x + cos( ( theta + yaw_ ) / 2.0 ) * dt_ * linear_vel_;
  // double y = robot_pose.position.y + sin( ( theta + yaw_ ) / 2.0 ) * dt_ * linear_vel_;

  double x = robot_pose.position.x;
  double y = robot_pose.position.y;
  appro_integral(x, y, dt ,yaw_, linear_vel, angluar_vel);

  roll_ = 0.0;  // Roll角（绕X轴旋转）
  pitch_ = 0.0; // Pitch角（绕Y轴旋转）
  yaw_ = theta; // Yaw角（绕Z轴旋转）

  // 创建tf2 Quaternion对象
  tf2::Quaternion quat;
  quat.setRPY( roll_, pitch_, yaw_ );
  predict_pose.position.x = x;
  predict_pose.position.y = y;
  predict_pose.orientation.x = quat.getX();
  predict_pose.orientation.y = quat.getY();
  predict_pose.orientation.z = quat.getZ();
  predict_pose.orientation.w = quat.getW();
}

void MPC_Controller::create_robot_range( const geometry_msgs::Pose robot_pose, grid_map::Position &p_front_right, grid_map::Position &p_front_left, grid_map::Position 
&p_back_right, grid_map::Position &p_back_left )
{
  double roll_, pitch_, yaw_;
  tf::Quaternion q( robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z,
                    robot_pose.orientation.w );
  tf::Matrix3x3 m( q );
  m.getRPY( roll_, pitch_, yaw_ );
  double x = robot_pose.position.x; //+ 0.2*std::cos(yaw);
  double y = robot_pose.position.y; //+ 0.2*std::sin(yaw);
  grid_map::Position p_front_right_(
      x + std::cos( yaw_ ) * 0.5 * length + std::cos( yaw_ - M_PI / 2.0 ) * 0.5 * width,
      y + std::sin( yaw_ ) * 0.5 * length + std::sin( yaw_ - M_PI / 2.0 ) * 0.5 * width );
  grid_map::Position p_front_left_(
      x + std::cos( yaw_ ) * 0.5 * length + std::cos( yaw_ + M_PI / 2.0 ) * 0.5 * width,
      y + std::sin( yaw_ ) * 0.5 * length + std::sin( yaw_ + M_PI / 2.0 ) * 0.5 * width );
  grid_map::Position p_back_right_( x + std::cos( yaw_ + M_PI ) * 0.5 * length +
                                       std::cos( yaw_ + M_PI + M_PI / 2.0 ) * 0.5 * width,
                                   y + std::sin( yaw_ + M_PI ) * 0.5 * length +
                                       std::sin( yaw_ + M_PI + M_PI / 2.0 ) * 0.5 * width );
  grid_map::Position p_back_left_( x + std::cos( yaw_ + M_PI ) * 0.5 * length +
                                      std::cos( yaw_ + M_PI - M_PI / 2.0 ) * 0.5 * width,
                                  y + std::sin( yaw_ + M_PI ) * 0.5 * length +
                                      std::sin( yaw_ + M_PI - M_PI / 2.0 ) * 0.5 * width );

  p_front_right = p_front_right_;
  p_front_left = p_front_left_;
  p_back_right = p_back_right_;
  p_back_left = p_back_left_;
  // grid_map::Position p_mid_right( 0.5 * ( p_front_right[0] + p_back_right[0] ),
  //                                 0.5 * ( p_front_right[1] + p_back_right[1] ) );
  // grid_map::Position p_mid_left( 0.5 * ( p_front_left[0] + p_back_left[0] ),
  //                                0.5 * ( p_front_left[1] + p_back_left[1] ) );

  /*create points for each side*/
  /*based on two point to genarate a middle point, do this step as a loop*/
  // robot_front.clear();
  // robot_back.clear();

  // robot_front.push_back( robot_range( p_front_right, 0.0 ) );
  // robot_front.push_back( robot_range( p_front_left, 0.0 ) );
  // robot_back.push_back( robot_range( p_back_right, 0.0 ) );
  // robot_back.push_back( robot_range( p_back_left, 0.0 ) );
  // robot_middle.push_back( robot_range( p_mid_right, 0.0 ) );
  // robot_middle.push_back( robot_range( p_mid_left, 0.0 ) );
}

bool MPC_Controller::finde_next_sol(geometry_msgs::Pose predict_pos, double lin_vel_dir){
  std::vector<bool> collision_buffer;
  for ( double i = 0.0; i < 0.7; i+=0.1 ) {
    double ang_vel = i;
    double j_min = 0.00;
    if ( std::abs( ang_vel ) < 0.20 ) {
      j_min = -0.01;
    } else {
      j_min = -0.01;
    }

    double lin_vel = 0.1*lin_vel_dir;
    geometry_msgs::Pose predict_pos2;

    predict_position( predict_pos, lin_vel, ang_vel, predict_pos2,dt_c );
    grid_map::Position p_front_right;
    grid_map::Position p_front_left;
    grid_map::Position p_back_right;
    grid_map::Position p_back_left;
    create_robot_range( predict_pos2, p_front_right, p_front_left,p_back_right, p_back_left);
    // bool collision=false;
    bool collision = collision_detection( predict_pos2, 0.3, p_front_right, p_front_left,p_back_right, p_back_left);
    collision_buffer.push_back(collision);
    
    ang_vel = -i;
   
    // lin_vel = j*lin_vel_dir;

    predict_position( predict_pos, lin_vel, ang_vel, predict_pos2,dt_c );

    create_robot_range( predict_pos2, p_front_right, p_front_left,p_back_right, p_back_left);
    // bool collision=false;
    collision = collision_detection( predict_pos2, 0.3, p_front_right, p_front_left,p_back_right, p_back_left);
    collision_buffer.push_back(collision);
    
  }
  for (bool value : collision_buffer) {
    // 处理 value
    if(value==false){
      return true;
    }
  }

  return false;
}
bool MPC_Controller::compute_cmd( double &linear_vel, double &angluar_vel )
{
  geometry_msgs::Pose lookaheadPose;
  geometry_msgs::Pose lookaheadPose2;
  geometry_msgs::Pose lookaheadPose_angle;
  calc_local_path( lookaheadPose, lookahead );
  lookaheadPose2 = lookaheadPose;
  // calc_local_path( lookaheadPose2, 0.4);
  calc_local_path( lookaheadPose_angle, lookahead_angle);

  double roll_, pitch_, yaw_;
  tf::Quaternion q( robot_control_state.pose.orientation.x, robot_control_state.pose.orientation.y,
                    robot_control_state.pose.orientation.z, robot_control_state.pose.orientation.w );
  tf::Matrix3x3 m( q );
  m.getRPY( roll_, pitch_, yaw_ );

  double roll_2, pitch_2, yaw_2;
  tf::Quaternion q2( lookaheadPose.orientation.x, lookaheadPose.orientation.y,
                    lookaheadPose.orientation.z, lookaheadPose.orientation.w );
  tf::Matrix3x3 m2( q2 );
  m2.getRPY( roll_2, pitch_2, yaw_2 );


  double angle_current_to_waypoint = std::atan2( lookaheadPose2.position.y - robot_control_state.pose.position.y, lookaheadPose2.position.x - robot_control_state.pose.position.x );
  double angle_to_carrot = constrainAngle_mpi_pi(angle_current_to_waypoint - yaw_);
  // std::cout<<"current_angle_diff:  "<<angle_to_carrot<<"\n\n\n\n";

  double lin_vel_dir =1.00;
  if (reverseAllowed()){
    if(fabs(angle_to_carrot) > M_PI/2.0){
      lin_vel_dir = -1.00;
      yaw_ += M_PI;
      if(yaw_> M_PI*2.0){
        yaw_ -= M_PI*2.0;
      }
      double angle_to_carrot = constrainAngle_mpi_pi(angle_current_to_waypoint - yaw_);
    }
    else{
      // ROS_INFO(" go gogo \n\n\n\n\n\n");

    }
  }
  std::vector<node> cmd_buffer;
  double init_anlgle_vel = robot_control_state.velocity_angular.z;
  for ( double i = -0.3; i < 0.3; i+=0.05 ) {
    double ang_vel = i+init_anlgle_vel;
    double j_min = 0.00;
    if ( std::abs( ang_vel ) < 0.20 ) {
      j_min = -0.01;
    } else {
      j_min = -0.01;
    }
    for ( double j=0.25; j > 0.04; j -= 0.025 ) {
      double lin_vel = j*lin_vel_dir;
      geometry_msgs::Pose predict_pos;
      geometry_msgs::Pose predict_pos2;

      predict_position( robot_control_state.pose, lin_vel, ang_vel, predict_pos2,dt_c );
      predict_position( robot_control_state.pose, lin_vel, ang_vel, predict_pos,dt_ );
      double min = 0.0;
      double dis = std::sqrt( std::pow( lookaheadPose.position.x - predict_pos.position.x, 2 ) +
                              std::pow( lookaheadPose.position.y - predict_pos.position.y, 2 ) );
      double roll3, pitch3, yaw3;
      tf::Quaternion q3( predict_pos.orientation.x, predict_pos.orientation.y,
                          predict_pos.orientation.z, predict_pos.orientation.w );
      tf::Matrix3x3 m3( q3 );
      m3.getRPY( roll3, pitch3, yaw3 );
      if(lin_vel_dir<0){
        yaw3 += M_PI;
        if(yaw3> M_PI*2.0){
          yaw3 -=M_PI*2.0;
        }
      }
      double angle_to_waypoint = std::atan2( lookaheadPose_angle.position.y - predict_pos.position.y,
                                              lookaheadPose_angle.position.x - predict_pos.position.x );
      double angle = std::abs( constrainAngle_mpi_pi( yaw3 - yaw_2 ) );
      double angle_2 = std::abs( constrainAngle_mpi_pi( yaw3 - angle_to_waypoint ) );
      // double angle_2 = std::abs( constrainAngle_mpi_pi( yaw3 - alignment_angle ) );

      double reward = -w_a * (angle_2) - w_l * dis + w_min * min + 0.01 * lin_vel-0.0*std::abs(ang_vel);
      node cmd_( lin_vel, ang_vel, reward, predict_pos2 );
      cmd_buffer.push_back( cmd_ );
      
    }
  }

  if ( cmd_buffer.size() > 0 ) {
    std::sort( cmd_buffer.begin(), cmd_buffer.end(), MPC_Controller::compareByReward );

    for(auto value : cmd_buffer){
      grid_map::Position p_front_right;
      grid_map::Position p_front_left;
      grid_map::Position p_back_right;
      grid_map::Position p_back_left;
      create_robot_range( value.predict_pos2, p_front_right, p_front_left,p_back_right, p_back_left);
      bool collision = collision_detection( value.predict_pos2, 0.5, p_front_right, p_front_left,p_back_right, p_back_left);
      if(collision==false){
        linear_vel = value.linear_vel;
        angluar_vel = value.angle_vel;
        return true;
      }
    }

    
    // std::cout << "distacne  : " << cmd_buffer[0].min_distance << "\n";
  } else {
     linear_vel=0;
     angluar_vel=0;
  }
  

  return true;
}

// void MPC_Controller::obsticke_distance( std::vector<robot_range> &robot, grid_map::GridMap map )
// {

//   grid_map::Position robot_pos( robot[0].position );
//   robot[0].distance = ray_detection( -( M_PI / 2.0 ), robot_pos, map );

//   grid_map::Position robot_pos2( robot[1].position );
//   robot[1].distance = ray_detection( M_PI / 2.0, robot_pos2, map );
// }

void MPC_Controller::obsticke_distance( std::vector<robot_range> &robot,
                                        geometry_msgs::Pose robot_pose )
{


}

double MPC_Controller::obsticke_distance( geometry_msgs::Pose robot_pose )
{

  return 0;
  // std::cout<<"distance    "<< robot_front[0].distance<<"\n";
}

double MPC_Controller::ray_detection( double angle, grid_map::Position robot_position,
                                      grid_map::GridMap map )
{
  double angles[3];
  quaternion2angles( robot_control_state.pose.orientation, angles );

  double yaw = angles[0] + angle;
  double x = 3.0 * std::cos( yaw );
  double y = 3.0 * std::sin( yaw );
  dis_buffer.clear();
  for ( grid_map::GridMapIterator iterator( map ); !iterator.isPastEnd(); ++iterator ) {
    const grid_map::Index index( *iterator );
    const float &value = map.get( "occupancy" )( index( 0 ), index( 1 ) ); // elevation
    if ( value > 0 && value != NAN ) {
      grid_map::Position position;
      map.getPosition( index, position );
      grid_map::Position position1( position[0] - robot_position[0], position[1] - robot_position[1] );
      grid_map::Position position2( x, y );
      // double diff = isPointOnSegment(position1,position2,length);
      if ( isPointOnSegment( position1, position2 ) ) {
        double dis = compute_distance( position, robot_position );
        dis_buffer.push_back( { dis, index, position } );
      }
    }
  }

  if ( !dis_buffer.empty() ) {

    std::sort( dis_buffer.begin(), dis_buffer.end(), MPC_Controller::compareByDis );
    return dis_buffer[0].distance;
  } else {
    return 0.3;
  }
}
bool MPC_Controller::compareByDis( const dis_buffer_type &a, const dis_buffer_type &b )
{
  return a.distance < b.distance;
}

bool MPC_Controller::isPointOnSegment( const grid_map::Position A, const grid_map::Position B )
{
  double vectorOA_x = A[0] - 0;
  double vectorOA_y = A[1] - 0;
  double vectorOB_x = B[0] - 0;
  double vectorOB_y = B[1] - 0;

  double dotProduct = vectorOA_x * vectorOB_x + vectorOA_y * vectorOB_y;

  grid_map::Position O( 0, 0 );
  double lengthOB = compute_distance( O, B );
  double lenghtOA = compute_distance( O, A );
  // 判断点 C 是否在线段 AB 上
  return ( dotProduct / ( lenghtOA * lengthOB ) > 0.8 );
}

double MPC_Controller::compute_distance( grid_map::Position pos1, grid_map::Position pos2 )
{
  double x1 = pos1[0];
  double y1 = pos1[1];
  double x2 = pos2[0];
  double y2 = pos2[1];
  return std::sqrt( std::pow( x1 - x2, 2 ) + std::pow( y1 - y2, 2 ) );
}

bool MPC_Controller::compareByDistance( robot_range &a, robot_range &b )
{
  return a.distance < b.distance;
}

bool MPC_Controller::compareByReward( const node &a, const node &b )
{
  return a.reward > b.reward;
}

double MPC_Controller::get_min_distance()
{

  // std::sort( robot_back.begin(), robot_back.end(), MPC_Controller::compareByDistance );
  // std::sort( robot_front.begin(), robot_front.end(), MPC_Controller::compareByDistance );
  // std::sort (robot_middle.begin(), robot_middle.end(), MPC_Controller::compareByDistance);
  // std::sort(robot_front.begin(),robot_right.end(),NarrowPassageController::compareByDistance);
  // std::sort(robot_back.begin(),robot_right.end(),NarrowPassageController::compareByDistance);

  // std::vector<robot_range> buffer;
  // buffer.push_back(robot_back[0]);
  // buffer.push_back(robot_front[0]);
  // buffer.push_back(robot_middle[0]);

  // std::sort(buffer.begin(),buffer.end(), MPC_Controller::compareByDistance);

  return ( 0 );
}

double MPC_Controller::crossProduct( const Vector_ &AB, const Point_ &C )
{
  Point_ A = AB.start;
  Point_ B = AB.end;
  return ( B.x - A.x ) * ( C.y - A.y ) - ( B.y - A.y ) * ( C.x - A.x );
  // >0 left, <0 right
}



void MPC_Controller::pd_controller2( geometry_msgs::Pose clost_pose, double &last_e, const double p, const double d, double &linear_vel, double &angular_vel )
{
  // create_robot_range( robot_control_state.pose );
  // grid_map::Position middle( ( robot_front[0].position[0] + robot_front[1].position[0] ) / 2.0,
  //                            ( robot_front[0].position[1] + robot_front[1].position[1] ) / 2.0 );
  double roll, pitch, yaw;
  tf::Quaternion q( robot_control_state.pose.orientation.x, robot_control_state.pose.orientation.y,
                    robot_control_state.pose.orientation.z, robot_control_state.pose.orientation.w );
  tf::Matrix3x3 m( q );
  m.getRPY( roll, pitch, yaw );

  double roll2, pitch2, yaw2;
  tf::Quaternion q2( clost_pose.orientation.x, clost_pose.orientation.y, clost_pose.orientation.z,
                     clost_pose.orientation.w );
  tf::Matrix3x3 m2( q2 );
  m2.getRPY( roll2, pitch2, yaw2 );

  // geometry_msgs::Pose lookahead2;
  // calc_local_path(lookahead2, 0.2);

  //   double roll3, pitch3, yaw3;
  // tf::Quaternion q3( lookahead2.orientation.x, lookahead2.orientation.y, lookahead2.orientation.z,
  //                    lookahead2.orientation.w );
  // tf::Matrix3x3 m3( q3 );
  // m3.getRPY( roll3, pitch3, yaw3 );

  double angle_to_waypoint =
      std::atan2( clost_pose.position.y - robot_control_state.pose.position.y,
                  clost_pose.position.x - robot_control_state.pose.position.x );

  // double angle_to_waypoint2 =
  //     std::atan2( lookahead2.position.y - robot_control_state.pose.position.y,
  //                 lookahead2.position.x - robot_control_state.pose.position.x );
  // double angle_to_waypoint =
  //     std::atan2( clost_pose.position.y - middle[1],
  //                 clost_pose.position.x - middle[0] );

  double e = constrainAngle_mpi_pi( angle_to_waypoint - yaw ) +
             constrainAngle_mpi_pi( yaw2 - yaw ); // constrainAngle_mpi_pi( -yaw + yaw2 )
  // std::cout << "angle _e :" << yaw << "     " << yaw2 << "\n\n\n";
  double ed = e - last_e;
  double out = p * e + d * ed;
  out /= 2.0;
  // out *=0.05;
  out = ( std::abs( out ) > 0.5 ) ? ( ( out > 0 ) ? 0.5 : -0.5 ) : out;
  last_e = e;
  // error = e;
  angular_vel = out;
  linear_vel = ( 0.5 - std::abs( out ) ) / 0.5 * 0.3;
  linear_vel = ( linear_vel < 0.1 ) ? 0.1 : linear_vel;
}

bool MPC_Controller::collision_detection( const geometry_msgs::Pose robot_pose, double threshold, const grid_map::Position &p_front_right, const grid_map::Position &p_front_left, const grid_map::Position 
&p_back_right,  const grid_map::Position &p_back_left )
{
  if(get_elevation_map){
    double x = robot_pose.position.x;
    double y = robot_pose.position.y;

    grid_map::Position robot_position2( x, y );
    grid_map::Length length2( 2, 2 );
    bool isSuccess;
    grid_map::GridMap submap = elevation_map.getSubmap( robot_position2, length2, isSuccess );


    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;

    grid_map::Index front_right;
    submap.getIndex( p_front_right, front_right );
    grid_map::Index back_right;
    submap.getIndex( p_back_right, back_right );
    grid_map::Index front_left;
    submap.getIndex( p_front_left, front_left );
    grid_map::Index back_left;
    submap.getIndex( p_back_left, back_left );

    int count = 0;
    for ( grid_map::LineIterator iterator( submap, front_right, back_right ); !iterator.isPastEnd();
          ++iterator ) {
      // std::cout<<"value : "<<submap.at( "elevation", *iterator );
      double value = submap.at( "elevation", *iterator );
      if ( value > threshold && value != NAN ) {
          return true;
        
      }
    }
    count = 0;
    for ( grid_map::LineIterator iterator( submap, front_left, back_left ); !iterator.isPastEnd();
          ++iterator ) {
      // std::cout<<"value : "<<occupancy_map.at( "occupancy", *iterator )<<"  ";
      double value = submap.at( "elevation", *iterator );
      if ( value > threshold && value != NAN ) {
      
          return true;
        
      }
    }
    count = 0;
    for ( grid_map::LineIterator iterator( submap, front_left, front_right ); !iterator.isPastEnd();
          ++iterator ) {
      // std::cout<<"value : "<<occupancy_map.at( "occupancy", *iterator );
      double value = submap.at( "elevation", *iterator );
      if ( value > threshold && value != NAN ) {

  
          return true;
        
      }
    }
    count = 0;
    for ( grid_map::LineIterator iterator( submap, back_left, back_right ); !iterator.isPastEnd();
          ++iterator ) {
      double value = submap.at( "elevation", *iterator );
      if ( value > threshold && value != NAN ) {
     
          return true;
        
      }
    }
  }
 

  return false;
}

double MPC_Controller::optimal_path( geometry_msgs::Pose &lookahead_pose, double distance )
{

  if(get_elevation_map){
    int psize = current_path_.poses.size();
    int path_po_lenght;

    path_po_lenght = 0;
    // calculate path_po_lenght -> number of waypoints in the carrot distance
    int collision_points = 0;
    for ( int i = 1; i < psize; i++ ) {
      double curr_dist = std::sqrt(
          std::pow( current_path_.poses[0].pose.position.x - current_path_.poses[i].pose.position.x, 2 ) +
          std::pow( current_path_.poses[0].pose.position.y - current_path_.poses[i].pose.position.y,
                    2 ) );

      if ( curr_dist > distance ) // search for points
      {
        break;
      } else {
        path_po_lenght = path_po_lenght + 1;
        grid_map::Position center( current_path_.poses[path_po_lenght].pose.position.x,
                                  current_path_.poses[path_po_lenght].pose.position.y );
        grid_map::Length length2( 1, 1 );
        bool success;
        grid_map::GridMap sub_map = elevation_map.getSubmap( center, length2, success );

        for ( grid_map::SpiralIterator iterator( sub_map, center, 0.375 ); !iterator.isPastEnd(); // 0.275
              ++iterator ) {
          double value = sub_map.at( "elevation", *iterator );
          if ( value > 0.75 && value != NAN ) {
            // std::cout<<"collision!!!!!!!\n\n\n";
            grid_map::Position pos;
            const grid_map::Index index( *iterator );
            sub_map.getPosition( index, pos );
            // std::cout << "dis: " << compute_distance( pos, center ) << "\n";
            collision_points++;

            adjust_pos( path_po_lenght, compute_distance( pos, center ), collision_points );
            // current_path.poses[path_po_lenght + st_point].pose.position.x =;
            smoothPathPublisher.publish( current_path_ );
            break;
          }
        }
      }
    }
  }
  // std::cout << "collsiion point: " << collision_points << "\n\n\n";
  return ( 0.01 );
}

bool MPC_Controller::adjust_pos( int index, double radius, int collision_points )
{

  // for ( int i = 0; i <index; i++ ) {
  geometry_msgs::Pose &center = current_path_.poses[index].pose;
  double roll, pitch, yaw;
  tf::Quaternion q( center.orientation.x, center.orientation.y, center.orientation.z,
                    center.orientation.w );
  tf::Matrix3x3 m( q );
  m.getRPY( roll, pitch, yaw );
  // grid_map::Position center_( center.position.x, center.position.y );
  // bool success;
  // grid_map::Length length2( 1, 1 );

  // grid_map::GridMap sub_map = map.getSubmap( center_, length2, success );
  double diff;
  if ( radius < 0.31 ) {
    diff = std::abs( radius - 0.31 );
  } else {
    diff = 0.02;
  }
  // double diff = std::abs( radius - 0.3 );
  // diff = 0.02;
  // std::cout<<"diff:  "<<diff<<"\n";
  center.position.x += diff * cos( yaw + M_PI / 2.0 );
  center.position.y += diff * sin( yaw + M_PI / 2.0 );

  // }

  return true;
}

double MPC_Controller::calc_local_path( geometry_msgs::Pose &lookahead_pose, double distance )
{

  int psize = current_path_.poses.size();
  int path_po_lenght;
  std::vector<Eigen::Vector2d> points;
  // start point from the closest point
  calcClosestPoint();
  points.emplace_back( closest_point.point.x, closest_point.point.y );

  // search for closest point on path
  double min_dif = std::numeric_limits<double>::max();
  int st_point = 0;
  for ( int i = 0; i < psize; i++ ) {
    double po_dist =
        std::sqrt( std::pow( current_path_.poses[i].pose.position.x - points.front().x(), 2 ) +
                   std::pow( current_path_.poses[i].pose.position.y - points.front().y(), 2 ) );
    if ( po_dist < min_dif ) {
      min_dif = po_dist;
      st_point = i;
    }
  }

  path_po_lenght = 0;
  // calculate path_po_lenght -> number of waypoints in the carrot distance
  int collision_points = 0;
  for ( int i = st_point + 1; i < psize; i++ ) {
    // double curr_dist =
    //     std::sqrt( std::pow( closest_point.point.x - current_path_.poses[i].pose.position.x, 2 ) +
    //                std::pow( closest_point.point.y - current_path_.poses[i].pose.position.y, 2 ) );
       double curr_dist =
        std::sqrt( std::pow( closest_point.point.x - current_path_.poses[i].pose.position.x, 2 ) +
                   std::pow( closest_point.point.y - current_path_.poses[i].pose.position.y, 2 ) );
    if ( curr_dist > distance ) // search for points
    {
      break;
    } else {
      path_po_lenght = path_po_lenght + 1;

    }
  }
  lookahead_pose = current_path_.poses[path_po_lenght + st_point].pose;

  return ( st_point + path_po_lenght );

}

// Calculate the closest Point on the linear interpolated path, returns index of next point on path
int MPC_Controller::calcClosestPoint()
{
  // Calculate the two closest Points on the path
  size_t closest = 1;
  size_t second_closest = 0;
  double shortest_dist = 999999;
  for ( size_t i = 0; i < current_path_.poses.size(); i++ ) {
    double dist = std::sqrt(
        std::pow( robot_control_state.pose.position.x - current_path_.poses[i].pose.position.x, 2 ) +
        std::pow( robot_control_state.pose.position.y - current_path_.poses[i].pose.position.y, 2 ) );
    if ( dist < shortest_dist ) {
      shortest_dist = dist;
      second_closest = closest;
      closest = i;
    }
  }

  if ( closest == 0 ) {
    second_closest = 1;
  } else {
    if ( ( closest + 1 ) < current_path_.poses.size() ) {
      double prev_dx = ( current_path_.poses[closest - 1].pose.position.x -
                         current_path_.poses[closest].pose.position.x );
      double prev_dy = ( current_path_.poses[closest - 1].pose.position.y -
                         current_path_.poses[closest].pose.position.y );
      double next_dx = ( current_path_.poses[closest + 1].pose.position.x -
                         current_path_.poses[closest].pose.position.x );
      double next_dy = ( current_path_.poses[closest + 1].pose.position.y -
                         current_path_.poses[closest].pose.position.y );
      double robot_dx =
          ( robot_control_state.pose.position.x - current_path_.poses[closest].pose.position.x );
      double robot_dy =
          ( robot_control_state.pose.position.y - current_path_.poses[closest].pose.position.y );

      double angle_prev = std::acos( ( prev_dx * robot_dx + prev_dy * robot_dy ) /
                                     ( sqrt( prev_dx * prev_dx + prev_dy * prev_dy ) +
                                       sqrt( robot_dx * robot_dx + robot_dy * robot_dy ) ) );
      double angle_next = std::acos( ( next_dx * robot_dx + next_dy * robot_dy ) /
                                     ( sqrt( next_dx * next_dx + next_dy * next_dy ) +
                                       sqrt( robot_dx * robot_dx + robot_dy * robot_dy ) ) );

      if ( fabs( angle_prev ) < fabs( angle_next ) ) {
        second_closest = closest - 1;
      } else {
        second_closest = closest + 1;
      }
    } else {
      second_closest = closest - 1;
    }
  }
  // Calculate the closest Point on the connection line of the two closest points on the path
  double l1 = current_path_.poses[second_closest].pose.position.x -
              current_path_.poses[closest].pose.position.x;
  double l2 = current_path_.poses[second_closest].pose.position.y -
              current_path_.poses[closest].pose.position.y;

  double r1 = -l2;
  double r2 = l1;

  double lambda =
      ( l2 * ( current_path_.poses[closest].pose.position.x - robot_control_state.pose.position.x ) +
        l1 * ( robot_control_state.pose.position.y - current_path_.poses[closest].pose.position.y ) ) /
      ( r1 * l2 - r2 * l1 );

  closest_point.point.x = robot_control_state.pose.position.x + lambda * r1;
  closest_point.point.y = robot_control_state.pose.position.y + lambda * r2;
  closest_point.header = robot_state_header;

  // ROS_INFO("closest: %i, second: %i", closest, second_closest);
  // ROS_INFO("closest: x: %f, y: %f", current_path.poses[closest].pose.position.x, current_path.poses[closest].pose.position.y);
  // ROS_INFO("second: x: %f, y: %f", current_path.poses[second_closest].pose.position.x, current_path.poses[second_closest].pose.position.y);

  if ( closest > second_closest ) {
    return closest;
  } else {
    return second_closest;
  }
}
