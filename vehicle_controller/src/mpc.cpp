#include <vehicle_controller/mpc.h>

#include "iostream"
MPC_Controller::MPC_Controller( ros::NodeHandle &nh_ )
    : Controller( nh_ ), nh_dr_paramsss( "~/mpc_params" )
{
  // LQR parameters
  map_sub = nh_.subscribe( "/map", 1, &MPC_Controller::map_messageCallback2, this );
  dr_controller_params_server =
      new dynamic_reconfigure::Server<vehicle_controller::MPCParamsConfig>( nh_dr_paramsss );
  dr_controller_params_server->setCallback(
      boost::bind( &MPC_Controller::controllerParamsCallback, this, _1, _2 ) );
  // stateSubscriber = nh.subscribe( "/odom", 1, &MPC_Controller::stateCallback, this,
  // ros::TransportHints().tcpNoDelay( true ) );
  //   lqr_params_narrow = nh_dr_params.subscribe("/lqr_params_narrow",1, &Lqr_Controller::lqr_params_callback,this);
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

//   dr_controller_params_server = new
//   dynamic_reconfigure::Server<vehicle_controller::MPCParamsConfig>(nh_dr_paramsss);
//   dr_controller_params_server->setCallback(boost::bind(&MPC_Controller::controllerParamsCallback,
//   this, _1, _2)); return true;
// }
void MPC_Controller::controllerParamsCallback( vehicle_controller::MPCParamsConfig &config,
                                               uint32_t level )
{
  std::cout << "get params \n\n\n\n\n";
  lookahead = config.lookahead_distance;
  p = config.P;
  d = config.D;
  p2 = config.P2;
  d2 = config.D2;
  // lqr_r = config.R;
}

void MPC_Controller::computeMoveCmd()
{
  // std::cout << "x:  " << robot_control_state.pose.position.x << "\n";
  geometry_msgs::Twist cmd;
  double linear_vel = 0.0;
  double angular_vel = 0.0;

  // cmd.linear.x = 0.1;
  cmd.angular.z = 0.0;
  compute_cmd( linear_vel, angular_vel );
  /*  -----------------------------------------------------------------*/
  cmd.angular.z = angular_vel;
  cmd.linear.x = linear_vel;
  // send cmd to control interface
  vehicle_control_interface_->executeTwist( cmd, robot_control_state, yaw, pitch, roll );
}

void MPC_Controller::stateCallback( const nav_msgs::Odometry odom_state )
{
  //   updateRobotState( odom_state );
  //   geometry_msgs::Pose predict_pose;
  //   predicteRobotState(predict_pose, 0.0, 0.0);
}

void MPC_Controller::map_messageCallback2( const nav_msgs::OccupancyGrid &msg )
{
  grid_map::GridMapRosConverter::fromOccupancyGrid( msg, std::string( "occupancy" ),
                                                    occupancy_map ); // distance_transform occupancy
  get_map = true;
  // std::cout<<"get map"<<"\n\n\n";
  // occupancy_map =
  // for (grid_map::GridMapIterator iterator(occupancy_map);!iterator.isPastEnd(); ++iterator ){
  //         const grid_map::Index index(*iterator);
  //         const float value = occupancy_map.get("distance_transform")(index(0), index(1));
  //         std::cout<<value<<" ";
  //     }
}

void MPC_Controller::predict_distance( const geometry_msgs::Pose robot_pose )
{
  grid_map::Position robot_position2( robot_pose.position.x, robot_pose.position.y );
  grid_map::Length length2( 2, 2 );
  bool isSuccess;
  grid_map::GridMap submap = occupancy_map.getSubmap( robot_position2, length2, isSuccess );
  std::cout << submap.getSize() << "\n\n\n";
  obsticke_distance( robot_front, submap );
  obsticke_distance( robot_back, submap );
  obsticke_distance( robot_middle, submap );
}

void MPC_Controller::predict_position( const geometry_msgs::Pose robot_pose, double linear_vel,
                                       double angluar_vel, geometry_msgs::Pose &predict_pose )
{
  double roll_, pitch_, yaw_;
  tf::Quaternion q( robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z,
                    robot_pose.orientation.w );
  tf::Matrix3x3 m( q );
  m.getRPY( roll_, pitch_, yaw_ );
  double dt_ = 0.5;
  double theta = angluar_vel * dt_ + yaw_;
  double linear_vel_ = linear_vel; // robot_control_state.velocity_linear.x;
  // std::cout<<"liner_vel:  "<<linear_vel_<<"\n\n\n";
  double x = robot_pose.position.x + cos( theta ) * dt_ * linear_vel_;
  double y = robot_pose.position.y + sin( theta ) * dt_ * linear_vel_;

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

void MPC_Controller::create_robot_range( const geometry_msgs::Pose robot_pose )
{
  double roll_, pitch_, yaw_;
  tf::Quaternion q( robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z,
                    robot_pose.orientation.w );
  tf::Matrix3x3 m( q );
  m.getRPY( roll_, pitch_, yaw_ );
  double x = robot_pose.position.x; //+ 0.2*std::cos(yaw);
  double y = robot_pose.position.y; //+ 0.2*std::sin(yaw);
  double diagonal_length = 0.50 * std::sqrt( std::pow( length, 2 ) + std::pow( width, 2 ) );
  grid_map::Position p_front_right(
      x + std::cos( yaw_ ) * 0.5 * length + std::cos( yaw_ - M_PI / 2.0 ) * 0.5 * width,
      y + std::sin( yaw_ ) * 0.5 * length + std::sin( yaw_ - M_PI / 2.0 ) * 0.5 * width );
  grid_map::Position p_front_left(
      x + std::cos( yaw_ ) * 0.5 * length + std::cos( yaw_ + M_PI / 2.0 ) * 0.5 * width,
      y + std::sin( yaw_ ) * 0.5 * length + std::sin( yaw_ + M_PI / 2.0 ) * 0.5 * width );
  grid_map::Position p_back_right( x + std::cos( yaw_ + M_PI ) * 0.5 * length +
                                       std::cos( yaw_ + M_PI + M_PI / 2.0 ) * 0.5 * width,
                                   y + std::sin( yaw_ + M_PI ) * 0.5 * length +
                                       std::sin( yaw_ + M_PI + M_PI / 2.0 ) * 0.5 * width );
  grid_map::Position p_back_left( x + std::cos( yaw_ + M_PI ) * 0.5 * length +
                                      std::cos( yaw_ + M_PI - M_PI / 2.0 ) * 0.5 * width,
                                  y + std::sin( yaw_ + M_PI ) * 0.5 * length +
                                      std::sin( yaw_ + M_PI - M_PI / 2.0 ) * 0.5 * width );
  grid_map::Position p_mid_right( 0.5 * ( p_front_right[0] + p_back_right[0] ),
                                  0.5 * ( p_front_right[1] + p_back_right[1] ) );
  grid_map::Position p_mid_left( 0.5 * ( p_front_left[0] + p_back_left[0] ),
                                 0.5 * ( p_front_left[1] + p_back_left[1] ) );

  /*create points for each side*/
  /*based on two point to genarate a middle point, do this step as a loop*/
  robot_front.clear();
  robot_back.clear();

  robot_front.push_back( robot_range( p_front_right, 0.0 ) );
  robot_front.push_back( robot_range( p_front_left, 0.0 ) );
  robot_back.push_back( robot_range( p_back_right, 0.0 ) );
  robot_back.push_back( robot_range( p_back_left, 0.0 ) );
  robot_middle.push_back( robot_range( p_mid_right, 0.0 ) );
  robot_middle.push_back( robot_range( p_mid_left, 0.0 ) );
}

bool MPC_Controller::compute_cmd( double &linear_vel, double &angluar_vel )
{
  geometry_msgs::Pose lookaheadPose;
  calc_local_path( lookaheadPose );

  double current_angle_diff;
  double roll_, pitch_, yaw_;
  tf::Quaternion q( robot_control_state.pose.orientation.x, robot_control_state.pose.orientation.y,
                    robot_control_state.pose.orientation.z, robot_control_state.pose.orientation.w );
  tf::Matrix3x3 m( q );
  m.getRPY( roll_, pitch_, yaw_ );

  double roll2, pitch2, yaw2;
  tf::Quaternion q2( lookaheadPose.orientation.x, lookaheadPose.orientation.y,
                     lookaheadPose.orientation.z, lookaheadPose.orientation.w );
  tf::Matrix3x3 m2( q2 );
  m2.getRPY( roll2, pitch2, yaw2 );
  current_angle_diff = constrainAngle_mpi_pi( yaw2 - yaw_ );
  current_angle_diff /= 3.0;
  // std::cout<<"current_angle_diff:  "<<current_angle_diff<<"\n\n\n\n";
  if ( current_angle_diff > 0 ) {
    for ( int i = 0; i < sizeof( angluar_array_positive ) / sizeof( angluar_array_positive[0] );
          i++ ) {
      double ang_vel = angluar_array_positive[i];
      if ( std::abs( current_angle_diff ) - std::abs( ang_vel ) > 0.1 ) {
        continue;
      }
      for ( int j = 0; j < sizeof( linear_array ) / sizeof( linear_array[0] ); j++ ) {
        double lin_vel = linear_array[j];
        geometry_msgs::Pose predict_pos;
        predict_position( robot_control_state.pose, lin_vel, ang_vel, predict_pos );
        create_robot_range( predict_pos );
        bool collision = collision_detection( predict_pos );
        if ( collision == false ) {
          linear_vel = lin_vel;
          angluar_vel = ang_vel;
          return true;
        }
      }
    }
  }

  else if ( current_angle_diff < 0 ) {
    for ( int i = 0; i < sizeof( angluar_array_negative ) / sizeof( angluar_array_negative[0] );
          i++ ) {
      double ang_vel = angluar_array_negative[i];
      if ( std::abs( current_angle_diff ) - std::abs( ang_vel ) > 0.1 ) {
        continue;
      }
      for ( int j = 0; j < sizeof( linear_array ) / sizeof( linear_array[0] ); j++ ) {
        double lin_vel = linear_array[j];
        geometry_msgs::Pose predict_pos;
        predict_position( robot_control_state.pose, lin_vel, ang_vel, predict_pos );
        create_robot_range( predict_pos );
        bool collision = collision_detection( predict_pos );
        if ( collision == false ) {
          linear_vel = lin_vel;
          angluar_vel = ang_vel;
          return true;
        }
      }
    }
  }
}

// void MPC_Controller::obsticke_distance( std::vector<robot_range> &robot, grid_map::GridMap map )
// {

//   grid_map::Position robot_pos( robot[0].position );
//   robot[0].distance = ray_detection( -( M_PI / 2.0 ), robot_pos, map );

//   grid_map::Position robot_pos2( robot[1].position );
//   robot[1].distance = ray_detection( M_PI / 2.0, robot_pos2, map );
// }

void MPC_Controller::obsticke_distance( std::vector<robot_range> &robot, grid_map::GridMap map )
{
  for ( int i = 0; i < robot.size(); i++ ) {
    grid_map::Position robot_pos( robot[i].position );
    double min_distance = MAXFLOAT;
    for ( grid_map::GridMapIterator iterator( map ); !iterator.isPastEnd(); ++iterator ) {
      const grid_map::Index index( *iterator );
      const float value = map.get( "occupancy" )( index( 0 ), index( 1 ) );
      if ( value != NAN && value != 0 ) {
        grid_map::Position obsticale_pos;
        map.getPosition( index, obsticale_pos );
        double distance = compute_distance( robot_pos, obsticale_pos );
        if ( distance < min_distance ) {
          min_distance = distance;
        }
      }
    }
    robot[i].distance = min_distance;
    // std::cout<<"size: "<<robot.size()<<"\n";
  }
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

void MPC_Controller::get_min_distance( robot_ladar &rl )
{

  std::sort( robot_right.begin(), robot_right.end(), MPC_Controller::compareByDistance );
  std::sort( robot_left.begin(), robot_left.end(), MPC_Controller::compareByDistance );
  // std::sort(robot_front.begin(),robot_right.end(),NarrowPassageController::compareByDistance);
  // std::sort(robot_back.begin(),robot_right.end(),NarrowPassageController::compareByDistance);

  rl.right_distance = robot_right[0].distance;
  rl.left_distance = robot_left[0].distance;
  // front = robot_front[0].distance;
  // back = robot_back[0].distance;
  rl.front_distance = 0;
  rl.back_distance = 0;
}

double MPC_Controller::crossProduct( const Vector_ &AB, const Point_ &C )
{
  Point_ A = AB.start;
  Point_ B = AB.end;
  return ( B.x - A.x ) * ( C.y - A.y ) - ( B.y - A.y ) * ( C.x - A.x );
  // >0 left, <0 right
}

double MPC_Controller::pd_controller( double &last_e_front, double &last_e_back, const double p,
                                      const double d )
{
  double e_front = robot_front[1].distance - robot_front[0].distance;
  std::cout << "p:  " << p << "   d:  " << d << "\n\n\n";

  std::cout << "left:  " << robot_front[1].distance << " right:  " << robot_front[0].distance
            << "\n\n\n";

  double ed_front = e_front - last_e_front;
  last_e_front = ed_front;
  double angle_front = std::tan( e_front / ( length ) );
  std::cout << "e_front : " << e_front << "\n\n\n\n";
  double angle_front_d = std::tan( ed_front / ( length ) );

  double out = p * e_front + d * ed_front;
  out /= 2.0;

  double e_back = robot_back[0].distance - robot_back[1].distance;
  double ed_back = e_back - last_e_back;
  last_e_back = ed_back;
  double angle_back = std::tan( e_back / ( length ) );
  double angle_back_d = std::tan( ed_back / ( length ) );
  std::cout << "e_back : " << e_back << "\n\n\n\n";

  double out2 = ( p * e_back + d * ed_back ) / 2.0;
  // out /=2.0;
  // out += 0.1*out2;
  // out *=0.05;
  out = ( std::abs( out ) > 1.0 ) ? ( ( out > 0 ) ? 1.0 : -1.0 ) : out;

  return out;
}

double MPC_Controller::pd_controller2( geometry_msgs::Pose clost_pose, double &last_e,
                                       const double p, const double d )
{
  double roll, pitch, yaw;
  tf::Quaternion q( robot_control_state.pose.orientation.x, robot_control_state.pose.orientation.y,
                    robot_control_state.pose.orientation.z, robot_control_state.pose.orientation.w );
  tf::Matrix3x3 m( q );
  m.getRPY( roll, pitch, yaw );

  double robot_angles[3];
  quaternion2angles( robot_control_state.pose.orientation, robot_angles );

  double clost_angles[3];
  quaternion2angles( clost_pose.orientation, clost_angles );

  double roll2, pitch2, yaw2;
  tf::Quaternion q2( clost_pose.orientation.x, clost_pose.orientation.y, clost_pose.orientation.z,
                     clost_pose.orientation.w );
  tf::Matrix3x3 m2( q2 );
  m2.getRPY( roll2, pitch2, yaw2 );

  double angle_to_waypoint =
      std::atan2( clost_pose.position.y - robot_control_state.pose.position.y,
                  clost_pose.position.x - robot_control_state.pose.position.x );

  double e = constrainAngle_mpi_pi( -yaw + yaw2 );
  std::cout << "angle _e :" << yaw << "     " << yaw2 << "\n\n\n";
  double ed = e - last_e;
  double out = p * e + d * ed;
  out /= 2.0;
  // out *=0.05;
  out = ( std::abs( out ) > 1.0 ) ? ( ( out > 0 ) ? 1.0 : -1.0 ) : out;
  last_e = e;
  // error = e;
  return out;
}

bool MPC_Controller::collision_detection( const geometry_msgs::Pose robot_pose )
{

  double x = robot_pose.position.x;
  double y = robot_pose.position.y;

  grid_map::Position robot_position2( x, y );
  grid_map::Length length2( 2, 2 );
  bool isSuccess;
  grid_map::GridMap submap = occupancy_map.getSubmap( robot_position2, length2, isSuccess );
  grid_map::Position p_front_right( robot_front[0].position[0], robot_front[0].position[1] );
  grid_map::Position p_front_left( robot_front[1].position[0], robot_front[1].position[1] );
  grid_map::Position p_back_right( robot_back[0].position[0], robot_back[0].position[1] );
  grid_map::Position p_back_left( robot_back[1].position[0], robot_back[1].position[1] );

  grid_map::Position p_mid_right( 0.5 * ( p_front_right[0] + p_back_right[0] ),
                                  0.5 * ( p_front_right[1] + p_back_right[1] ) );
  grid_map::Position p_mid_left( 0.5 * ( p_front_left[0] + p_back_left[0] ),
                                 0.5 * ( p_front_left[1] + p_back_left[1] ) );

  bool front_left_collision = false;
  bool front_right_collision = false;
  bool back_left_collision = false;
  bool back_right_collision = false;

  geometry_msgs::Twist cmd;
  // double linear_vel = 0.0;
  cmd.linear.x = 0.0;
  cmd.angular.z = 0.0;

  grid_map::Index front_right;
  occupancy_map.getIndex( p_front_right, front_right );
  grid_map::Index back_right;
  occupancy_map.getIndex( p_back_left, back_right );
  grid_map::Index front_left;
  occupancy_map.getIndex( p_front_left, front_left );
  grid_map::Index back_left;
  occupancy_map.getIndex( p_back_left, back_left );
  grid_map::Index mid_left;
  occupancy_map.getIndex( p_mid_left, mid_left );
  grid_map::Index mid_right;
  occupancy_map.getIndex( p_mid_right, mid_right );
  int count=0;
  for ( grid_map::LineIterator iterator( occupancy_map, front_right, back_right );
        !iterator.isPastEnd(); ++iterator ) {
    // std::cout<<"value : "<<occupancy_map.at( "occupancy", *iterator );
    double value = occupancy_map.at( "occupancy", *iterator );
    if ( value > 0 && value != NAN ) {
      // front_left_collision = true;
      // vehicle_control_interface_->executeTwist( cmd, robot_control_state, yaw, pitch, roll );
      // stop();
      if(count>0){
              std::cout << "collison !!!!!!!!!!!!!!1111\n\n\n\n";
        return true;
      // break;
      }
    }
  }
  count = 0;
  for ( grid_map::LineIterator iterator( occupancy_map, front_left, back_left );
        !iterator.isPastEnd(); ++iterator ) {
    // std::cout<<"value : "<<occupancy_map.at( "occupancy", *iterator )<<"  ";
    double value = occupancy_map.at( "occupancy", *iterator );
    if ( value > 0 && value != NAN ) {
      // front_left_collision = true;
      // vehicle_control_interface_->executeTwist( cmd, robot_control_state, yaw, pitch, roll );
      // stop();
      count++;
      // front_left_collision = true;
      // vehicle_control_interface_->executeTwist( cmd, robot_control_state, yaw, pitch, roll );
      // stop();
      if(count>0){
              std::cout << "collison !!!!!!!!!!!!!!1111\n\n\n\n";
        return true;
      }

      // break;
    }
  }
  count = 0;
  for ( grid_map::LineIterator iterator( occupancy_map, front_left, front_right );
        !iterator.isPastEnd(); ++iterator ) {
    // std::cout<<"value : "<<occupancy_map.at( "occupancy", *iterator );
    double value = occupancy_map.at( "occupancy", *iterator );
    if ( value > 0 && value != NAN ) {
      count++;
      // front_left_collision = true;
      // vehicle_control_interface_->executeTwist( cmd, robot_control_state, yaw, pitch, roll );
      // stop();
      if(count>0){
              std::cout << "collison !!!!!!!!!!!!!!1111\n\n\n\n";
        return true;
      }

      // break;
    }
  }
  count = 0;
  for ( grid_map::LineIterator iterator( occupancy_map, back_left, back_right );
        !iterator.isPastEnd(); ++iterator ) {
    double value = occupancy_map.at( "occupancy", *iterator );
    if ( value > 0 && value != NAN ) {
      count ++;
      // front_left_collision = true;
      // vehicle_control_interface_->executeTwist( cmd, robot_control_state, yaw, pitch, roll );
      // stop();
      if(count>0){
              std::cout << "collison !!!!!!!!!!!!!!1111\n\n\n\n";
        return true;
      }

      // break;
    }
  }

  return false;
}

double MPC_Controller::calc_local_path( geometry_msgs::Pose &lookahead_pose )
{

  int psize = current_path.poses.size();
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
        std::sqrt( std::pow( current_path.poses[i].pose.position.x - points.front().x(), 2 ) +
                   std::pow( current_path.poses[i].pose.position.y - points.front().y(), 2 ) );
    if ( po_dist < min_dif ) {
      min_dif = po_dist;
      st_point = i;
    }
  }

  path_po_lenght = 0;
  // calculate path_po_lenght -> number of waypoints in the carrot distance
  for ( int i = st_point + 1; i < psize; i++ ) {
    double curr_dist =
        std::sqrt( std::pow( closest_point.point.x - current_path.poses[i].pose.position.x, 2 ) +
                   std::pow( closest_point.point.y - current_path.poses[i].pose.position.y, 2 ) );

    if ( curr_dist > lookahead ) // search for points
    {
      break;
    } else {
      path_po_lenght = path_po_lenght + 1;
      //      co_points = co_points + 1;
      //      points[co_points][0] = current_path.poses[i].pose.position.x;
      //      points[co_points][1] = current_path.poses[i].pose.position.y;
    }
  }
  lookahead_pose = current_path.poses[path_po_lenght].pose;
  // robot_index = st_point;
  return ( st_point + path_po_lenght );

  double angle_carrot = std::atan2(
      current_path.poses[st_point + path_po_lenght].pose.position.y - closest_point.point.y,
      current_path.poses[st_point + path_po_lenght].pose.position.x - closest_point.point.x );

  for ( int i = 0; i <= path_po_lenght; i++ ) {
    double angle_waypoint =
        std::atan2( current_path.poses[st_point + i].pose.position.y - closest_point.point.y,
                    current_path.poses[st_point + i].pose.position.x - closest_point.point.x );
    double angle_diff_carrot2waypoint = constrainAngle_mpi_pi( angle_carrot - angle_waypoint );
    // ROS_INFO("angle diff carrot2waypoint: %f", angle_diff_carrot2waypoint);
    if ( fabs( angle_diff_carrot2waypoint ) < M_PI_2 ) {
      points.emplace_back( current_path.poses[st_point + i].pose.position.x,
                           current_path.poses[st_point + i].pose.position.y );
    }
  }
}

// Calculate the closest Point on the linear interpolated path, returns index of next point on path
int MPC_Controller::calcClosestPoint()
{
  // Calculate the two closest Points on the path
  size_t closest = 1;
  size_t second_closest = 0;
  double shortest_dist = 999999;
  for ( size_t i = 0; i < current_path.poses.size(); i++ ) {
    double dist = std::sqrt(
        std::pow( robot_control_state.pose.position.x - current_path.poses[i].pose.position.x, 2 ) +
        std::pow( robot_control_state.pose.position.y - current_path.poses[i].pose.position.y, 2 ) );
    if ( dist < shortest_dist ) {
      shortest_dist = dist;
      second_closest = closest;
      closest = i;
    }
  }

  if ( closest == 0 ) {
    second_closest = 1;
  } else {
    if ( ( closest + 1 ) < current_path.poses.size() ) {
      double prev_dx = ( current_path.poses[closest - 1].pose.position.x -
                         current_path.poses[closest].pose.position.x );
      double prev_dy = ( current_path.poses[closest - 1].pose.position.y -
                         current_path.poses[closest].pose.position.y );
      double next_dx = ( current_path.poses[closest + 1].pose.position.x -
                         current_path.poses[closest].pose.position.x );
      double next_dy = ( current_path.poses[closest + 1].pose.position.y -
                         current_path.poses[closest].pose.position.y );
      double robot_dx =
          ( robot_control_state.pose.position.x - current_path.poses[closest].pose.position.x );
      double robot_dy =
          ( robot_control_state.pose.position.y - current_path.poses[closest].pose.position.y );

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
  double l1 = current_path.poses[second_closest].pose.position.x -
              current_path.poses[closest].pose.position.x;
  double l2 = current_path.poses[second_closest].pose.position.y -
              current_path.poses[closest].pose.position.y;

  double r1 = -l2;
  double r2 = l1;

  double lambda =
      ( l2 * ( current_path.poses[closest].pose.position.x - robot_control_state.pose.position.x ) +
        l1 * ( robot_control_state.pose.position.y - current_path.poses[closest].pose.position.y ) ) /
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
