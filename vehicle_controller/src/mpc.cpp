#include <vehicle_controller/mpc.h>

#include "iostream"
MPC_Controller::MPC_Controller( ros::NodeHandle &nh_ )
    : Controller( nh_ ), nh_dr_params( "~/lqr_controller_params" )
{
  // LQR parameters
  map_sub = nh_.subscribe( "/map", 1, &MPC_Controller::map_messageCallback2, this );

  // stateSubscriber = nh.subscribe( "/odom", 1, &MPC_Controller::stateCallback, this,
  // ros::TransportHints().tcpNoDelay( true ) );
  //   lqr_params_narrow = nh_dr_params.subscribe("/lqr_params_narrow",1, &Lqr_Controller::lqr_params_callback,this);
}

MPC_Controller::~MPC_Controller() { }

void MPC_Controller::reset()
{
  Controller::reset();
  //   lqr_y_error = 0;
  //   lqr_angle_error = 0;
  //   state = INACTIVE;
  //   current = 0;
}

void MPC_Controller::computeMoveCmd()
{
  // std::cout << "x:  " << robot_control_state.pose.position.x << "\n";
  geometry_msgs::Twist cmd;
  double linear_vel = 0.2;
  // cmd.linear.x = 0.1;
  cmd.angular.z = 0.0;
  create_robot_range( robot_control_state.pose);
  predict_distance( robot_control_state.pose );
  double error_front = robot_front[1].distance - robot_front[0].distance;
  double error_back = robot_back[1].distance - robot_back[0].distance;
  double error_front_ = error_front * std::abs( error_front );

  double out_back = pd_controller( error_front_, last_error_front, 50, 1 );
  out_back *=-0.5;
  std::cout << "error_front:  " << error_front << "\n";
  double out_front = pd_controller( error_front_, last_error_front, 50, 1 );
  double _error_front_ = ( std::abs( error_front ) > 0.2 ) ? 0.2 : std::abs(error_front);
  linear_vel = ( ( 0.21 - _error_front_ ) / 0.21 ) * 0.2;
  linear_vel = ( std::abs( linear_vel ) < 0.1 ) ? ( linear_vel > 0 ? 0.1 : -0.1 ) : linear_vel;
  out_front += out_back;


  
    try  
    {  
          collision_detection( linear_vel, out_front, dt );

    }  
    catch (std::exception& e)  
    {  
        // cout << "Standard exception: " << e.what() << endl;  
    }  
  // collision_detection( linear_vel, out_front, dt );
  std::cout << "anlge _ vel:  " << out_front << "\n";

  out_front = ( std::abs( out_front ) > 0.5 ) ? ( ( out_front > 0 ) ? 0.5 : -0.5 ) : out_front;
  // std::cout << "anlge _ vel:  " << out_front << "\n";

  // double out_front = 0;
  // double out_mid = pd_controller(error_mid, last_error_mid, 5,2.5);
  // out_mid = 0;

  cmd.angular.z = out_front;
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

  obsticke_distance( robot_front, submap );
  obsticke_distance( robot_back, submap );
  obsticke_distance( robot_middle, submap );
  std::cout << "right:  " << robot_front[0].distance << "    left: " << robot_front[1].distance
            << "\n";
  // // obsticke_distance(robot_front,submap);
  // // obsticke_distance(robot_back,submap);
}

void MPC_Controller::create_robot_range( const geometry_msgs::Pose robot_pose)
{
  double roll, pitch, yaw;
  tf::Quaternion q( robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z,
                    robot_pose.orientation.w );
  tf::Matrix3x3 m( q );
  m.getRPY( roll, pitch, yaw );
  double x = robot_pose.position.x; //+ 0.2*std::cos(yaw);
  double y = robot_pose.position.y; //+ 0.2*std::sin(yaw);
  double diagonal_length = 0.50 * std::sqrt( std::pow( length, 2 ) + std::pow( width, 2 ) );
  grid_map::Position p_front_right(
      x + std::cos( yaw ) * 0.5 * length + std::cos( yaw - M_PI / 2.0 ) * 0.5 * width,
      y + std::sin( yaw ) * 0.5 * length + std::sin( yaw - M_PI / 2.0 ) * 0.5 * width );
  grid_map::Position p_front_left(
      x + std::cos( yaw ) * 0.5 * length + std::cos( yaw + M_PI / 2.0 ) * 0.5 * width,
      y + std::sin( yaw ) * 0.5 * length + std::sin( yaw + M_PI / 2.0 ) * 0.5 * width );
  grid_map::Position p_back_right(
      x + std::cos( yaw + M_PI ) * 0.5 * length + std::cos( yaw + M_PI + M_PI / 2.0 ) * 0.5 * width,
      y + std::sin( yaw + M_PI ) * 0.5 * length + std::sin( yaw + M_PI + M_PI / 2.0 ) * 0.5 * width );
  grid_map::Position p_back_left(
      x + std::cos( yaw + M_PI ) * 0.5 * length + std::cos( yaw + M_PI - M_PI / 2.0 ) * 0.5 * width,
      y + std::sin( yaw + M_PI ) * 0.5 * length + std::sin( yaw + M_PI - M_PI / 2.0 ) * 0.5 * width );
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

double MPC_Controller::pd_controller( const double e, double &last_e, const double p, const double d )
{
  double ed = e - last_e;

  double out = p * e + d * ed;
  // out *=0.05;
  out = ( std::abs( out ) > 0.2 ) ? ( ( out > 0 ) ? 0.2 : -0.2 ) : out;
  last_e = e;

  return out;
}

void MPC_Controller::collision_detection( double &linear_vel, double &angluar_vel, double dt )
{

  double roll, pitch, yaw_;
  tf::Quaternion q( robot_control_state.pose.orientation.x, robot_control_state.pose.orientation.y,
                    robot_control_state.pose.orientation.z, robot_control_state.pose.orientation.w );
  tf::Matrix3x3 m( q );
  m.getRPY( roll, pitch, yaw_ );
  double theta = angluar_vel * dt+ yaw_;
  double linear_vel_ = linear_vel;//robot_control_state.velocity_linear.x;
  // std::cout<<"liner_vel:  "<<linear_vel_<<"\n\n\n";
  double x = robot_control_state.pose.position.x + cos( theta ) * dt * linear_vel_ ;
  double y = robot_control_state.pose.position.y + sin( theta ) * dt * linear_vel_ ;


  grid_map::Position robot_position2( x, y );
  grid_map::Length length2( 2, 2 );
  bool isSuccess;
  grid_map::GridMap submap = occupancy_map.getSubmap( robot_position2, length2, isSuccess );
  yaw_ = theta;
  grid_map::Position p_front_right(
      x + std::cos( yaw_ ) * 0.5 * length + std::cos( yaw_ - M_PI / 2.0 ) * 0.5 * width,
      y + std::sin( yaw_ ) * 0.5 * length + std::sin( yaw_ - M_PI / 2.0 ) * 0.5 * width );
  grid_map::Position p_front_left(
      x + std::cos( yaw_ ) * 0.5 * length + std::cos( yaw_ + M_PI / 2.0 ) * 0.5 * width,
      y + std::sin( yaw_ ) * 0.5 * length + std::sin( yaw_ + M_PI / 2.0 ) * 0.5 * width );
  grid_map::Position p_back_right(
      x + std::cos( yaw_ + M_PI ) * 0.5 * length + std::cos( yaw_ + M_PI + M_PI / 2.0 ) * 0.5 * width,
      y + std::sin( yaw_ + M_PI ) * 0.5 * length + std::sin( yaw_ + M_PI + M_PI / 2.0 ) * 0.5 * width );
  grid_map::Position p_back_left(
      x + std::cos( yaw_ + M_PI ) * 0.5 * length + std::cos( yaw_ + M_PI - M_PI / 2.0 ) * 0.5 * width,
      y + std::sin( yaw_ + M_PI ) * 0.5 * length + std::sin( yaw_ + M_PI - M_PI / 2.0 ) * 0.5 * width );

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
  cmd.linear.x = 0.1;
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

  for ( grid_map::LineIterator iterator( occupancy_map, front_right, mid_right );
        !iterator.isPastEnd(); ++iterator ) {
    // std::cout<<"value : "<<occupancy_map.at( "occupancy", *iterator );
    if ( occupancy_map.at( "occupancy", *iterator ) > 0 ) {
      front_right_collision = true;
        vehicle_control_interface_->executeTwist( cmd, robot_control_state, yaw, pitch, roll );

      break;
    }
  }

  for ( grid_map::LineIterator iterator( occupancy_map, front_left, mid_left );
        !iterator.isPastEnd(); ++iterator ) {
      // std::cout<<"value : "<<occupancy_map.at( "occupancy", *iterator )<<"  ";

    if ( occupancy_map.at( "occupancy", *iterator ) > 0 ) {
      front_left_collision = true;
        vehicle_control_interface_->executeTwist( cmd, robot_control_state, yaw, pitch, roll );

      break;
    }
  }

  for ( grid_map::LineIterator iterator( occupancy_map, mid_right, back_right );
        !iterator.isPastEnd(); ++iterator ) {
    // std::cout<<"value : "<<occupancy_map.at( "occupancy", *iterator );
    if ( occupancy_map.at( "occupancy", *iterator ) > 0 ) {
      back_right_collision = true;
        vehicle_control_interface_->executeTwist( cmd, robot_control_state, yaw, pitch, roll );

      break;
    }
  }

  for ( grid_map::LineIterator iterator( occupancy_map, mid_left, back_left );
        !iterator.isPastEnd(); ++iterator ) {
    if ( occupancy_map.at( "occupancy", *iterator ) > 0 ) {
      back_left_collision = true;
        vehicle_control_interface_->executeTwist( cmd, robot_control_state, yaw, pitch, roll );

      break;
    }
  }

  if ( front_left_collision) {
    angluar_vel -= 0.005;
    linear_vel = 0.00;
    collision_detection( linear_vel, angluar_vel, dt );
    std::cout << "front_left side collision collision !!!!!!"
              << "\n\n\n";
  } 
  else if (back_right_collision)
  {
    angluar_vel -= 0.005;
    linear_vel = 0.00;
    collision_detection( linear_vel, angluar_vel, dt );
    std::cout << "back_right side collision collision !!!!!!"
              << "\n\n\n";
  }
  
  else if ( front_right_collision) {
    angluar_vel += 0.005;
    linear_vel = 0.00;
    collision_detection( linear_vel, angluar_vel, dt );
    std::cout << "front_right side collision collision !!!!!!"
              << "\n\n\n";
  }
  else if (back_left_collision)
  {
    angluar_vel += 0.005;
    linear_vel = 0.00;
    collision_detection( linear_vel, angluar_vel, dt );
    std::cout << "back_left side collision collision !!!!!!"
              << "\n\n\n";
  }
  

  
}