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

  /*  -----------------------------------------------------------------*/
  create_robot_range( robot_control_state.pose );
  predict_distance( robot_control_state.pose );

  // std::cout << "anlge _ vel:  " << out_front << "\n";

  // double out_front = 0;
  // double out_mid = pd_controller(error_mid, last_error_mid, 5,2.5);
  // out_mid = 0;
  /*--------------------------------------------------------------------------------------------------------*/

  /*------------------------------------------------------------------------------------*/
  // double _error_front_ = ( std::abs( error_front ) > 0.2 ) ? 0.2 : std::abs( error_front );
  // linear_vel = ( ( 0.21 - _error_front_ ) / 0.21 ) * 0.2;
  // linear_vel = ( std::abs( linear_vel ) < 0.02 ) ? ( linear_vel > 0 ? 0.2 : -0.2 ) : linear_vel;
  double out_front = calc_local_path();
  collision_detection(linear_vel,out_front,dt);
  // double out_front_ = pd_controller(last_error_front, last_error_back,2.5,1);
  /*------------------------------------------------------------------------------------*/
  // std::cout << "out _ diff:  " << std::abs(out_front - out_front_) << "\n";
  // std::cout << "out _ :  " << out_front_ << "\n";
  // if(std::abs(out_front)<0.2){
  //   cmd.angular.z = out_front+out_front_;
  // }
  // else{
  //   cmd.angular.z = out_front;
  // }
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
  // std::cout << "right:  " << robot_front[0].distance << "    left: " << robot_front[1].distance
            // << "\n";
  // // obsticke_distance(robot_front,submap);
  // // obsticke_distance(robot_back,submap);
}

void MPC_Controller::create_robot_range( const geometry_msgs::Pose robot_pose )
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

  grid_map::Position robot_pos( robot[0].position );
  robot[0].distance = ray_detection( -( M_PI / 2.0 ), robot_pos, map );

  grid_map::Position robot_pos2( robot[1].position );
  robot[1].distance = ray_detection( M_PI / 2.0, robot_pos2, map );
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
    return 1.0;
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
  return ( dotProduct / ( lenghtOA * lengthOB ) > 0.99 );
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

double MPC_Controller::pd_controller(double &last_e_front, double &last_e_back, const double p, const double d )
{
  double e_front = robot_front[1].distance - robot_front[0].distance; 

  std::cout<<"left:  "<<robot_front[1].distance <<" right:  "<<robot_front[0].distance<<"\n\n\n";

  double ed_front = e_front - last_e_front;
  last_e_front = ed_front;
  double angle_front = std::tan(e_front/ (length));
  std::cout<<"e_front : "<<e_front<<"\n\n\n\n";
  double angle_front_d = std::tan(ed_front/ (length));

  double out = p * e_front + d * ed_front;
  out /=2.0; 

  double e_back = robot_back[0].distance - robot_back[1].distance; 
  double ed_back = e_back - last_e_back;
  last_e_back = ed_back;
  double angle_back = std::tan(e_back/ (length));
  double angle_back_d = std::tan(ed_back/ (length));
    std::cout<<"e_back : "<<e_back<<"\n\n\n\n";

  // out += (p * e_back + d * ed_back) /2.0 * 0.5;
  // out /=2.0; 

  // out *=0.05;
  out = ( std::abs( out ) > 0.2 ) ? ( ( out > 0 ) ? 0.2 : -0.2 ) : out;

  return out;
}

double MPC_Controller::pd_controller2( geometry_msgs::Pose clost_pose, double &last_e,
                                       const double p, const double d)
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
  tf::Quaternion q2( clost_pose.orientation.x, clost_pose.orientation.y,
                    clost_pose.orientation.z, clost_pose.orientation.w );
  tf::Matrix3x3 m2( q2 );
  m2.getRPY( roll2, pitch2, yaw2 );

  double angle_to_waypoint =
      std::atan2( clost_pose.position.y - robot_control_state.pose.position.y,
                  clost_pose.position.x - robot_control_state.pose.position.x );

  double e = constrainAngle_mpi_pi( -yaw + angle_to_waypoint ) + constrainAngle_mpi_pi( -yaw + yaw2 );
  std::cout << "angle _e :" << yaw<<"     "<< yaw2 << "\n\n\n";
  double ed = e - last_e;
  double out = p * e + d * ed;
  out /=2.0;
  // out *=0.05;
  out = ( std::abs( out ) > 1.0 ) ? ( ( out > 0 ) ? 1.0 : -1.0 ) : out;
  last_e = e;
  // error = e;
  return out;
}

void MPC_Controller::collision_detection( double &linear_vel, double &angluar_vel, double dt )
{

  double roll, pitch, yaw_;
  tf::Quaternion q( robot_control_state.pose.orientation.x, robot_control_state.pose.orientation.y,
                    robot_control_state.pose.orientation.z, robot_control_state.pose.orientation.w );
  tf::Matrix3x3 m( q );
  m.getRPY( roll, pitch, yaw_ );
  double theta = angluar_vel * dt + yaw_;
  double linear_vel_ = linear_vel; // robot_control_state.velocity_linear.x;
  // std::cout<<"liner_vel:  "<<linear_vel_<<"\n\n\n";
  double x = robot_control_state.pose.position.x + cos( theta ) * dt * linear_vel_;
  double y = robot_control_state.pose.position.y + sin( theta ) * dt * linear_vel_;

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

  for ( grid_map::LineIterator iterator( occupancy_map, front_right, mid_right );
        !iterator.isPastEnd(); ++iterator ) {
    // std::cout<<"value : "<<occupancy_map.at( "occupancy", *iterator );
    if ( occupancy_map.at( "occupancy", *iterator ) > 0 ) {
      front_right_collision = true;
      vehicle_control_interface_->executeTwist( cmd, robot_control_state, yaw, pitch, roll );
      // stop();
      std::cout << "collison !!!!!!!!!!!!!!1111\n\n\n\n";

      break;
    }
  }

  for ( grid_map::LineIterator iterator( occupancy_map, front_left, mid_left );
        !iterator.isPastEnd(); ++iterator ) {
    // std::cout<<"value : "<<occupancy_map.at( "occupancy", *iterator )<<"  ";

    if ( occupancy_map.at( "occupancy", *iterator ) > 0 ) {
      front_left_collision = true;
      vehicle_control_interface_->executeTwist( cmd, robot_control_state, yaw, pitch, roll );
      // stop();
      std::cout << "collison !!!!!!!!!!!!!!1111\n\n\n\n";

      break;
    }
  }

  for ( grid_map::LineIterator iterator( occupancy_map, mid_right, back_right );
        !iterator.isPastEnd(); ++iterator ) {
    // std::cout<<"value : "<<occupancy_map.at( "occupancy", *iterator );
    if ( occupancy_map.at( "occupancy", *iterator ) > 0 ) {
      back_right_collision = true;
      vehicle_control_interface_->executeTwist( cmd, robot_control_state, yaw, pitch, roll );
      // stop();
      std::cout << "collison !!!!!!!!!!!!!!1111\n\n\n\n";

      break;
    }
  }

  for ( grid_map::LineIterator iterator( occupancy_map, mid_left, back_left );
        !iterator.isPastEnd(); ++iterator ) {
    if ( occupancy_map.at( "occupancy", *iterator ) > 0 ) {
      back_left_collision = true;
      vehicle_control_interface_->executeTwist( cmd, robot_control_state, yaw, pitch, roll );
      // stop();
      std::cout << "collison !!!!!!!!!!!!!!1111\n\n\n\n";
      break;
    }
  }

  if ( front_left_collision) {
    angluar_vel -= 0.005;
    // linear_vel = 0.00;
    collision_detection( linear_vel, angluar_vel, dt );
    std::cout << "front_left side collision collision !!!!!!"
              << "\n\n\n";
  }
  else if (back_right_collision)
  {
    angluar_vel -= 0.005;
    // linear_vel = 0.00;
    collision_detection( linear_vel, angluar_vel, dt );
    std::cout << "back_right side collision collision !!!!!!"
              << "\n\n\n";
  }

  else if ( front_right_collision) {
    angluar_vel += 0.005;
    // linear_vel = 0.00;
    collision_detection( linear_vel, angluar_vel, dt );
    std::cout << "front_right side collision collision !!!!!!"
              << "\n\n\n";
  }
  else if (back_left_collision)
  {
    angluar_vel += 0.005;
    // linear_vel = 0.00;
    collision_detection( linear_vel, angluar_vel, dt );
    std::cout << "back_left side collision collision !!!!!!"
              << "\n\n\n";
  }
}
double MPC_Controller::calc_local_path()
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

    if ( curr_dist > 0.2 ) // search for points
    {
      break;
    } else {
      path_po_lenght = path_po_lenght + 1;
      //      co_points = co_points + 1;
      //      points[co_points][0] = current_path.poses[i].pose.position.x;
      //      points[co_points][1] = current_path.poses[i].pose.position.y;
    }
  }

  return ( pd_controller2( current_path.poses[st_point + path_po_lenght].pose, last_error_front, 2,
                           1 ) );

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
