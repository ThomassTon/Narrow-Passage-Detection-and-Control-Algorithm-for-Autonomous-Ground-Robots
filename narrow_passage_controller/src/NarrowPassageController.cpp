#include <NarrowPassageController.hpp>

namespace narrow_passgae_controller
{
NarrowPassageController::NarrowPassageController( ros::NodeHandle &nodeHandle ) : nh( nodeHandle )
{

  nh.setCallbackQueue( &queue_1 );
  narrow_passage_sub = nh.subscribe("/approach_goal", 1, &NarrowPassageController::narrow_passage_messageCallback, this );
  speed_pub = nh.advertise<std_msgs::Float32>( "/speed", 1 );
  smoothPathPublisher = nh.advertise<nav_msgs::Path>( "smooth_path_circle", 1, true );
  approachedPublisher = nh.advertise<narrow_passage_detection_msgs::NarrowPassageController>("endpoint_approached", 1, true );
  map_sub = nh.subscribe( "/move_base_lite_node/debug_planning_map", 1, &NarrowPassageController::map_messageCallback2, this );
  nh.setCallbackQueue( &queue_2 );
  stateSubscriber = nh.subscribe( "/odom", 1, &NarrowPassageController::stateCallback, this, ros::TransportHints().tcpNoDelay( true ) );
}

void NarrowPassageController::map_messageCallback2( const grid_map_msgs::GridMap &msg )
{
  grid_map::GridMapRosConverter::fromMessage( msg, occupancy_map );
}

void NarrowPassageController::narrow_passage_messageCallback(
    const narrow_passage_detection_msgs::NarrowPassage msg )
{
  reset();
  mid_point = msg.midpose;
  end_point = msg.endpose;
  extend_point = msg.extendpose;
  lookahead_detected = true;
}
void NarrowPassageController::reset(){
  approached_endpoint = false;
  approached_extendpoint = false;
  lookahead_detected = false;
}

void NarrowPassageController::stateCallback( const nav_msgs::Odometry odom_state )
{
  robot_pose = odom_state.pose.pose;
  if(lookahead_detected && approached_endpoint ==false){
    // ROS_INFO("Start to generate the path \n\n\n\n\n\n");
    path_to_approach(robot_pose, end_point, mid_point);
    if (approached_endpoint==false && endpoint_approached( end_point )) {
    narrow_passage_detection_msgs::NarrowPassageController msg;
    msg.approached_endpoint = true;
    msg.approached_extendpoint = false;
    approachedPublisher.publish( msg );
    approached_endpoint = true;  
    lookahead_detected = false;
    // ROS_INFO("get the end point \n\n\n\n");
    }
  }



  // if(approached_endpoint){

  //   if(endpoint_approached(extend_point) && approached_extendpoint==false){
  //     approached_extendpoint=true;
  //     narrow_passage_detection_msgs::NarrowPassageController msg;
  //     msg.approached_endpoint = true;
  //     msg.approached_extendpoint = true;
  //     approachedPublisher.publish( msg );
  //     reset();
  //   }
  //   if(approached_extendpoint==false){
  //     nav_msgs::Path extend;
  //     geometry_msgs::PoseStamped waypoint;
  //     waypoint.pose = extend_point;
  //     extend.poses.push_back( waypoint );

  //     extend.header.frame_id = "world";
  //     extend.header.stamp = ros::Time::now();
  //     smoothPathPublisher.publish( extend );
  //   }


  // }

}

double NarrowPassageController::compute_distance( grid_map::Position pos1, grid_map::Position pos2 )
{
  double x1 = pos1[0];
  double y1 = pos1[1];
  double x2 = pos2[0];
  double y2 = pos2[1];
  return std::sqrt( std::pow( x1 - x2, 2 ) + std::pow( y1 - y2, 2 ) );
}

bool NarrowPassageController::compareByDistance( robot_range &a, robot_range &b )
{
  return a.distance < b.distance;
}

void NarrowPassageController::path_to_approach( geometry_msgs::Pose start, geometry_msgs::Pose end,
                                                geometry_msgs::Pose mid )
{
  double start_x = 0;
  double start_y = 0;
  double end_x = end.position.x - start.position.x;
  double end_y = end.position.y - start.position.y;
  double mid_x = mid.position.x - start.position.x;
  double mid_y = mid.position.y - start.position.y;
  double R_x;
  double R_y;
  double r;
  // double angle_pi;
  // double s;
  // double yaw;

  R_x = ( 2 * end_x * mid_x * end_y - std::pow( end_x, 2 ) * mid_y + std::pow( end_y, 2 ) * mid_y -
          end_y * std::pow( end_x, 2 ) - std::pow( end_y, 3 ) ) /
        ( 2 * ( mid_x * end_y - mid_y * end_x ) );
  R_y = ( 2 * end_x * mid_y * end_y - std::pow( end_y, 2 ) * mid_x + std::pow( end_x, 2 ) * mid_x -
          end_x * std::pow( end_y, 2 ) - std::pow( end_x, 3 ) ) /
        ( 2 * ( -mid_x * end_y + mid_y * end_x ) );
  r = std::sqrt( std::pow( R_x, 2 ) + std::pow( R_y, 2 ) );

  R_x += start.position.x;
  R_y += start.position.y;

  double roll, pitch, yaw;

  // double angle_ = std::atan2( start.position.y - R_y, start.position.x - R_x );
  // double angle_2 = std::atan2( end.position.y - R_y, end.position.x - R_x );

  // double angle_diff = constrainAngle_mpi_pi( angle_2 - angle_ );
  // nav_msgs::Path circle;
  // double angle_diff_abs = std::abs( angle_diff ) / M_PI * 180;
  // for ( int i = 2; i < angle_diff_abs; i++ ) {
  //   double angle_plus = i / angle_diff_abs * angle_diff;
  //   angle_plus += angle_;
  //   double x = R_x + cos( angle_plus ) * r;
  //   double y = R_y + sin( angle_plus ) * r;
  //   geometry_msgs::PoseStamped waypoint;
  //   waypoint.pose.position.x = x;
  //   waypoint.pose.position.y = y;
  //   waypoint.pose.position.z = 0;
    
  //   circle.poses.push_back( waypoint );
  // }


  nav_msgs::Path circle_;
  nav_msgs::Path circle;
  double end_angle = std::atan2 (mid.position.y - end.position.y, mid.position.x - mid.position.x);
  double e_x = end.position.x - 0.1*cos(end_angle);
  double e_y = end.position.y - 0.1*sin(end_angle);
  
  double angle_end = std::atan2( end.position.y - R_y, end.position.x - R_x );
  double angle_e = std::atan2( e_y - R_y, e_x - R_x );
  double angle_diff = constrainAngle_mpi_pi( angle_e - angle_end);
  double angle_dir = (angle_diff>0.0)  ? 1.0:-1.0;

  bool abort = false;
  int a = 1;
  while (!abort)
  {
    double angle_waypoint = a/180.0*M_PI*angle_dir + angle_end;
    double x = R_x + cos( angle_waypoint ) * r;
    double y = R_y + sin( angle_waypoint ) * r;
    geometry_msgs::PoseStamped waypoint;
    waypoint.pose.position.x = x;
    waypoint.pose.position.y = y;
    waypoint.pose.position.z = 0;
    circle_.poses.push_back( waypoint );
    a++;
    if(std::sqrt(std::pow(x - start.position.x,2)+ std::pow(y - start.position.y,2))<0.05)
    {
      abort = true;
      break;
    }
  }

  for(int i=circle_.poses.size()-1; i>=0; i--){
    circle.poses.push_back( circle_.poses.at(i));
  }
  // std::cout<<"finish generate the path , size: "<<circle.poses.size()<<std::endl;
  geometry_msgs::PoseStamped waypoint;
  waypoint.pose = end;
  circle.poses.push_back( waypoint );
  for ( int i = 0; i < circle.poses.size() - 1; i++ ) {
    double angle =
        std::atan2( circle.poses[i + 1].pose.position.y - circle.poses[i].pose.position.y,
                    circle.poses[i + 1].pose.position.x - circle.poses[i].pose.position.x );
    double roll = 0;
    double pitch = 0;
    double yaw = angle;

    tf2::Quaternion quat;
    quat.setRPY( roll, pitch, yaw );

    circle.poses[i].pose.orientation.x = quat.getX();
    circle.poses[i].pose.orientation.y = quat.getY();
    circle.poses[i].pose.orientation.z = quat.getZ();
    circle.poses[i].pose.orientation.w = quat.getW();
  }
  if(check_path_collision(circle)){
    circle.poses.clear();
    circle.poses.push_back(waypoint);
    ROS_INFO("COLLISION ON THE PATH \n\n\n\n\n\n");
    return;
  }

  // waypoint.pose = mid;
  // circle.poses.push_back(waypoint);
  circle.header.frame_id = "world";
  circle.header.stamp = ros::Time::now();
  smoothPathPublisher.publish( circle );
}

bool NarrowPassageController::check_path_collision(nav_msgs::Path circle){
  const grid_map::Matrix& dist_data = occupancy_map["distance_transform"];
  for(int i=0; i< circle.poses.size();i++){
    grid_map::Index id;
    grid_map::Position pos(circle.poses[i].pose.position.x, circle.poses[i].pose.position.y);
    occupancy_map.getIndex(pos, id);
    if(dist_data(id[0],id[1])<0.25/0.05){
      return true;  // will has collision on the path
    }
  }
  return false;
}

bool NarrowPassageController::endpoint_approached( geometry_msgs::Pose end )
{
  double dis_diff = std::sqrt( std::pow( end.position.x - robot_pose.position.x, 2 ) +
                               std::pow( end.position.y - robot_pose.position.y, 2 ) );
  std::cout<<" dis_diff"<<dis_diff<<std::endl;
  if ( dis_diff < 0.05 ) {
    return true;
  }
  return false;
}

} // namespace narrow_passgae_controller
