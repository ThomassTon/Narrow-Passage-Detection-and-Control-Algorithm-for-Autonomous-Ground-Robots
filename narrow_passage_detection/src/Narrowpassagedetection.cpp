#include "Narrowpassagedetection.hpp"
#include <unistd.h>

namespace narrow_passage_detection
{

bool show = false;

Narrowpassagedetection::Narrowpassagedetection( ros::NodeHandle &nodeHandle ) : nh( nodeHandle )
{

  nh.setCallbackQueue( &queue_1 );
  map_sub = nh.subscribe( "/elevation_mapping/elevation_map_raw", 1,
                          &Narrowpassagedetection::map_messageCallback, this );
  extend_point_pub =
      nh.advertise<narrow_passage_detection_msgs::NarrowPassage>( "/approach_goal", 1 );
  nh.setCallbackQueue( &queue_2 );

  path_sub = nh.subscribe( "/smooth_path", 1, &Narrowpassagedetection::path_messageCallback, this );
  pose_sub = nh.subscribe( "/odom", 1, &Narrowpassagedetection::pose_messageCallback, this );
  endpoint_approached =
      nh.subscribe( "/narrow_passage_controller_node/endpoint_approached", 1,
                    &Narrowpassagedetection::endpoint_approaced_messageCallback, this );

  vel_sub = nh.subscribe( "/cmd_vel_raw", 1, &Narrowpassagedetection::vel_messageCallback, this );
  map_sub2 = nh.subscribe( "/move_base_lite_node/debug_planning_map", 1,
                           &Narrowpassagedetection::map_messageCallback2, this );
  map_pub = nh.advertise<grid_map_msgs::GridMap>( "/narrow_passage_map", 1 );
  width_pub = nh.advertise<std_msgs::String>( "/passage_width", 1 );
  approach_distacne_pub =
      nh.advertise<narrow_passage_detection_msgs::NarrowPassage>( "/narrow_passage_approach", 1 );

  // nh.setCallbackQueue(&queue_3);

  // path_sub = nh.subscribe("/smooth_path",1,&Narrowpassagedetection::path_messageCallback, this);

  // maxduration.fromSec(1.00);

  // mapUpdateTimer_ = nh.createTimer(maxduration, &Narrowpassagedetection::mapUpdateTimerCallback,
  // this, false, false); mapUpdateTimer_.start(); setupTimers(); initialize();
}

void Narrowpassagedetection::map_messageCallback2( const grid_map_msgs::GridMap &msg )
{
  grid_map::GridMapRosConverter::fromMessage( msg, occupancy_map );
}
void Narrowpassagedetection::path_messageCallback( const nav_msgs::Path &msg )
{
  // std::cout<<msg.poses[0]<<std::endl;

  // for(int i =0;i<msg.poses.size();)
  // {

  // }
  path_msg = msg;
  get_path = true;
}
void Narrowpassagedetection::adjust_map( grid_map::GridMap &map )
{
  for ( grid_map::GridMapIterator iterator( map ); !iterator.isPastEnd(); ++iterator ) {
    const grid_map::Index index( *iterator );
    float &value = map.get( "elevation" )( index( 0 ), index( 1 ) ); // elevation

    if ( value < 0.1 ) {
      value = NAN;
    }
  }
}

void Narrowpassagedetection::endpoint_approaced_messageCallback(
    const narrow_passage_detection_msgs::NarrowPassageController &msg )
{
  if ( msg.approached_endpoint || msg.approached_extendpoint ) {
    // get_smoothpath=false;
    // detection_count= 0;
    // narrow_passage_dectected = false;
  }
}

bool Narrowpassagedetection::lookahead_detection()
{
  int index = get_path_index( path_msg, 2.0 );
  tf::Quaternion quat;
  tf::quaternionMsgToTF( path_msg.poses[index].pose.orientation, quat );
  double roll_, pitch_, yaw_;
  bool isSuccess;
  tf::Matrix3x3( quat ).getRPY( roll_, pitch_, yaw_ );
  grid_map::Position robot_position2( path_msg.poses[index].pose.position.x,
                                      path_msg.poses[index].pose.position.y );
  grid_map::Length length2( 2, 2 );
  grid_map::GridMap map = outputmap.getSubmap( robot_position2, length2, isSuccess );

  bool narrow = generate_output2( path_msg.poses[index].pose.position.x,
                                  path_msg.poses[index].pose.position.y, yaw_, map, mid_pose, index );
  return narrow;
}

bool Narrowpassagedetection::robot_detection()
{
  int index = get_path_index( path_msg, 0.7 );
  tf::Quaternion quat;
  tf::quaternionMsgToTF( robot_pose.orientation, quat );
  double roll_, pitch_, yaw_;
  bool isSuccess;
  tf::Matrix3x3( quat ).getRPY( roll_, pitch_, yaw_ );
  grid_map::Position robot_position2( robot_pose.position.x + 0.5 * cos( yaw_ ),
                                      robot_pose.position.y + 0.5 * sin( yaw_ ) );
  grid_map::Length length2( 2, 2 );
  grid_map::GridMap map = outputmap.getSubmap( robot_position2, length2, isSuccess );

  bool narrow =
      generate_output2( robot_pose.position.x + 0.5 * cos( yaw_ ),
                        robot_pose.position.y + 0.5 * sin( yaw_ ), yaw_, map, mid_pose, index );
  return narrow;
}

void Narrowpassagedetection::map_messageCallback( const grid_map_msgs::GridMap &msg )
{

  // ROS_INFO("Received message: \n\n\n\n\n\n\n\n\n\n\n\n");
  grid_map::GridMapRosConverter::fromMessage( msg, elevationmap_ );
  getmap = true;
  adjust_map( elevationmap_ );
  outputmap = elevationmap_;
  grid_data = outputmap["elevation"];
  ros::Time start_time = ros::Time::now();
  bool isSuccess;
  grid_map::GridMap map;
  grid_map::Length length( 2.5, 2.5 );
  grid_map::Length length2( 2, 2 );
  if ( get_path ) {
    bool lookahead = lookahead_detection();
    if ( lookahead && extended_point == false ) {
      lookahead_narrow_passage_dectected = true;
      lookahead_detection_count++;
      // ROS_INFO("narrowa ssssss  %.3f \n\n\n", detection_count);
    }
  }
  if ( lookahead_detection_count > 3 && lookahead_narrow_passage_dectected == true ) {
    lookahead_narrow_passage_dectected = false;
    extended_point = true;
    geometry_msgs::Pose approach_pose = extend_point( mid_pose, 0.7, true );
    geometry_msgs::Pose extend_pose = extend_point( mid_pose, 0.7, false );
    extend_point_publisher( mid_pose, approach_pose, extend_pose );
    lookahead_detection_count = 0;
  }

  // grid_map::Position robot_position(robot_pose_msg.pose.pose.position.x,robot_pose_msg.pose.pose.position.y);
  // map = outputmap.getSubmap(robot_position,length, isSuccess);
  // generate_output(robot_pose_msg.pose.pose.position.x, robot_pose_msg.pose.pose.position.y, robot_yaw,map, mid_pose);

  ros::Time end_time = ros::Time::now();
  ros::Duration duration = end_time - start_time;

  // // 输出时间差
  // ROS_INFO( "Time elapsed: %.3f seconds", duration.toSec() );
  narrowmap_pub( outputmap );
}

void Narrowpassagedetection::pose_messageCallback( const nav_msgs::Odometry &pose )
{

  robot_pose = pose.pose.pose;
  // std::cout<<"yaw :"<<yaw<<std::endl;
}

void Narrowpassagedetection::vel_messageCallback( const geometry_msgs::Twist &msg )
{
  vel_msg = msg;
  if ( vel_msg.linear.x < -0.0002 ) {
    backward = true;
  }
  if ( vel_msg.linear.x > 0.0002 ) {
    backward = false;
  }
}

void Narrowpassagedetection::narrowmap_pub( grid_map::GridMap map )
{
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage( map, message );
  map_pub.publish( message );
}

bool Narrowpassagedetection::generate_output2( double pos_x, double pos_y, double yaw_,
                                               grid_map::GridMap map, geometry_msgs::Pose &pos,
                                               int index )
{
  std::vector<grid_map::Position> pos_buffer;
  create_ray2( pos_x, pos_y, yaw_, map, pos_buffer );
  grid_map::Position center( pos_x, pos_y );
  compute_passage_width2( map, center, yaw_, pos_buffer, pos, index );
  return ( compute_passage_width2( map, center, yaw_, pos_buffer, pos, index ) );
  // return true;
}

void Narrowpassagedetection::create_ray2( double pos_x, double pos_y, double yaw_,
                                          grid_map::GridMap map,
                                          std::vector<grid_map::Position> &pos_buffer )
{
  grid_map::Position center( pos_x, pos_y );
  for ( grid_map::SpiralIterator iterator( map, center, 0.4 ); !iterator.isPastEnd(); ++iterator ) {
    grid_map::Index index = *iterator;
    double value = map.at( "elevation", *iterator );
    if ( value > 0.40 && value != NAN ) {
      grid_map::Position pos;
      map.getPosition( index, pos );
      pos_buffer.push_back( pos );
    }
  }
  // if ( !pos_buffer.empty() ) {
  //   std::ofstream outputfile3( "/home/haolei/Documents/ray_detection.txt" );

  //   if ( outputfile3.is_open() ) {
  //     for ( const auto &value : pos_buffer ) {
  //       outputfile3 << value[0] << "    " << value[1] << "\n";
  //     }
  //     outputfile3.close();
  //   }
  // }
}

void Narrowpassagedetection::ray_detection2( double x, double y, double angle,
                                             grid_map::Position robot_position, grid_map::GridMap map )
{
  dis_buffer.clear();
  for ( grid_map::GridMapIterator iterator( map ); !iterator.isPastEnd(); ++iterator ) {
    const grid_map::Index index( *iterator );
    const float &value = map.get( "elevation" )( index( 0 ), index( 1 ) );
    if ( std::isfinite( value ) ) {
      grid_map::Position position;
      map.getPosition( index, position );
      grid_map::Position position1( position[0] - robot_position[0], position[1] - robot_position[1] );
      grid_map::Position position2( x, y );
      // double diff = isPointOnSegment(position1,position2,length);
      if ( isPointOnSegment( position1, position2, 0.999 ) ) {
        double dis = calculateDistance( position, robot_position );
        dis_buffer.push_back( { dis, index, position } );
      }
    }
  }

  if ( !dis_buffer.empty() ) {

    std::sort( dis_buffer.begin(), dis_buffer.end(), Narrowpassagedetection::compareByDis );
    grid_map::Index index = dis_buffer[0].index;
    int index1 = index[0];
    int index2 = index[1];
    auto result =
        std::find_if( ray_buffer.begin(), ray_buffer.end(), [index1, index2]( const auto &element ) {
          return element.index[0] == index1 && element.index[1] == index2;
        } );
    if ( result == ray_buffer.end() ) {
      ray_buffer.push_back(
          { angle, dis_buffer[0].distance, dis_buffer[0].index, dis_buffer[0].position } );
    }
    // if(ray_buffer.empty()){
    //     ray_buffer.push_back({angle,dis_buffer[0].distance,dis_buffer[0].index,dis_buffer[0].position});
    // }
    // else if(ray_buffer.back().index[0]!=index[0]){
    //     ray_buffer.push_back({angle,dis_buffer[0].distance,dis_buffer[0].index,dis_buffer[0].position});
    // }
  }
}

double Narrowpassagedetection::calculateDistance( const grid_map::Position &A,
                                                  const grid_map::Position &B )
{
  return std::sqrt( std::pow( B[0] - A[0], 2 ) + std::pow( B[1] - A[1], 2 ) );
}

bool Narrowpassagedetection::compareByDis( const dis_buffer_type &a, const dis_buffer_type &b )
{
  return a.distance < b.distance;
}
bool Narrowpassagedetection::compareByWidth( const passage_width_buffer_type &a,
                                             const passage_width_buffer_type &b )
{
  return a.wide < b.wide;
}

bool Narrowpassagedetection::compute_passage_width2( grid_map::GridMap map,
                                                     grid_map::Position center, double yaw,
                                                     std::vector<grid_map::Position> pos_buffer,
                                                     geometry_msgs::Pose &pos, int index )
{
  std::vector<point_info> point_buffer;
  for ( auto value : pos_buffer ) {
    double angle = std::atan2( value[1] - center[1], value[0] - center[0] );
    double angle_diff_center = constrainAngle_mpi_pi( yaw - angle );
    // std::cout<<" angle_diff"<<angle_diff_center<<"\n";
    int side = ( angle_diff_center > 0 ) ? 1 : 0;
    if ( std::abs( angle_diff_center ) < M_PI / 1.8 ) {
      point_info point_( value, side );
      point_buffer.push_back( point_ );
    }
  }
  //   if ( !point_buffer.empty() ) {
  //     std::ofstream outputfile6( "/home/haolei/Documents/width_buffer.txt" );
  //     if ( outputfile6.is_open() ) {
  //       for ( const auto &value : point_buffer ) {
  //         outputfile6 << value.position[0] << "   " << value.position[1] << "     " << value.side
  //                     << "\n";
  //       }
  //       outputfile6.close();
  //     }
  //   }
  std::vector<grid_map::Position> buffer1;
  std::vector<grid_map::Position> buffer2;

  for ( auto value : point_buffer ) {
    if ( value.side == 0 ) {
      buffer1.push_back( value.position );
    } else {
      buffer2.push_back( value.position );
    }
  }
  double min_distance = MAXFLOAT;
  grid_map::Position pos1;
  grid_map::Position pos2;
  for ( auto value : buffer1 ) {
    for ( auto value2 : buffer2 ) {
      double dis = calculateDistance( value, value2 );
      if ( dis < min_distance ) {
        pos1 = value;
        pos2 = value2;
        min_distance = dis;
      }
    }
  }
  if ( min_distance != MAXFLOAT && min_distance > 0.5 ) {
    // std::cout << " distance " << min_distance << "\n\n\n\n\n";
    // std::cout << "pos1: " << pos1[0] << "   " << pos1[1] << "     pos2:  " << pos2[0] << "   "
    //           << pos2[1] << "\n\n\n\n";
    mark_narrow_passage( pos1, pos2 );
    pos.position.x = 0.5 * ( pos1[0] + pos2[0] );
    pos.position.y = 0.5 * ( pos1[1] + pos2[1] );

    double dis = MAXFLOAT;
    int index_ = 0;
    for ( int i = index; i < path_msg.poses.size(); i++ ) {
      double dis_ = std::sqrt( std::pow( pos.position.x - path_msg.poses[i].pose.position.x, 2 ) +
                               std::pow( pos.position.y - path_msg.poses[i].pose.position.y, 2 ) );
      if ( dis_ < dis ) {
        dis = dis_;
        index_ = i;
      }
    }
    // std::cout<<"index:  "<<index_<<"\n\n\n\n";
    // double roll, pitch, yaw;
    // tf::Quaternion q( center.orientation.x, center.orientation.y, center.orientation.z,
    //                   center.orientation.w );
    // tf::Matrix3x3 m( q );
    // m.getRPY( roll, pitch, yaw );
    pos.orientation = path_msg.poses[index_].pose.orientation;
    double roll, pitch, yaw;
    tf::Quaternion q(
        path_msg.poses[index_].pose.orientation.x, path_msg.poses[index_].pose.orientation.y,
        path_msg.poses[index_].pose.orientation.z, path_msg.poses[index_].pose.orientation.w );
    tf::Matrix3x3 m( q );
    m.getRPY( roll, pitch, yaw );
    // std::cout<<"yaw___-:"<<yaw<<"\n\n\n\n\n";
    return true;
  }
  // if(!detected){
  //     width=0;
  //     ROS_INFO("CLASSIFICATION");
  //     buffer1.clear();
  //     buffer2.clear();
  //     classification(buffer1, buffer2, ray_buffer);

  //     width_buffer.clear();
  //     if(buffer1.size()==0||buffer2.size()==0){
  //         return;
  //     }
  //     for(int i=0;i<buffer1.size();i++)
  //     {
  //         for(int j=0; j<buffer2.size();j++)
  //         {
  //             const double distance = calculateDistance(buffer1[i].position,buffer2[j].position);
  //             width_buffer.push_back({distance,buffer1[i].index,buffer2[j].index, buffer1[i].position, buffer2[j].position});
  //         }
  //     }

  //     std::sort(width_buffer.begin(),width_buffer.end(),
  //     Narrowpassagedetection::compareByWidth); width = width_buffer[0].wide;

  // }

  // publish width

  return false;
}

bool Narrowpassagedetection::is_obstacle( const passage_width_buffer_type &a, grid_map::GridMap map )
{
  const grid_map::Position position1 = a.position1;
  const grid_map::Position position2 = a.position2;
  int num_obstacle = 0;
  for ( grid_map::GridMapIterator iterator( map ); !iterator.isPastEnd(); ++iterator ) {

    const grid_map::Index index( *iterator );
    grid_map::Position position3;
    map.getPosition( index, position3 );
    if ( isPointOnSegment( position1, position2, position3 ) ||
         isPointOnSegment( position2, position1, position3 ) ) {
      grid_map::Index occupancy_index;
      occupancy_map.getIndex( position3, occupancy_index );
      if ( occupancy_map["occupancy"]( occupancy_index[0], occupancy_index[1] ) == 0 ||
           occupancy_map["occupancy"]( occupancy_index[0], occupancy_index[1] ) == NAN ) {
        grid_map::Index index2;
        outputmap.getIndex( position3, index2 );
        outputmap["elevation"]( index2[0], index2[1] ) = 0;
        // num_no_obstacle++;
      } else {
        num_obstacle++;
      }
    }
  }
  if ( num_obstacle > 5 ) {
    return false;
  }
  return true;
}

void Narrowpassagedetection::mark_narrow_passage( grid_map::Position pos1, grid_map::Position pos2 )
{

  for ( grid_map::GridMapIterator iterator( outputmap ); !iterator.isPastEnd(); ++iterator ) {
    const grid_map::Index index( *iterator );
    grid_map::Position position3;
    outputmap.getPosition( index, position3 );
    if ( isPointOnSegment( pos1, pos2, position3 ) || isPointOnSegment( pos2, pos1, position3 ) ) {
      outputmap["elevation"]( index[0], index[1] ) = 0;
      //   std::cout<<"marked !!!!!!!\n\n\n";
    }
  }
}

bool Narrowpassagedetection::isPointOnSegment( const grid_map::Position A, const grid_map::Position B,
                                               const grid_map::Position C )
{
  double vectorAB_x = B[0] - A[0];
  double vectorAB_y = B[1] - A[1];
  double vectorAC_x = C[0] - A[0];
  double vectorAC_y = C[1] - A[1];

  double dotProduct = vectorAB_x * vectorAC_x + vectorAB_y * vectorAC_y;

  double lengthAB = calculateDistance( A, B );

  double lengthAC = calculateDistance( A, C );

  // 判断点 C 是否在线段 AB 上
  return ( ( dotProduct / ( lengthAB * lengthAC ) > 0.99 ) && lengthAC < lengthAB && lengthAC != 0.0 );
}

bool Narrowpassagedetection::isPointOnSegment( const grid_map::Position A,
                                               const grid_map::Position B, float max )
{
  double vectorOA_x = A[0] - 0;
  double vectorOA_y = A[1] - 0;
  double vectorOB_x = B[0] - 0;
  double vectorOB_y = B[1] - 0;

  double dotProduct = vectorOA_x * vectorOB_x + vectorOA_y * vectorOB_y;

  grid_map::Position O( 0, 0 );
  double lengthOB = calculateDistance( O, B );
  double lenghtOA = calculateDistance( O, A );
  // 判断点 C 是否在线段 AB 上
  return ( dotProduct / ( lenghtOA * lengthOB ) > max );
}

bool Narrowpassagedetection::compareByPose( const ray_buffer_type &a, const ray_buffer_type &b )
{

  return ( a.position[0] != b.position[0] ) ? ( a.position[0] < b.position[0] )
                                            : ( a.position[1] < b.position[1] );
}

int Narrowpassagedetection::get_path_index( const nav_msgs::Path path_msg, const float distance )
{
  int count = path_msg.poses.size();
  float distance_ = 0.0;
  int index = 1;
  for ( ; index < count; index++ ) {
    distance_ += std::sqrt(
        std::pow( path_msg.poses[index].pose.position.x - path_msg.poses[index - 1].pose.position.x,
                  2 ) +
        std::pow( path_msg.poses[index].pose.position.y - path_msg.poses[index - 1].pose.position.y,
                  2 ) );
    if ( distance_ > distance ) {
      return index;
    }
  }
  return index;
}

geometry_msgs::Pose Narrowpassagedetection::extend_point( geometry_msgs::Pose pose, float distance,
                                                          bool extend_or_approach )
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF( pose.orientation, quat );
  double roll_, pitch_, yaw_;
  tf::Matrix3x3( quat ).getRPY( roll_, pitch_, yaw_ );
  double angle = yaw_;
  //   std::cout<<"yaw_ :: "<<yaw_;
  if ( extend_or_approach ) {
    angle -= M_PI;
  }
  //   if ( backward ) {
  //     angle -= M_PI;
  //   }
  double x = distance * std::cos( angle );
  double y = distance * std::sin( angle );

  geometry_msgs::Pose pose_;
  pose_.orientation = pose.orientation;
  pose_.position.x = pose.position.x + x;
  pose_.position.y = pose.position.y + y;
  pose_.position.z = pose.position.z;
  grid_map::Position position(pose_.position.x,pose_.position.y);
  grid_map::Index id;
  occupancy_map.getIndex(position,id);
  
  if(occupancy_map.get("distance_transform")(id[0], id[1])<0.6){
    grid_map::Position mid_pose(pose.position.x, pose.position.y);
    geometry_msgs::Pose adjust_pose;
    if(adjust_point(pose_,adjust_pose, mid_pose, distance)){
      return adjust_pose;
    }
  }

  return pose_;
}
void Narrowpassagedetection::touchDistanceField( const grid_map::Matrix &dist_trans_map,
                                                 const grid_map::Index &current_point,
                                                 const int idx_x, const int idx_y,
                                                 float &highest_val, grid_map::Index &highest_index, grid_map::Position mid_pose, double dis )
{
  if (dist_trans_map(idx_x, idx_y) == std::numeric_limits<float>::max())
        return;

      float this_delta = dist_trans_map(idx_x, idx_y) - dist_trans_map(current_point(0),current_point(1));
      grid_map::Position pose;
      grid_map::Index id(idx_x,idx_y);
      occupancy_map.getPosition(id,pose);
      float distance_to_mid = std::sqrt(std::pow(pose[0]-mid_pose[0],2)+ std::pow(pose[1]-mid_pose[1],2));
      double dis_delta = distance_to_mid-dis;
      if ( (this_delta > 0.0f) && (this_delta > highest_val) && (dis_delta>0.0)&&(dis_delta<0.3)){
        highest_val = this_delta;
        highest_index = grid_map::Index(idx_x, idx_y);
      }
}

bool Narrowpassagedetection::adjust_point( geometry_msgs::Pose &start_pose ,geometry_msgs::Pose &adjusted_pose, grid_map::Position mid_pose, double dis )
{
  const grid_map::Matrix& dist_data = occupancy_map["distance_transform"];
  grid_map::Index current_index;
  grid_map::Index next_index;
  grid_map::Position pose(start_pose.position.x, start_pose.position.y);
  double dist_from_obstacle = 0.0;
  occupancy_map.getIndex(pose, current_index);
  bool abort =false;
  while(!abort)
  {
    float highest_cost = 0.0f;
    touchDistanceField(dist_data,
                            current_index,
                            current_index(0)-1,
                            current_index(1),
                            highest_cost,
                            next_index, mid_pose,dis );

    touchDistanceField(dist_data,
                            current_index,
                            current_index(0),
                            current_index(1)-1,
                            highest_cost,
                            next_index, mid_pose,dis);

    touchDistanceField(dist_data,
                            current_index,
                            current_index(0),
                            current_index(1)+1,
                            highest_cost,
                            next_index, mid_pose,dis);

    touchDistanceField(dist_data,
                            current_index,
                            current_index(0)+1,
                            current_index(1),
                            highest_cost,
                            next_index, mid_pose,dis);

    touchDistanceField(dist_data,
                            current_index,
                            current_index(0)-1,
                            current_index(1)-1,
                            highest_cost,
                            next_index, mid_pose,dis);

    touchDistanceField(dist_data,
                            current_index,
                            current_index(0)-1,
                            current_index(1)+1,
                            highest_cost,
                            next_index, mid_pose,dis);

    touchDistanceField(dist_data,
                            current_index,
                            current_index(0)+1,
                            current_index(1)-1,
                            highest_cost,
                            next_index, mid_pose,dis);

    touchDistanceField(dist_data,
                            current_index,
                            current_index(0)+1,
                            current_index(1)+1,
                            highest_cost,
                            next_index, mid_pose,dis);


    dist_from_obstacle = dist_data(current_index(0), current_index(1));

    //std::cout << "curr_index: " << current_index << "\ndist_from_obstacle: " << dist_from_obstacle << "dist_from_start: " << dist_from_start << " highest cost: " << highest_cost << "\n";

    // If gradient could not be followed..
    if (highest_cost == 0.0f){
      if (dist_from_obstacle < 0.6){
        ROS_WARN("Could not find gradient of distance transform leading to free area, returning original pose");
        adjusted_pose = start_pose;
        return false;

      // Could not reach desired distance from obstacles, but enough clearance
      }else if (dist_from_obstacle < 0.7){
        ROS_WARN("Could not find gradient of distance transform reaching desired final distance");
        abort = true;

      }else{
        ROS_INFO("Reached final distance");
        abort = true;
      }

    // Gradient following worked
    }else{

      if (dist_from_obstacle > 0.7){
        abort = true;

      // Otherwise continue gradient following
      }else{
        current_index = next_index;
      }
    }
  }

  grid_map::Position adjust_position;
  occupancy_map.getPosition(current_index,adjust_position);
  adjusted_pose.position.x = adjust_position[0];
  adjusted_pose.position.y = adjust_position[1];
  return true;
}

bool Narrowpassagedetection::finde_intersection_point( std::vector<passage_width_buffer_type> width_buffer,
                                                       nav_msgs::Path &msg, geometry_msgs::Pose &pos )
{
  int count = msg.poses.size();
  // grid_map::Position mid_pos()
  float pos_x = ( width_buffer[0].position1[0] + width_buffer[0].position2[0] ) / 2.0;
  float pos_y = ( width_buffer[0].position1[1] + width_buffer[0].position2[1] ) / 2.0;
  int index = 0;
  float distance = MAXFLOAT;
  for ( int i = 0; i < count; i++ ) {
    float cost = std::sqrt( std::pow( msg.poses[i].pose.position.x - pos_x, 2 ) +
                            std::pow( msg.poses[i].pose.position.y - pos_y, 2 ) );
    if ( cost < distance ) {
      distance = cost;
      index = i;
    }
  }
  if ( distance < 0.05 ) {

    pos.position = msg.poses[index].pose.position;
    //            pos.position.x = pos_x;
    //            pos.position.y = pos_y;
    return true;
  }
  return false;
}

bool Narrowpassagedetection::is_on_path( std::vector<passage_width_buffer_type> width_buffer,
                                         nav_msgs::Path &msg, geometry_msgs::Pose &pos )
{
  int count = msg.poses.size();
  // grid_map::Position mid_pos()
  float pos_x = ( width_buffer[0].position1[0] + width_buffer[0].position2[0] ) / 2.0;
  float pos_y = ( width_buffer[0].position1[1] + width_buffer[0].position2[1] ) / 2.0;
  int index = 0;
  float distance = MAXFLOAT;
  for ( int i = 0; i < count; i++ ) {
    float cost = std::sqrt( std::pow( msg.poses[i].pose.position.x - pos_x, 2 ) +
                            std::pow( msg.poses[i].pose.position.y - pos_y, 2 ) );
    if ( cost < distance ) {
      distance = cost;
      index = i;
    }
  }
  if ( distance < 0.05 ) {

    pos = msg.poses[index].pose;
    // pos.position = msg.poses[index].pose.position;

    return true;
  }
  return false;
}

void Narrowpassagedetection::extend_point_publisher( geometry_msgs::Pose mid_pos,
                                                     geometry_msgs::Pose end_pos,
                                                     geometry_msgs::Pose extend_pos )
{
  narrow_passage_detection_msgs::NarrowPassage msg;
  msg.midpose = mid_pos;
  msg.endpose = end_pos;
  msg.extendpose = extend_pos;
  extend_point_pub.publish( msg );
}

void Narrowpassagedetection::extend_point_publisher( geometry_msgs::Pose mid_pos,
                                                     geometry_msgs::Pose end_pos )
{
  narrow_passage_detection_msgs::NarrowPassage msg;
  msg.midpose = mid_pos;
  msg.endpose = end_pos;
  extend_point_pub.publish( msg );
}

bool Narrowpassagedetection::approach_distance( nav_msgs::Odometry robot_pose_msg,
                                                geometry_msgs::Pose mid_pose, float &distance,
                                                nav_msgs::Path path_msg )
{
  float x_1 = robot_pose_msg.pose.pose.position.x;
  float y_1 = robot_pose_msg.pose.pose.position.y;
  float x_2 = mid_pose.position.x;
  float y_2 = mid_pose.position.y;
  float distance_ = 0;
  int count = path_msg.poses.size();
  distance_ = std::sqrt( std::pow( x_1 - x_2, 2 ) + std::pow( x_2 - y_2, 2 ) );
  //        for (int i =1; i<count;i++){
  //            distance_ +=std::sqrt(std::pow(path_msg.poses[i-1].pose.position.x-path_msg.poses[i].pose.position.x,
  //            2) + std::pow(path_msg.poses[i-1].pose.position.y - path_msg.poses[i].pose.position.y, 2));
  //            if(std::sqrt(std::pow(x_2-path_msg.poses[i].pose.position.x,2 ) + std::pow(y_2 - path_msg.poses[i].pose.position.y, 2))< 0.1){
  //                break;
  //            }
  //
  //        }
  distance = distance_;

  if ( distance_ < 2.5 ) {
    return true;
  }
  return false;
}

} // namespace narrow_passage_detection