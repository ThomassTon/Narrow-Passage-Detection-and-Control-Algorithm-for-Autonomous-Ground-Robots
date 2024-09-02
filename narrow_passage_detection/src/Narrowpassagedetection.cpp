#include "Narrowpassagedetection.hpp"
#include <unistd.h>

namespace narrow_passage_detection
{

bool show = false;

Narrowpassagedetection::Narrowpassagedetection( ros::NodeHandle &nodeHandle ) : nh( nodeHandle )
{

  nh.setCallbackQueue( &queue_1 );
  map_sub = nh.subscribe( "/elevation_mapping/elevation_map", 1,  // /elevation_mapping_rgbd/elevation_map_raw
                          &Narrowpassagedetection::map_messageCallback, this );
  extend_point_pub =
      nh.advertise<narrow_passage_detection_msgs::NarrowPassage>( "/narrow_passage_entrance", 1 );
  // nh.setCallbackQueue( &queue_2 );

  path_sub = nh.subscribe( "/smooth_path", 1, &Narrowpassagedetection::path_messageCallback, this );
  pose_sub = nh.subscribe( "/odom", 1, &Narrowpassagedetection::pose_messageCallback, this );
  endpoint_approached =
      nh.subscribe( "/endpoint_approached", 1,
                    &Narrowpassagedetection::endpoint_approaced_messageCallback, this );

  vel_sub = nh.subscribe( "/cmd_vel_raw", 1, &Narrowpassagedetection::vel_messageCallback, this );
  map_sub2 = nh.subscribe( "/move_base_lite_node/debug_planning_map", 1,
                           &Narrowpassagedetection::map_messageCallback2, this );
  map_pub = nh.advertise<grid_map_msgs::GridMap>( "/narrow_passage_map", 1 );
  map_pub2 = nh.advertise<grid_map_msgs::GridMap>( "/narrow_passage_map_det", 1 );

  width_pub = nh.advertise<std_msgs::String>( "/passage_width", 1 );
  narrow_passage_detected_pub = nh.advertise<narrow_passage_detection_msgs::NarrowPassageDetection>("/narrow_passage_detected", 1);
  approach_distacne_pub =
      nh.advertise<narrow_passage_detection_msgs::NarrowPassage>( "/narrow_passage_approach", 1 );

  // nh.setCallbackQueue(&queue_3);

  // path_sub = nh.subscribe("/smooth_path",1,&Narrowpassagedetection::path_messageCallback, this);
  nh.setCallbackQueue( &queue_2 );

  maxduration.fromSec(0.5);

  mapUpdateTimer_ = nh.createTimer(maxduration, &Narrowpassagedetection::mapUpdateTimerCallback, this, false, false); 
  // mapUpdateTimer_.start(); 

}
void Narrowpassagedetection::mapUpdateTimerCallback(const ros::TimerEvent&){
  // if(getmap){
  //   narrowmap_pub( elevationmap_ );
  // last_time
  ros::Time now_time = ros::Time::now();
  ros::Duration interval = now_time - last_time;
  ROS_INFO("Function called, interval: %f seconds", interval.toSec());

  ros::Duration timd_diff = now_time - path_get_time;
  if(get_elevation_map&&(timd_diff.toSec()<3.0)){
    detecting();
    //    if(robot_detection2()){
    //   grid_map_msgs::GridMap message;
    //   grid_map::GridMapRosConverter::toMessage( outputmap, message );
    //   map_pub2.publish( message );
    // }
  }

  last_time = ros::Time::now();
  // }
}

void Narrowpassagedetection::detecting(){
  ros::Time start_time = ros::Time::now();
  bool isSuccess;
  grid_map::GridMap map;
  grid_map::Length length( 2.5, 2.5 );
  grid_map::Length length2( 2, 2 );
  geometry_msgs::Pose mid_pose;
  geometry_msgs::Pose mid_pose2;
 
  if ( get_path ) { //get_path
  
    bool lookahead = lookahead_detection(mid_pose);
    if ( lookahead==true && extended_point == false ) {
      lookahead_narrow_passage_dectected = true;
      // lookahead_detection_count++;
      // ROS_INFO("narrowa ssssss  %.3f \n\n\n", detection_count);
    }
    if((lookahead ==false || approach_end_point ==true) && extended_point == true && robot_detection() ==false){
      narrow_passage_detection_msgs::NarrowPassageDetection msg_2;
      msg_2.narrow_passage_detected = false;
      ROS_INFO("through out the narrow passage!!!!!!!!!!! \n\n\n\n\n\n");
      narrow_passage_detected_pub.publish(msg_2);
      reset();
    }
  }
  if ( lookahead_detection_count > 10 && lookahead_narrow_passage_dectected == true && extended_point ==false) {  // could be 2

    std::cout<<"detected a narrow passage !!!!!!!!!!!!!!!!!!!!!!!! \n\n\n\n\n\n";
    // lookahead_narrow_passage_dectected = false;
    extended_point = true;
    geometry_msgs::Pose approach_pose = extend_point( mid_pose, 0.1, true );

    geometry_msgs::Pose extend_pose;
    extend_point_publisher( mid_pose, approach_pose, extend_pose );
    lookahead_detection_count = 0;
    // grid_map_msgs::GridMap message;
    // grid_map::GridMapRosConverter::toMessage( elevationmap_, message );
    // map_pub2.publish( message );

  }
  // if(robot_detection2()){
  //   grid_map_msgs::GridMap message;
  //   grid_map::GridMapRosConverter::toMessage( outputmap, message );
  //   map_pub2.publish( message );
  // }


  // grid_map::Position robot_position(robot_pose_msg.pose.pose.position.x,robot_pose_msg.pose.pose.position.y);
  // map = outputmap.getSubmap(robot_position,length, isSuccess);
  // generate_output(robot_pose_msg.pose.pose.position.x, robot_pose_msg.pose.pose.position.y, robot_yaw,map, mid_pose);

  ros::Time end_time = ros::Time::now();
  ros::Duration duration = end_time - start_time;
  narrowmap_pub( elevationmap_ );

  // // 输出时间差
  // ROS_INFO( "Time elapsed: %.3f seconds", duration.toSec() );
}

void Narrowpassagedetection::map_messageCallback2( const grid_map_msgs::GridMap &msg )
{
  grid_map::GridMapRosConverter::fromMessage( msg, occupancy_map );
  getmap = true;
}
void Narrowpassagedetection::path_messageCallback( const nav_msgs::Path &msg )
{
  // std::cout<<msg.poses[0]<<std::endl;

  // for(int i =0;i<msg.poses.size();)
  // {

  // }
  path_get_time = ros::Time::now();
  path_msg = msg;
  get_path = true;
}
void Narrowpassagedetection::computegradient(grid_map::GridMap &map){
  cv::Mat input_img;
  if(! grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "elevation", CV_8UC1, input_img)){
    return;
  }

  int len1= input_img.rows;
  int len2= input_img.cols;

  // cv::imwrite("/home/yuan/Documents/input.jpg",input_img);

  cv::Mat gradientX(len1,len2,CV_8UC1), gradientY(len1,len2,CV_8UC1),magnitude(len1,len2,CV_8UC1), gaussian_img(len1,len2,CV_8UC1);
  // cv::GaussianBlur(input_img, input_img, cv::Size(3, 3), 0);
  
  // cv::Mat kernal_mat2 = (cv::Mat_<float>(3, 3) <<
  // 1 / 16.0f, 2 / 16.0f, 1 / 16.0f,
  // 2 / 16.0f, 4 / 16.0f, 2 / 16.0f,
  // 1 / 16.0f, 2 / 16.0f, 1 / 16.0f);
  // cv::filter2D(input_img,gaussian_img,-1,kernal_mat2);


  // cv::Mat kernal_mat1 = (cv::Mat_<char>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
  // cv::filter2D(gaussian_img,gaussian_img,-1,kernal_mat1);


  cv::Sobel(input_img, gradientX, CV_32F, 1, 0, 3);
  cv::Sobel(input_img, gradientY, CV_32F, 0, 1, 3);
  cv::cartToPolar(gradientX, gradientY, gradient, direction, true);
  // std::cout<<"compute "<<"\n\n\n\n\n\n\n\n\n"<<std::endl;
  gradient.convertTo(gradient,CV_8UC1);



  for(int i=0;i<gradient.rows;i++)  
  {  
      for(int j=0;j<gradient.cols;j++)  
      {  

          if(gradient.at<uchar>(i,j)< uchar(50))
          {
              gradient.at<uchar>(i,j)=uchar(0);
          }
          else{
              gradient.at<uchar>(i,j) = uchar(255);
          }
      }  
  }
  convert_from_gradient(gradient,map);


}

void Narrowpassagedetection::convert_from_gradient(cv::Mat _gradient, grid_map::GridMap &map){
  for (grid_map::GridMapIterator iterator(map);!iterator.isPastEnd(); ++iterator ){
    const grid_map::Index index(*iterator);
    float& value = map.get( "elevation" )( index( 0 ), index( 1 ) );
    const grid_map::Index imageIndex(iterator.getUnwrappedIndex());
    const float maxValue = map.get("elevation").maxCoeffOfFinites();
    if(_gradient.at<uchar>(imageIndex(0),imageIndex(1))== uchar(0)&&std::isfinite(value)){  //&value<0.7*maxValue
        value = NAN;
    }          

  }
}
void Narrowpassagedetection::adjust_map( grid_map::GridMap &map )
{

  for ( grid_map::GridMapIterator iterator( map ); !iterator.isPastEnd(); ++iterator ) {
    const grid_map::Index index( *iterator );
    float &value = map.get( "elevation" )( index( 0 ), index( 1 ) ); // elevation

    if ( value<0.1 ) {
      value = NAN;
    }
  }

  computegradient(map);
  for ( grid_map::GridMapIterator iterator( map ); !iterator.isPastEnd(); ++iterator ) {
    const grid_map::Index index( *iterator );
    float &value = map.get( "elevation" )( index( 0 ), index( 1 ) ); // elevation

    if ( value<0.01 ) {
      value = NAN;
    }
  }

}

void Narrowpassagedetection::endpoint_approaced_messageCallback(
    const narrow_passage_detection_msgs::NarrowPassageController &msg )
{
  if ( msg.approached_endpoint || msg.approached_extendpoint ) {

    approach_end_point = true;
    // get_smoothpath=false;
    // extended_point=false;
    // detection_count= 0;
    // narrow_passage_dectected = false;
  }
}

bool Narrowpassagedetection::lookahead_detection(geometry_msgs::Pose &mid_pose)
{
  // int index = get_path_index( path_msg, 2.0 );
  // tf::Quaternion quat;
  // tf::quaternionMsgToTF( path_msg.poses[index].pose.orientation, quat );
  // double roll_, pitch_, yaw_;
  // bool isSuccess;
  // tf::Matrix3x3( quat ).getRPY( roll_, pitch_, yaw_ );
  // grid_map::Position robot_position2( path_msg.poses[index].pose.position.x,
  //                                     path_msg.poses[index].pose.position.y );
  // grid_map::Length length2( 2, 2 );
  // grid_map::GridMap map = outputmap.getSubmap( robot_position2, length2, isSuccess );
  // bool narrow = generate_output2( path_msg.poses[index].pose.position.x,
  //                                 path_msg.poses[index].pose.position.y, yaw_, map, mid_pose, index );
  /*------------------------------------------------------------------------------------*/
  bool narrow = false;
  double min_width = MAXFLOAT;
  int count=0;
  for(double i =2.0; i>0.3; i -=0.1)
  {
    int index = get_path_index( path_msg, i );
    tf::Quaternion quat;
    tf::quaternionMsgToTF( path_msg.poses[index].pose.orientation, quat );
    double roll_, pitch_, yaw_;
    bool isSuccess;
    tf::Matrix3x3( quat ).getRPY( roll_, pitch_, yaw_ );
    grid_map::Position robot_position2( path_msg.poses[index].pose.position.x,
                                        path_msg.poses[index].pose.position.y );
    grid_map::Length length2( 2, 2 );
    grid_map::GridMap map = elevationmap_.getSubmap( robot_position2, length2, isSuccess );
    bool narrow_ = generate_output2( path_msg.poses[index].pose.position.x, path_msg.poses[index].pose.position.y, yaw_, map, mid_pose, index, min_width );
    narrow |= narrow_;
    // narrow |= narrow_;
    if(narrow_){
      count++;
    }
    if(min_width-global_min_width>-0.05 && min_width-global_min_width<0.05 &&count>1){
      lookahead_detection_count = 11;
    }
    global_min_width = min_width<global_min_width? min_width:global_min_width;
    
    
  }

  return narrow;
}
bool Narrowpassagedetection::robot_detection2()
{

  bool narrow = false;
  double min_width = MAXFLOAT;
  
  bool narrow2 = false;
  double pos_x = robot_pose.position.x;
  double pos_y = robot_pose.position.y;
  grid_map::Position center(pos_x, pos_y);
  tf::Quaternion quat;
  tf::quaternionMsgToTF( robot_pose.orientation, quat );
  double roll_, pitch_, yaw_;
  bool isSuccess;
  tf::Matrix3x3( quat ).getRPY( roll_, pitch_, yaw_ );
  grid_map::Length length2( 2, 2 );
  grid_map::GridMap map = elevationmap_.getSubmap( center, length2, isSuccess );
  geometry_msgs::Pose mid_;
  narrow = generate_output2( pos_x, pos_y, yaw_, map, mid_, 0 ,min_width);

  // narrow |=narrow2;
  return narrow;
}

bool Narrowpassagedetection::robot_detection()
{

  bool narrow = false;
  double min_width = MAXFLOAT;
  bool isSuccess;
  for(double i =0.4; i<-0.01; i-=0.1)
  {
    int index = get_path_index( path_msg, i );
    tf::Quaternion quat;
    tf::quaternionMsgToTF( path_msg.poses[index].pose.orientation, quat );
    double roll_, pitch_, yaw_;
    tf::Matrix3x3( quat ).getRPY( roll_, pitch_, yaw_ );
    grid_map::Position robot_position2( path_msg.poses[index].pose.position.x,
                                        path_msg.poses[index].pose.position.y );
    grid_map::Length length2( 2, 2 );
    grid_map::GridMap map = elevationmap_.getSubmap( robot_position2, length2, isSuccess );
    geometry_msgs::Pose mid_;

    narrow |= generate_output2( path_msg.poses[index].pose.position.x, path_msg.poses[index].pose.position.y, yaw_, map, mid_, index ,min_width);
  }
 
  bool narrow2 = false;
  double pos_x = robot_pose.position.x;
  double pos_y = robot_pose.position.y;
  grid_map::Position center(pos_x, pos_y);
  grid_map::Length length2( 2, 2 );
  grid_map::GridMap map = elevationmap_.getSubmap( center, length2, isSuccess );
  for ( grid_map::CircleIterator iterator( elevationmap_, center, 0.5 ); !iterator.isPastEnd(); ++iterator ) {
    double value = elevationmap_.at( "elevation", *iterator );
    if ( value > 0.40 && value != NAN ) {
      narrow2 = true;
    }
  }
  narrow |=narrow2;
  return narrow;
}

void Narrowpassagedetection::reset(){
  lookahead_detection_count = 0;
  lookahead_narrow_passage_dectected = false;
  extended_point = false;
  global_min_width = MAXFLOAT;
  approach_end_point  =false;
}
void printVector(const std::vector<std::string>& vec) {
    for (const auto& str : vec) {
        std::cout << str << " ";
    }
    std::cout << "\n";
}
void Narrowpassagedetection::map_messageCallback( const grid_map_msgs::GridMap &msg )
{

  // ROS_INFO("Received message: \n\n\n\n\n\n\n\n\n\n\n\n");
  grid_map::GridMapRosConverter::fromMessage( msg, outputmap );
  adjust_map( outputmap );
  // outputmap = elevationmap_;
  elevationmap_ = outputmap;
  // printVector(outputmap.getBasicLayers());
  ros::Time now_time = ros::Time::now();
  ros::Duration timd_diff = now_time - path_get_time;
  if((timd_diff.toSec()<3.0)){
    detecting();
  }
  narrowmap_pub( elevationmap_ );

  get_elevation_map = true;
    // std:: cout<<"step -2\n\n\n\n";


}

void Narrowpassagedetection::pose_messageCallback( const nav_msgs::Odometry &pose )
{

  robot_pose = pose.pose.pose;
  // std::cout<<"yaw :"<<yaw<<std::endl;
  if(get_elevation_map){
      // std:: cout<<"step -1\n\n\n\n";
    // if(robot_detection2()){
    //   grid_map_msgs::GridMap message;
    //   grid_map::GridMapRosConverter::toMessage( outputmap, message );
    //   map_pub2.publish( message );
    // }

    // detecting();
    // robot_detection2();
  }
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
                                               int index,double &min_width )
{
  std::vector<grid_map::Position> pos_buffer;
  create_ray2( pos_x, pos_y, yaw_, map, pos_buffer );
  grid_map::Position center( pos_x, pos_y );
  // compute_passage_width2( map, center, yaw_, pos_buffer, pos, index );
  return ( compute_passage_width2( map, center, yaw_, pos_buffer, pos, index ,min_width) );
  // return true;
}

void Narrowpassagedetection::create_ray2( double pos_x, double pos_y, double yaw_,
                                          grid_map::GridMap map,
                                          std::vector<grid_map::Position> &pos_buffer )
{
  grid_map::Position center( pos_x, pos_y );
  for ( grid_map::SpiralIterator iterator( map, center, 0.5); !iterator.isPastEnd(); ++iterator ) {
    grid_map::Index index = *iterator;
    double value = map.at( "elevation", *iterator );
    if ( value > 0.20 && value != NAN ) {
      grid_map::Position pos;
      map.getPosition( index, pos );
      pos_buffer.push_back( pos );
    }
  }
  // std:: cout<<"step 1\n\n\n\n";
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
                                                     geometry_msgs::Pose &pos, int index ,double &min_width)
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
  // std::cout<<"min_distacne : "<<min_distance<<"\n\n\n\n\n\n";
  if ( min_distance != MAXFLOAT && min_distance > 0.5) {
    if(min_distance-min_width>0.02){
      return false;
    }
    min_width = min_distance<min_width ? min_distance:min_width;
    // std::cout << " distance " << min_distance << "\n\n\n\n\n";
    // std::cout << "pos1: " << pos1[0] << "   " << pos1[1] << "     pos2:  " << pos2[0] << "   "
    //           << pos2[1] << "\n\n\n\n";
    mark_narrow_passage( pos1, pos2 );
    pos.position.x = 0.5 * ( pos1[0] + pos2[0] );
    pos.position.y = 0.5 * ( pos1[1] + pos2[1] );

    double theta_mid = std::atan2(pos1[1] - pos2[1], pos1[0] - pos2[0]);

    // std::cout<<"mid: "<<pos.position.x<<" "<<pos.position.y<<"   robot"<<robot_pose.position.x<<"  "<<robot_pose.position.y<<"\n\n\n";

    double dis = MAXFLOAT;
    int index_ = 0;
    if(path_msg.poses.size()<1){
      return true;
    }
    for ( int i = 0; i < path_msg.poses.size(); i++ ) {
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
    int index_2=0;
    if(index_==path_msg.poses.size()-1){
      index_2 = index_-1;
    }
    else{
      index_2 = index_ +1;
    }

    double deltaX = path_msg.poses[index_2].pose.position.x - path_msg.poses[index_].pose.position.x;
    double deltaY = path_msg.poses[index_2].pose.position.y - path_msg.poses[index_].pose.position.y;

    // 使用 atan2 计算角度
    double yaw_2 = atan2(deltaY, deltaX);

    if(std::abs(constrainAngle_mpi_pi(yaw_2-(theta_mid+M_PI/2.0))) < M_PI/2.0){
      yaw_2 = theta_mid + M_PI/2.0;
    }
    else{
      yaw_2 = theta_mid - M_PI/2.0;
    }

    tf::Quaternion quaternion_2 = tf::createQuaternionFromRPY(0.0, 0.0, yaw_2);

    pos.orientation.x = quaternion_2.x();
    pos.orientation.y = quaternion_2.y();
    pos.orientation.z = quaternion_2.z();
    pos.orientation.w = quaternion_2.w();
  
    return true;
  }


  return false;
}



void Narrowpassagedetection::mark_narrow_passage( grid_map::Position pos1, grid_map::Position pos2 )
{
  // std:: cout<<"step 2\n\n\n\n";
  int count=0;
  grid_map::GridMap elevationmap_mark = elevationmap_;
  for ( grid_map::GridMapIterator iterator( elevationmap_mark ); !iterator.isPastEnd(); ++iterator ) {
    const grid_map::Index index( *iterator );
    grid_map::Position position3;
    elevationmap_mark.getPosition( index, position3 );
    if ( isPointOnSegment( pos1, pos2, position3 ) || isPointOnSegment( pos2, pos1, position3 ) ) {
        elevationmap_mark.at("elevation", *iterator) = -0.0;
    }
  }
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage( elevationmap_mark, message );
  map_pub2.publish( message );

    // std:: cout<<"marked count"<<count<<" \n\n\n\n";

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

geometry_msgs::Pose Narrowpassagedetection::extend_point( geometry_msgs::Pose &pose, float distance,
                                                          bool extend_or_approach )
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF( pose.orientation, quat );
  double roll_, pitch_, yaw_;
  tf::Matrix3x3( quat ).getRPY( roll_, pitch_, yaw_ );


    // 使用 atan2 计算角度
  double angle = yaw_;

  // std::cout<<"yaw_ :: "<<yaw_<<"!!!!!!!!!!!!!!!!!\n\n\n\n\n";
  // if ( extend_or_approach ) {
  //   angle -= M_PI;
  // }
  //   if ( backward ) {
  //     angle -= M_PI;
  //   }
  double x ;
  double y ;
  // if(backward){
  //   x = distance * std::cos( angle );
  //   y = distance * std::sin( angle );
  //   // std::cout<<"backward!!!!!!!!!!!!!!!!!\n\n\n\n\n";
  //   yaw_ -=M_PI;
  //   tf::Quaternion quaternion;
  //   quaternion.setRPY(roll_, pitch_, yaw_);
  //   pose.orientation.x=quaternion.getX();
  //   pose.orientation.y=quaternion.getY();
  //   pose.orientation.z = quaternion.getZ();
  //   pose.orientation.w = quaternion.getW();
  // }
  // else{
    x = -distance * std::cos( angle );
    y = -distance * std::sin( angle );
  // }



  geometry_msgs::Pose pose_;
  pose_.orientation = pose.orientation;
  pose_.position.x = pose.position.x + x;
  pose_.position.y = pose.position.y + y;
  pose_.position.z = pose.position.z;
  grid_map::Position position(pose_.position.x,pose_.position.y);
  grid_map::Index id;
  occupancy_map.getIndex(position,id);
  
  if(occupancy_map.get("distance_transform")(id[0], id[1])<0.25/0.05){
    ROS_INFO("adjust point \n\n\n\n\n\n");
    grid_map::Position mid_pose_(pose.position.x, pose.position.y);
    geometry_msgs::Pose adjust_pose;
    if(adjust_point(pose_,adjust_pose, mid_pose_, distance)){
      ROS_INFO(" finish  adjust point \n\n\n\n\n\n");
      adjust_pose.orientation = pose.orientation;
      return adjust_pose;
    }
  }

  return pose_;
}
void Narrowpassagedetection::touchDistanceField( const grid_map::Matrix &dist_trans_map,
                                                 const grid_map::Index &current_point,
                                                 const int idx_x, const int idx_y,
                                                 float &highest_val, grid_map::Index &highest_index, grid_map::Position mid_pose, grid_map::Position start_pose ,double dis )
{
  if (dist_trans_map(idx_x, idx_y) == std::numeric_limits<float>::max())
        return;

      float this_delta = dist_trans_map(idx_x, idx_y) - dist_trans_map(current_point(0),current_point(1));
      grid_map::Position pose;
      grid_map::Index id(idx_x,idx_y);
      occupancy_map.getPosition(id,pose);
      float distance_to_mid = std::sqrt(std::pow(pose[0]-mid_pose[0],2)+ std::pow(pose[1]-mid_pose[1],2));
      float distance_to_start = std::sqrt(std::pow(pose[0]-start_pose[0],2)+ std::pow(pose[1]-start_pose[1],2));
      double dis_delta = distance_to_mid-dis;
      if ( (this_delta > 0.0f) && (this_delta > highest_val) && (dis_delta>0.0)&&(dis_delta<0.2)&& distance_to_start<0.22){
        highest_val = this_delta;
        highest_index = grid_map::Index(idx_x, idx_y);
      }
}

bool Narrowpassagedetection::adjust_point( geometry_msgs::Pose &start_pose ,geometry_msgs::Pose &adjusted_pose, grid_map::Position mid_pose, double dis )
{
  if(getmap){
    const grid_map::Matrix& dist_data = occupancy_map["distance_transform"];
  grid_map::Index current_index;
  grid_map::Index next_index;
  grid_map::Position start_pose_(start_pose.position.x, start_pose.position.y);
  double dist_from_obstacle = 0.0;
  occupancy_map.getIndex(start_pose_, current_index);

  std::vector <grid_map::Index> path_indices;
  path_indices.push_back(current_index);

  float dist_from_start = 0.0f;

  bool abort =false;
  while(!abort)
  {
    float highest_cost = 0.0f;
    touchDistanceField(dist_data,
                            current_index,
                            current_index(0)-1,
                            current_index(1),
                            highest_cost,
                            next_index, mid_pose,start_pose_,dis );

    touchDistanceField(dist_data,
                            current_index,
                            current_index(0),
                            current_index(1)-1,
                            highest_cost,
                            next_index, mid_pose,start_pose_,dis);

    touchDistanceField(dist_data,
                            current_index,
                            current_index(0),
                            current_index(1)+1,
                            highest_cost,
                            next_index, mid_pose, start_pose_,dis);

    touchDistanceField(dist_data,
                            current_index,
                            current_index(0)+1,
                            current_index(1),
                            highest_cost,
                            next_index, mid_pose, start_pose_,dis);

    touchDistanceField(dist_data,
                            current_index,
                            current_index(0)-1,
                            current_index(1)-1,
                            highest_cost,
                            next_index, mid_pose, start_pose_,dis);

    touchDistanceField(dist_data,
                            current_index,
                            current_index(0)-1,
                            current_index(1)+1,
                            highest_cost,
                            next_index, mid_pose, start_pose_,dis);

    touchDistanceField(dist_data,
                            current_index,
                            current_index(0)+1,
                            current_index(1)-1,
                            highest_cost,
                            next_index, mid_pose, start_pose_,dis);

    touchDistanceField(dist_data,
                            current_index,
                            current_index(0)+1,
                            current_index(1)+1,
                            highest_cost,
                            next_index, mid_pose,start_pose_,dis);


    dist_from_obstacle = dist_data(current_index(0), current_index(1));

    //std::cout << "curr_index: " << current_index << "\ndist_from_obstacle: " << dist_from_obstacle << "dist_from_start: " << dist_from_start << " highest cost: " << highest_cost << "\n";

    // If gradient could not be followed..
    if (highest_cost == 0.0f){
      if (dist_from_obstacle < 0.25/0.05){
        ROS_WARN("Could not find gradient of distance transform leading to free area, returning original pose");
        adjusted_pose = start_pose;
        return false;

      // Could not reach desired distance from obstacles, but enough clearance
      }else{
        ROS_INFO("Reached final distance");
        abort = true;
      }

    // Gradient following worked
    }else{

      if (dist_from_obstacle > 0.25/0.05){
        abort = true;

      // Otherwise continue gradient following
      }else{
        current_index = next_index;
        dist_from_start += highest_cost;
        path_indices.push_back(current_index);
      }
    }
  }

  grid_map::Position adjust_position;
  occupancy_map.getPosition(current_index,adjust_position);
  adjusted_pose.position.x = adjust_position[0];
  adjusted_pose.position.y = adjust_position[1];
  return true;
  }
  return false;
  
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
  msg.head.frame_id= "world";
  msg.head.stamp = ros::Time::now();
  msg.midpose = mid_pos;
  msg.endpose = end_pos;
  msg.extendpose = extend_pos;
  extend_point_pub.publish( msg );

  narrow_passage_detection_msgs::NarrowPassageDetection msg_2;
  msg_2.narrow_passage_detected = true;
  narrow_passage_detected_pub.publish(msg_2);

}

void Narrowpassagedetection::extend_point_publisher( geometry_msgs::Pose mid_pos,
                                                     geometry_msgs::Pose end_pos )
{
  narrow_passage_detection_msgs::NarrowPassage msg;
  msg.head.frame_id= "world";
  msg.head.stamp = ros::Time::now();
  msg.midpose = mid_pos;
  msg.endpose = end_pos;
  extend_point_pub.publish( msg );

  narrow_passage_detection_msgs::NarrowPassageDetection msg_2;
  msg_2.narrow_passage_detected = true;
  narrow_passage_detected_pub.publish(msg_2);
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