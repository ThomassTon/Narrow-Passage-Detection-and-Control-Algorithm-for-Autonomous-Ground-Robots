#include <NarrowPassageController.hpp>


namespace narrow_passgae_controller
{
NarrowPassageController::NarrowPassageController( ros::NodeHandle &nodeHandle ) : nh( nodeHandle )
{

  nh.setCallbackQueue( &queue_1 );
  narrow_passage_sub = nh.subscribe( "/approach_goal", 1, &NarrowPassageController::narrow_passage_messageCallback ,this );
  speed_pub = nh.advertise<std_msgs::Float32>( "/speed", 1 );
  smoothPathPublisher = nh.advertise<nav_msgs::Path>( "smooth_path_circle", 1, true );

  // map_sub = nh.subscribe("/map",1,&NarrowPassageController::map_messageCallback2,this);

  // map_sub = nh.subscribe("/elevation_mapping/elevation_map_raw",1, &Narrowpassagedetection::map_messageCallback,this);
  // extend_point_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base/narrow_goal",1);
  nh.setCallbackQueue( &queue_2 );
  stateSubscriber = nh.subscribe( "/odom", 1, &NarrowPassageController::stateCallback, this,
                                  ros::TransportHints().tcpNoDelay( true ) );
}

void NarrowPassageController::map_messageCallback2(const nav_msgs::OccupancyGrid& msg){
    grid_map::GridMapRosConverter::fromOccupancyGrid(msg,std::string("occupancy"), occupancy_map);   //distance_transform    occupancy
    get_map = true;
    // occupancy_map = 
    // for (grid_map::GridMapIterator iterator(occupancy_map);!iterator.isPastEnd(); ++iterator ){
    //         const grid_map::Index index(*iterator);
    //         const float value = occupancy_map.get("distance_transform")(index(0), index(1));
    //         std::cout<<value<<" ";
    //     }
}

void NarrowPassageController::narrow_passage_messageCallback(
    const narrow_passage_detection_msgs::NarrowPassage msg )
{
    path_to_approach(robot_pose, msg.endpose, msg.midpose);
}


void NarrowPassageController::stateCallback( const nav_msgs::Odometry odom_state )
{
  robot_pose = odom_state.pose.pose;
}


double NarrowPassageController::compute_distance(grid_map::Position pos1, grid_map::Position pos2){
  double x1 = pos1[0];
  double y1 = pos1[1];
  double x2 = pos2[0];
  double y2 = pos2[1];
  return std::sqrt(std::pow(x1-x2,2) + std::pow(y1 - y2, 2));
}

bool NarrowPassageController::compareByDistance(robot_range &a, robot_range &b){
  return a.distance< b.distance;
}




void NarrowPassageController::path_to_approach(geometry_msgs::Pose start , geometry_msgs::Pose end, geometry_msgs::Pose mid){
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


  R_x = (2*end_x*mid_x*end_y - std::pow(end_x,2)*mid_y + std::pow(end_y,2)*mid_y - end_y*std::pow(end_x,2) - std::pow(end_y,3))/(2*(mid_x*end_y - mid_y*end_x));
  R_y = (2*end_x*mid_y*end_y - std::pow(end_y,2)*mid_x + std::pow(end_x,2)*mid_x - end_x*std::pow(end_y,2) - std::pow(end_x,3))/(2*(- mid_x*end_y + mid_y*end_x));
  r = std::sqrt(std::pow(R_x,2) + std::pow(R_y,2));
  // angle_pi = M_PI - 2 * (std::atan2(end_y, end_x) - std::atan2(R_y, R_x));
  // s = r * angle_pi;
  // yaw = M_PI/2 + std::atan2(R_y, R_x);

  R_x+= start.position.x;
  R_y+=start.position.y;

  std::cout<<"start_x: "<<start.position.x<<"    start_y: "<<start.position.y<<"\n";
  std::cout<<"end_x: "<<end.position.x<<"    end_y: "<<end.position.y<<"\n";
  std::cout<<"mid_x: "<<mid.position.x<<"    mid_y: "<<mid.position.y<<"\n";
  std::cout<<"R_X: "<<R_x<<"    R_y: "<<R_y<<"\n";
  double roll, pitch, yaw;
  // tf::Quaternion q( end.orientation.x, end.orientation.y, end.orientation.z, end.orientation.w );
  // tf::Matrix3x3 m( q );
  // m.getRPY( roll, pitch, yaw );
  // double x_ = end.position.x + 0.05*cos(yaw-M_PI);
  // double y_ = end.position.y + 0.05*sin(yaw-M_PI);

  double angle_ = std::atan2(start.position.y- R_y, start.position.x-R_x);
  double angle_2 = std::atan2(end.position.y - R_y, end.position.x - R_x);

  double angle_diff = constrainAngle_mpi_pi(angle_2 - angle_);
  nav_msgs::Path circle;
  double angle_diff_abs = std::abs(angle_diff) /M_PI*180;
  for(int i=0; i<angle_diff_abs; i++){
    double angle_plus =  i/angle_diff_abs * angle_diff;
    angle_plus += angle_;
    double x = R_x + cos(angle_plus)*r;
    double y = R_y + sin(angle_plus)*r;
    geometry_msgs::PoseStamped waypoint;
    waypoint.pose.position.x = x;
    waypoint.pose.position.y = y;
    waypoint.pose.position.z = 0;
    // double roll = 0;  // 45度
    // double pitch = 0; // 30度
    // double yaw = angle_plus;   // 60度

    // tf2::Quaternion quat;
    // quat.setRPY(roll, pitch, yaw);
    circle.poses.push_back(waypoint);
  }
    geometry_msgs::PoseStamped waypoint;
    waypoint.pose = end;
    circle.poses.push_back(waypoint);
  for(int i = 0; i< circle.poses.size()-1;i++)
  {
    double angle = std::atan2(circle.poses[i+1].pose.position.y - circle.poses[i].pose.position.y, circle.poses[i+1].pose.position.x - circle.poses[i].pose.position.x);
        double roll = 0;  
    double pitch = 0; 
    double yaw = angle;  

    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);

    circle.poses[i].pose.orientation.x = quat.getX();
    circle.poses[i].pose.orientation.y = quat.getY();
    circle.poses[i].pose.orientation.z = quat.getZ();
    circle.poses[i].pose.orientation.w = quat.getW();


  }

  circle.header.frame_id = "world";
  circle.header.stamp = ros::Time::now();
  smoothPathPublisher.publish(circle);

}



} // namespace narrow_passgae_controller
