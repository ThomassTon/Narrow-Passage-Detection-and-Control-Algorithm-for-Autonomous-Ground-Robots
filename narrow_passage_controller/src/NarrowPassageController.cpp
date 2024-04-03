#include <NarrowPassageController.hpp>

namespace narrow_passgae_controller
{
NarrowPassageController::NarrowPassageController( ros::NodeHandle &nodeHandle ) : nh( nodeHandle )
{

  nh.setCallbackQueue( &queue_1 );
  narrow_passage_sub = nh.subscribe( "/narrow_passage_approach", 1,
                                     &NarrowPassageController::narrow_passage_messageCallback, this );
  speed_pub = nh.advertise<std_msgs::Float32>( "/speed", 1 );
  lqr_params_pub =
      nh.advertise<narrow_passage_detection_msgs::NarrowPassageController>( "/lqr_params_narrow", 1 );
  // map_sub = nh.subscribe("/elevation_mapping/elevation_map_raw",1, &Narrowpassagedetection::map_messageCallback,this);
  // extend_point_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base/narrow_goal",1);
  nh.setCallbackQueue( &queue_2 );
  stateSubscriber = nh.subscribe( "/odom", 1, &NarrowPassageController::stateCallback, this,
                                  ros::TransportHints().tcpNoDelay( true ) );
}

void NarrowPassageController::narrow_passage_messageCallback(
    const narrow_passage_detection_msgs::NarrowPassage msg )
{
  speed_publisher( msg );
  lqr_params_publisher( msg );
}
void NarrowPassageController::speed_publisher( const narrow_passage_detection_msgs::NarrowPassage msg )
{
  float speed = 0.3;
  float distance_ = msg.approachdistance;
  if ( distance_ < 1.5 ) {
    speed = 0.1;
  }

  std_msgs::Float32 speed_msg;
  speed_msg.data = speed;
  speed_pub.publish(speed_msg);
}

void NarrowPassageController::lqr_params_publisher(
    const narrow_passage_detection_msgs::NarrowPassage msg )
{
  narrow_passage_detection_msgs::NarrowPassageController lqr_params_msg;
  lqr_params_msg.q_11 = 40;
  lqr_params_msg.q_22 = 10;
  lqr_params_msg.lookahead_distance =1.8;
//  if(msg.approachdistance<0.7){
//    lqr_params_msg.lookahead_distance = 0.5;
//    lqr_params_msg.q_22 = 2;
//    lqr_params_msg.q_11 = 30;
//  }
  if(msg.approachdistance<1.5){
    lqr_params_msg.lookahead_distance = 0.3;
//    lqr_params_msg.q_11 = 30;
    lqr_params_msg.q_22 = 5;
  }
  if(msg.approachdistance<2.0){
    lqr_params_msg.lookahead_distance = 1.8;
  }
//
//
//  if ( msg.approachdistance < 2.0 ) {
//    lqr_params_msg.lookahead_distance = 1.8;
//  }
//  if(msg.approachdistance <2.0){
//    lqr_params_msg.lookahead_distance = msg.approachdistance;
//    lqr_params_msg.q_11 = 30-5*msg.approachdistance;
//    lqr_params_msg.q_22 = 10 - 3*msg.approachdistance;
//  }
//  if(lqr_params_msg.lookahead_distance<0.2){
//    lqr_params_msg.lookahead_distance = 0.2;
//  }
  lqr_params_pub.publish( lqr_params_msg );
}

void NarrowPassageController::stateCallback( const nav_msgs::Odometry odom_state )
{
  updateRobotState( odom_state );
  float pos_x, pos_y, theta;
  predicteRobotState(pos_x, pos_y, theta);

}

void NarrowPassageController::updateRobotState( const nav_msgs::Odometry odom_state )
{
  dt = ( odom_state.header.stamp - latest_odom_.header.stamp ).toSec();

  if ( dt < 0.0 || dt > 0.2 ) {
    ROS_INFO( "[vehicle_controller] dt between old robot state and new is %f seconds, setting "
              "to 0.0 for this iteration",
              dt );
    dt = 0.0;
  }

  pose.header = odom_state.header;
  pose.pose = odom_state.pose.pose;
  velocity_linear.header = odom_state.header;
  velocity_linear.vector = odom_state.twist.twist.linear;
  velocity_angular.header = odom_state.header;
  velocity_angular.vector = odom_state.twist.twist.angular;
  latest_odom_ = odom_state;
  std::cout<<"x_:  "<<pose.pose.position.x<<"   y_: "<<pose.pose.position.y <<"\n";
}

void NarrowPassageController::predicteRobotState( float &x, float &y, float &theta )
{
  double roll, pitch, yaw;
  tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  theta = velocity_angular.vector.z*dt/2.0 + yaw;
  x = pose.pose.position.x + cos(theta) *dt * velocity_linear.vector.x;
  y = pose.pose.position.y + sin(theta) *dt * velocity_linear.vector.x;
  std::cout<<"x:  "<<x<<"   y: "<<y <<"\n";
}



} // namespace narrow_passgae_controller
