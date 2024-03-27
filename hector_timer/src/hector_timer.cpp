#include "hector_timer.hpp"

namespace hector_timer
{
hector_timer_msgs::HectorTimer time_info;
HectorTimer::HectorTimer( ros::NodeHandle &nodeHandle ) : nh( nodeHandle ) {
  maxduration.fromSec(0.01);
  nh.setCallbackQueue(&queue_1);
  line_info_sub = nh.subscribe("/position_line_detection",1, &HectorTimer::line_info_messageCallback,this);

  nh.setCallbackQueue(&queue_2);
  timer_publisher =  nh.advertise<hector_timer_msgs::HectorTimer>("/hector_timer", 1);
  timer_pub_updater = nh.createTimer(maxduration, &HectorTimer::timer_pub, this, false, false);
}
void HectorTimer::line_info_messageCallback( const hector_timer_msgs::HectorTimerLineInfo &msg )
{
  if( line_decide(line_map,msg.lineid)){
    if(msg.linetype.value == msg.linetype.Start)
    {
      total_count += 1;
      start_time = ros::Time::now();
      time_info.lineid = msg.lineid;
      time_info.total_line_count = total_count;
      fisrt_line_info = msg;
      last_line_info = msg;
      timer_pub_updater.start();

    }
    else if(msg.linetype.value == msg.linetype.Middle){
      total_count +=1;
      ros::Duration sector_time_ = msg.header.stamp - last_line_info.header.stamp;
      time_info.sector_time = sector_time_.toSec();
      time_info.lineid = msg.lineid;
      time_info.total_line_count = total_count;
      ros::Duration total_time_ = msg.header.stamp - fisrt_line_info.header.stamp;
      last_line_info = msg;
    }
    else if(msg.linetype.value == msg.linetype.End)
    {
      total_count +=1;
      timer_pub_updater.stop();
      ros::Duration sector_time_ = msg.header.stamp - last_line_info.header.stamp;
      time_info.sector_time = sector_time_.toSec();
      ros::Duration total_time_ =end_time -  fisrt_line_info.header.stamp;
      time_info.total_time = total_time_.toSec();
      time_info.lineid = msg.lineid;
      time_info.total_line_count = total_count;
      timer_publisher.publish(time_info);
      total_count = 0;
    }
  }

//  std::cout<<"test:  "<<msg.linetype<<"\n\n\n\n\n\n\n\n\n";
}
void HectorTimer::timer_pub( const ros::TimerEvent&)
{
  end_time = ros::Time::now();
  ros::Duration total_time_ =end_time-  fisrt_line_info.header.stamp;
  time_info.total_time = total_time_.toSec();
  ros::Duration duration = end_time - start_time;
  float duration_pub = duration.toSec();
//  msg.data = "test";
  timer_publisher.publish(time_info);
}

bool HectorTimer::line_decide( std::map<int, int > &line_map ,int lineid)
{
  bool output = false;
  if (line_map.count(lineid) == 0)
  {
    output = true; // first collision detect.
  }
  line_map[lineid] = 1; // Collisions still occur
  return output;
}

} // namespace hector_timer
