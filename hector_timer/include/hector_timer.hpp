#ifndef HECTOR_TIMER_HPP
#define HECTOR_TIMER_HPP

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "iostream"
#include "std_msgs/String.h"
#include "hector_timer_msgs/HectorTimerLineInfo.h"
#include "hector_timer_msgs/HectorTimer.h"
#include "map"
namespace hector_timer{
ros::CallbackQueue queue_1;
ros::CallbackQueue queue_2;
class HectorTimer{
protected:
  std::map<int , int > line_map;
  ros::Subscriber line_info_sub;
  ros::Publisher timer_publisher;
  ros::Timer timer_pub_updater;
  ros::Duration maxduration;
  ros::Time end_time;
  ros::Time start_time;
  int total_count=0;

  hector_timer_msgs::HectorTimer time_info;
  hector_timer_msgs::HectorTimerLineInfo last_line_info;
  hector_timer_msgs::HectorTimerLineInfo fisrt_line_info;
  void line_info_messageCallback(const hector_timer_msgs::HectorTimerLineInfo& msg);
  void timer_pub(const ros::TimerEvent& eve);
  bool line_decide(std::map<int , int > &line_map, int lineid);

public:
  HectorTimer(ros::NodeHandle& nodeHandle);
  ros::NodeHandle nh;

};
}



#endif