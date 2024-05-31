#ifndef VEHICLE_CONTROLLER_CONTROLLER_NODE_H
#define VEHICLE_CONTROLLER_CONTROLLER_NODE_H

#include <vehicle_controller/carrot_controller.h>
#include <vehicle_controller/daf_controller.h>
#include <vehicle_controller/ackermann_pure_pursuit_controller.h>
#include <vehicle_controller/differential_pure_pursuit_controller.h>
#include <vehicle_controller/controller.h>

#include <vehicle_controller/lqr_controller.h>
#include <vehicle_controller/mpc.h>

#include <dynamic_reconfigure/server.h>
#include <vehicle_controller/ControllerTypeConfig.h>
#include <narrow_passage_detection_msgs/NarrowPassageDetection.h>



class ControllerNode
{
public:
  typedef enum { CARROT, DAF, ACKERM_PP, DIFF_PP , LQR, MPC} Control_Type_Enum;

  explicit ControllerNode(const ros::NodeHandle& nh);

protected:
  void reset();
  void controllerTypeCallback(vehicle_controller::ControllerTypeConfig & config, uint32_t level);
  void controllerTypeSwitchCallback(const narrow_passage_detection_msgs::NarrowPassageDetection &msg);

  ros::NodeHandle nh_;
  ros::Subscriber controllerTypeSwitch;
  std::shared_ptr<dynamic_reconfigure::Server<vehicle_controller::ControllerTypeConfig>> controller_type_reconfigure_server_;
  std::shared_ptr<Controller> controller_;
  std::string controller_type_;
  std::string _controller_type_;



  boost::shared_ptr<actionlib::ActionServer<move_base_lite_msgs::FollowPathAction> > follow_path_server_;
  actionlib::ActionServer<move_base_lite_msgs::FollowPathAction>::GoalHandle follow_path_goal_;

};

#endif // VEHICLE_CONTROLLER_CONTROLLER_NODE_H
