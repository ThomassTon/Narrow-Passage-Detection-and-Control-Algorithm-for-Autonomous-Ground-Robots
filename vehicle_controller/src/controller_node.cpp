#include <vehicle_controller/controller_node.h>

ControllerNode::ControllerNode(const ros::NodeHandle &nh)
: nh_(nh)
{
  controller_type_reconfigure_server_ = std::make_shared<dynamic_reconfigure::Server<vehicle_controller::ControllerTypeConfig>>();
  controller_type_reconfigure_server_->setCallback(boost::bind(&ControllerNode::controllerTypeCallback, this, _1, _2));
  controllerTypeSwitch = nh_.subscribe( "/narrow_passage_detected", 1, &ControllerNode::controllerTypeSwitchCallback, this );
  reset();
}

void ControllerNode::reset(){
  if(controller_type_ == "daf"){
    controller_ = std::make_shared<Daf_Controller>(nh_);
  }
  else if(controller_type_ == "ackermann_pure_pursuit"){
    controller_ = std::make_shared<Ackermann_Pure_Pursuit_Controller>(nh_);
  }
  else if(controller_type_ == "differential_pure_pursuit"){
    controller_ = std::make_shared<Differential_Pure_Pursuit_Controller>(nh_);
  }
  else if (controller_type_ == "lqr"){
    controller_ = std::make_shared<Lqr_Controller>(nh_); //Lqr_Controller
  }
  else if (controller_type_ == "mpc"){
    controller_ = std::make_shared<MPC_Controller>(nh_);
  }
  else{
    controller_ = std::make_shared<Carrot_Controller>(nh_);
  }
  controller_->configure();
  ROS_INFO_STREAM("Vehicle Controller Type is: " << controller_->getName()<<"\n\n\n\n\n\n\n\n");
}

void ControllerNode::controllerTypeCallback(vehicle_controller::ControllerTypeConfig &config, uint32_t level) {

  if(config.controller_type == DAF){
    controller_type_ = "daf";
  }
  else if(config.controller_type == ACKERM_PP){
    controller_type_ = "ackermann_pure_pursuit";
  }
  else if(config.controller_type == DIFF_PP){
    controller_type_ = "differential_pure_pursuit";
  }
  else if (config.controller_type == LQR){
    controller_type_ = "lqr";
  }
  else if (config.controller_type == MPC){
    controller_type_ = "mpc";
  }
  else{
    controller_type_ = "carrot";
  }
  _controller_type_ = controller_type_;
  ROS_INFO_STREAM("controlstype: "<<controller_type_<<"\n\n\n\n\n\n\n\n\n");
  reset();
}

void ControllerNode::controllerTypeSwitchCallback(const narrow_passage_detection_msgs::NarrowPassageDetection &msg){
  if(msg.narrow_passage_detected){

    controller_type_ = "mpc";
    reset();
    // controller_->followPathSet();
    ROS_INFO_STREAM("controllerType switch  \n\n\n\n\n\n\n");
  }
  else{
    controller_type_ = _controller_type_;
    reset();
    ROS_INFO_STREAM("controllerType switch back \n\n\n\n\n\n\n");
  }
    // ROS_INFO_STREAM("controlstype: "<<controller_type_<<"\n\n\n\n\n\n\n\n\n");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  ros::NodeHandle nh;

  ControllerNode cn(nh);


  while(ros::ok())
  {
    ros::spin();
  }

  ros::shutdown();
  return 0;
}
