#include<NarrowPassageController.hpp>


namespace narrow_passgae_controller{
    NarrowPassageController::NarrowPassageController(ros::NodeHandle& nodeHandle):nh(nodeHandle){
        
        // nh.setCallbackQueue(&queue_1);
        narrow_passage_sub = nh.subscribe("/narrow_passage_approach",1, &NarrowPassageController::narrow_passage_messageCallback,this);
        speed_pub = nh.advertise<std_msgs::Float32>("/speed",1);
        lqr_params_pub  = nh.advertise<narrow_passage_detection_msgs::NarrowPassageController>("/lqr_params_narrow",1);
        // map_sub = nh.subscribe("/elevation_mapping/elevation_map_raw",1, &Narrowpassagedetection::map_messageCallback,this);
        // extend_point_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base/narrow_goal",1);
        // nh.setCallbackQueue(&queue_2);

        // path_sub = nh.subscribe("/smooth_path",1,&Narrowpassagedetection::path_messageCallback, this);

        // pose_sub = nh.subscribe("/odom",1,&Narrowpassagedetection::pose_messageCallback,this);
        // vel_sub = nh.subscribe("/cmd_vel_raw",1,&Narrowpassagedetection::vel_messageCallback,this);
        // map_sub2 = nh.subscribe("/map",1,&Narrowpassagedetection::map_messageCallback2,this);
        // map_pub = nh.advertise<grid_map_msgs::GridMap>("/narrow_passage_map", 1);
        // width_pub = nh.advertise<std_msgs::String>("/passage_width",1);
        // approach_distacne_pub = nh.advertise<narrow_passage_detection_msgs::NarrowPassage>("/narrow_passage_approach",1);

        // nh.setCallbackQueue(&queue_3);

        // path_sub = nh.subscribe("/smooth_path",1,&Narrowpassagedetection::path_messageCallback, this);

        // maxduration.fromSec(1.00);


        // mapUpdateTimer_ = nh.createTimer(maxduration, &Narrowpassagedetection::mapUpdateTimerCallback, this, false, false);
        // mapUpdateTimer_.start();
        // setupTimers();
        // initialize();
    }

    void NarrowPassageController::narrow_passage_messageCallback(const narrow_passage_detection_msgs::NarrowPassage msg){
      speed_publisher(msg);
      lqr_params_publisher(msg);
    }
    void NarrowPassageController::speed_publisher(const narrow_passage_detection_msgs::NarrowPassage msg){
        float speed = 0.3;
        float distance_ = msg.approachdistance;
//        std::cout<<"distance : "<<distance_<<"----------------------------------------\n";
        if(distance_<2.0){
          speed=0.2;
        }

        std_msgs::Float32 speed_msg;
        speed_msg.data = speed;
//        speed_pub.publish(speed_msg);
    }

    void NarrowPassageController::lqr_params_publisher( const narrow_passage_detection_msgs::NarrowPassage msg )
    {
      narrow_passage_detection_msgs::NarrowPassageController lqr_params_msg;

      lqr_params_msg.q_11 = 40;
      lqr_params_msg.q_22 = 15;
      lqr_params_msg.lookahead_distance =0.4;
      if(msg.approachdistance<0.5){
        lqr_params_msg.lookahead_distance = 0.4;
        lqr_params_msg.q_22 = 2;
      }
      if(msg.approachdistance<1.0){
        lqr_params_msg.lookahead_distance = 0.4;
        lqr_params_msg.q_22 = 2;
      }
      if(msg.approachdistance<2.0){
        lqr_params_msg.lookahead_distance = 1.8;
      }
      lqr_params_pub.publish(lqr_params_msg);
    }
    


}
