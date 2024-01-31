#include"Narrowpassagedetection.hpp"
#include <unistd.h>


namespace narrow_passage_detection{

    bool show=false;

    Narrowpassagedetection::Narrowpassagedetection(ros::NodeHandle& nodeHandle):nh(nodeHandle){
        
        nh.setCallbackQueue(&queue_1);
        map_sub = nh.subscribe("/elevation_mapping/elevation_map_raw",1, &Narrowpassagedetection::map_messageCallback,this);
        nh.setCallbackQueue(&queue_2);

        pose_sub = nh.subscribe("/odom",1,&Narrowpassagedetection::pose_messageCallback,this);
        vel_pub = nh.subscribe("/cmd_vel_raw",1,&Narrowpassagedetection::vel_messageCallback,this);

        map_pub = nh.advertise<grid_map_msgs::GridMap>("/narrow_passage_map", 1);
        width_pub = nh.advertise<std_msgs::String>("/passage_width",1);
        // maxduration.fromSec(1.00);


        // mapUpdateTimer_ = nh.createTimer(maxduration, &Narrowpassagedetection::mapUpdateTimerCallback, this, false, false);
        // mapUpdateTimer_.start();
        // setupTimers();
        // initialize();
    }

    void Narrowpassagedetection::map_messageCallback(const grid_map_msgs::GridMap& msg)
    {

    // ROS_INFO("Received message: \n\n\n\n\n\n\n\n\n\n\n\n");
        grid_map::GridMapRosConverter::fromMessage(msg, elevationmap);
        outputmap = elevationmap;
       for(auto & data: elevationmap.getLayers()){
        std::cout<<data<<"   ";
       }
        grid_data = outputmap["elevation"];
        outputmap.erase("elevation");
        outputmap.erase("upper_bound");
        outputmap.erase("lower_bound");
        
        getmap=true;
        show = false;
        bool result = grid_map::GridMapCvConverter::toImage<unsigned char, 1>(elevationmap, "elevation", CV_8UC1, input_img);
        if(result){
            narrowmap_pub();
        }


    }

    void Narrowpassagedetection::pose_messageCallback(const nav_msgs::Odometry &pose)
    {
        pose_msg = pose;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(pose_msg.pose.pose.orientation, quat);
         
        tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);

        // std::cout<<"yaw :"<<yaw<<std::endl;
    }

    void Narrowpassagedetection::vel_messageCallback(const geometry_msgs::Twist& msg){
        vel_msg = msg;
        if(vel_msg.linear.x<-0.0002){
            backward = true;
        }
        if(vel_msg.linear.x>0.0002){
            backward = false;
        }
    }

    void Narrowpassagedetection::setupTimers(){
        mapUpdateTimer_ = nh.createTimer(maxduration, &Narrowpassagedetection::mapUpdateTimerCallback, this, false, false);
    }
    void Narrowpassagedetection::mapUpdateTimerCallback(const ros::TimerEvent&){
        // ROS_INFO("publish test publish test\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
        if(getmap==true&&!show){
            // const grid_map::Matrix& grid_data (elevationmap["elevation"]);

            bool result = grid_map::GridMapCvConverter::toImage<unsigned char, 1>(elevationmap, "elevation", CV_8UC1, input_img);
            if(result){
                narrowmap_pub();
            }
            getmap = false;
        }
    }
    void Narrowpassagedetection::initialize(){

        mapUpdateTimer_.start();

    }

    void Narrowpassagedetection::narrowmap_pub(){
        ROS_INFO("publish test publish test\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
        // cv::imshow("test",input_img);
        // cv::imwrite("/home/haolei/Documents/test.jpg",input_img);
        ros::Time start_time = ros::Time::now();
        if(generate_output())
        {   
            ros::Time end_time = ros::Time::now();
            ros::Duration duration = end_time - start_time;

    // 输出时间差
            ROS_INFO("Time elapsed: %.3f seconds", duration.toSec());            
            std::cout<<"publish success"<<"\n\n\n\n\n\n\n\n\n"<<std::endl;
            grid_map_msgs::GridMap message;
            grid_map::GridMapRosConverter::toMessage(outputmap, message);
            map_pub.publish(message);
        }
        }

    bool Narrowpassagedetection::generate_output(){
        // std::cout<<"size of inputimg:  "<<outputmap.getSize()<<"\n\n\n\n\n\n\n\n\n"<<std::endl;
        computegradient();


        // bool result = addLayerFromImage<unsigned char, 1>(input_img, "elevation", outputmap);
        outputmap.add("elevation",grid_data);
        create_ray();
        compute_passage_width();
        return true;
    }

    void Narrowpassagedetection::computegradient(){
        int len1= input_img.rows;
        int len2= input_img.cols;
        cv::imwrite("/home/yuan/Documents/input.jpg",input_img);

        cv::Mat gradientX(len1,len2,CV_8UC1), gradientY(len1,len2,CV_8UC1),magnitude(len1,len2,CV_8UC1), gaussian_img(len1,len2,CV_8UC1);
        // cv::GaussianBlur(input_img, input_img, cv::Size(3, 3), 0);
        
        cv::Mat kernal_mat2 = (cv::Mat_<float>(3, 3) <<
		1 / 16.0f, 2 / 16.0f, 1 / 16.0f,
		2 / 16.0f, 4 / 16.0f, 2 / 16.0f,
		1 / 16.0f, 2 / 16.0f, 1 / 16.0f);
        cv::filter2D(input_img,gaussian_img,-1,kernal_mat2);


        cv::Mat kernal_mat1 = (cv::Mat_<char>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
        cv::filter2D(gaussian_img,gaussian_img,-1,kernal_mat1);


        cv::Sobel(gaussian_img, gradientX, CV_32F, 1, 0, 3);
        cv::Sobel(gaussian_img, gradientY, CV_32F, 0, 1, 3);
        cv::cartToPolar(gradientX, gradientY, gradient, direction, true);
        std::cout<<"compute "<<"\n\n\n\n\n\n\n\n\n"<<std::endl;
        gradient.convertTo(gradient,CV_8UC1);




        for(int i=0;i<gradient.rows;i++)  
        {  
            for(int j=0;j<gradient.cols;j++)  
            {  
                //magnitude.at<Vec3b>(i,j)[0]=magnitude.at<Vec3b>(i,j)[0]/div*div+div/2;  
                // std::cout<<magnitude.at<uchar>(i,j)<<std::endl;
                if(gradient.at<uchar>(i,j)< uchar(255))
                {
                    gradient.at<uchar>(i,j)=uchar(0);
                }
                else{
                    gradient.at<uchar>(i,j) = uchar(255);
                }
            }  
        }
        convert_from_gradient();


    }

    void Narrowpassagedetection::convert_from_gradient(){
        for (grid_map::GridMapIterator iterator(elevationmap);!iterator.isPastEnd(); ++iterator ){
            const grid_map::Index index(*iterator);
            const float& value = grid_data(index(0), index(1));
            const grid_map::Index imageIndex(iterator.getUnwrappedIndex());
            const float maxValue = elevationmap.get("elevation").maxCoeffOfFinites();
            if(gradient.at<uchar>(imageIndex(0),imageIndex(1))== uchar(0)&&std::isfinite(value)&&value<0.7*maxValue){
                grid_data(index(0), index(1)) = NAN;
            }          
            // else if(grid_data(index(0), index(1))> (maxValue*0.5)){
            //     grid_data(index(0), index(1)) = maxValue;
            // }
            // else{
            //     grid_data(index(0), index(1)) = NAN;
            // }
        }
    }
    

    void Narrowpassagedetection::create_ray(){
        grid_map::Position position;
        grid_map::Index robot;
        ray_buffer.clear();
        test_buffer.clear();
        if(outputmap.getIndex(grid_map::Position(pose_msg.pose.pose.position.x,pose_msg.pose.pose.position.y),robot)){
            outputmap.getPosition(robot,position);
        }
        double angle = 0.0;



        for(angle=-45.0; angle<45.0;){
            // double k = std::tan(angle/180.00*M_PI+yaw);
            // double b = position[1]-k*position[0];
            // tan90 = false;
            // if(k>70){
            //     tan90=true;
            //     b = position[0];
            // }
            double angleRadians;
            if(backward){
                angleRadians = angle/180.00*M_PI + yaw - M_PI;
            }
            else{
                angleRadians = angle/180.00*M_PI+yaw;
            }
            
            double x = 3.0 * std::cos(angleRadians);
            double y = 3.0 * std::sin(angleRadians);
            // std::cout<<"k:  "<<k<< "  b:   "<<b<<std::endl;
            ray_detection(x,y,angle,position);
            angle +=0.05;
        }


        // if (outputfile1.is_open()){
        //     for (const auto& value : test_buffer)
        //     {
        //         outputfile1<<"angle:  "<<value.angle<<"    diff:  " <<value.distance <<"  distance: "<<value.index[0] <<"   index: "<< value.index[1]<<"   "<<value.index[1]<< "    positon:  "<<value.position[0]<<"   "<<value.position[1]<<"\n"; 
        //     }
        //     outputfile1.close();
        // }

        if(!ray_buffer.empty())
        {
            std::ofstream outputfile3("/home/haolei/Documents/ray_detection.txt");

        if (outputfile3.is_open()){
            for (const auto& value : ray_buffer)
            {
                outputfile3<<"angle:  " <<value.angle <<"  distance: "<<value.distance <<"   index: "<< value.index[0]<<"   "<<value.index[1]<< "    positon:  "<<value.position[0]<<"   "<<value.position[1]<<"\n"; 
            }
            outputfile3.close();
        }

        }


    }
    bool Narrowpassagedetection::compute_angle_diff(double a1,double a2){
        if(a1<0){
            a1 += 2*M_PI;
        }
        if(a2<0){
            a2 += 2*M_PI;
        }
        if(std::abs(a1-a2)<M_PI/2){
            return true;
        }
        return false;
    }
    void Narrowpassagedetection::ray_detection(double x, double y,double angle, grid_map::Position robot_position){
        dis_buffer.clear();
        for(grid_map::GridMapIterator iterator(outputmap); !iterator.isPastEnd(); ++iterator)
        {
            const grid_map::Index index(*iterator);
            const float& value = outputmap.get("elevation")(index(0), index(1));
            if(std::isfinite(value)){
                grid_map::Position position;
                outputmap.getPosition(index,position);
                grid_map::Position position1(position[0]-robot_position[0],position[1]-robot_position[1]);
                grid_map::Position position2(x,y);
                // double diff = isPointOnSegment(position1,position2,length);
                if(isPointOnSegment(position1,position2))
                {
                    double dis = calculateDistance(position,robot_position);
                    dis_buffer.push_back({dis, index,position});
                }

            }
        }

        if(!dis_buffer.empty())
        {
            
            std::sort(dis_buffer.begin(),dis_buffer.end(), Narrowpassagedetection::compareByDis);
            grid_map::Index index = dis_buffer[0].index;
            int index1 = index[0];
            int index2 = index[1];
            auto result = std::find_if(ray_buffer.begin(), ray_buffer.end(), 
                [index1, index2](const auto& element) {
                    return element.index[0]==index1 && element.index[1]==index2;
                }
            );
            if(result == ray_buffer.end())
            {
                ray_buffer.push_back({angle,dis_buffer[0].distance,dis_buffer[0].index,dis_buffer[0].position});
            }



        }
    }

    double Narrowpassagedetection::calculateDistance(const grid_map::Position& A, const grid_map::Position& B){
        return std::sqrt(std::pow(B[0] - A[0], 2) + std::pow(B[1] - A[1], 2));
    }

    bool Narrowpassagedetection::compareByDis(const dis_buffer_type& a, const dis_buffer_type& b) {
            return a.distance < b.distance;
        }
    bool Narrowpassagedetection::compareByWidth(const passage_width_buffer_type&a ,const passage_width_buffer_type&b){
        return a.wide < b.wide;
    }
    
    void Narrowpassagedetection::compute_passage_width(){
        std::vector <ray_buffer_type> buffer1;
        std::vector <ray_buffer_type> buffer2;
        std::vector <passage_width_buffer_type> width_buffer;
        double width;
        bool detected = false;
        for(int i = 1; i<ray_buffer.size();i++){

            if((std::abs(ray_buffer[i-1].angle-ray_buffer[i].angle)>6.0)){
                buffer1.clear();
                buffer2.clear();
                for(int j = 0;j<i;j++)
                {
                    buffer1.push_back(ray_buffer[j]);
                }
                for(int z = i;z<ray_buffer.size();z++ ){
                    buffer2.push_back(ray_buffer[z]);
                }
                width_buffer.clear();
                for(int i=0;i<buffer1.size();i++)
                {
                    for(int j=0; j<buffer2.size();j++)
                    {
                        const double distance = calculateDistance(buffer1[i].position,buffer2[j].position);
                        width_buffer.push_back({distance,buffer1[i].index,buffer2[j].index, buffer1[i].position, buffer2[j].position});
                    }
                }

                std::sort(width_buffer.begin(),width_buffer.end(), Narrowpassagedetection::compareByWidth);
                width = width_buffer[0].wide;
                if(width>0.3500){
                    detected = true;
                    break;
                }                
            }
        }

        if(!detected){
            width=0;
            ROS_INFO("CLASSIFICATION");
            buffer1.clear();
            buffer2.clear();
            classification(buffer1, buffer2, ray_buffer);

            width_buffer.clear();
            if(buffer1.size()==0||buffer2.size()==0){
                return;
            }
            for(int i=0;i<buffer1.size();i++)
            {
                for(int j=0; j<buffer2.size();j++)
                {
                    const double distance = calculateDistance(buffer1[i].position,buffer2[j].position);
                    width_buffer.push_back({distance,buffer1[i].index,buffer2[j].index, buffer1[i].position, buffer2[j].position});
                }
            }

            std::sort(width_buffer.begin(),width_buffer.end(), Narrowpassagedetection::compareByWidth);
            width = width_buffer[0].wide;

            // std::ofstream outputfile4("/home/haolei/Documents/buffer1.txt");
            // if (outputfile4.is_open()){
            //     for (const auto& value : buffer1)
            //     {
            //         outputfile4<<"  index1 : "<<value.index[0]<<"  "<<value.index[1] <<"   position: "<< value.position[0]<<"   "<<value.position[1]<< "\n"; 
            //     }
            //     outputfile4.close();
            //     ROS_INFO("save tay success\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
            // }

            // std::ofstream outputfile5("/home/haolei/Documents/buffer2.txt");
            // if (outputfile5.is_open()){
            //     for (const auto& value : buffer2)
            //     {
            //         outputfile5<<"  index1 : "<<value.index[0]<<"  "<<value.index[1] <<"   position: "<< value.position[0]<<"   "<<value.position[1]<< "\n"; 
            //     }
            //     outputfile5.close();
            //     ROS_INFO("save tay success\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
            // }


        }
        
        std::ofstream outputfile6("/home/haolei/Documents/width_buffer.txt");
            if (outputfile6.is_open()){
                for (const auto& value : width_buffer)
                {
                    outputfile6<<"width::  "<<value.wide<<"  index1 : "<<value.index1[0]<<"  "<<value.index1[1] <<"   index2: "<< value.index2[0]<<"   "<<value.index2[1]<< "\n"; 
                }
                outputfile6.close();
                ROS_INFO("save tay success\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
            }


       
        
        // publish width
        std_msgs::String width_msg;
        width_msg.data= std::to_string(width);
        width_pub.publish(width_msg);

        if(width<0.470&& width>0.350){
            mark_narrow_passage(width_buffer[0]);
        }
    }

    void Narrowpassagedetection::mark_narrow_passage(const passage_width_buffer_type& a){
        const grid_map::Position position1 = a.position1;
        const grid_map::Position position2 = a.position2;

        for(grid_map::GridMapIterator iterator(outputmap); !iterator.isPastEnd(); ++iterator)
        {
            const grid_map::Index index(*iterator);
            grid_map::Position position3;
            outputmap.getPosition(index,position3);
            if(isPointOnSegment(position1,position2,position3)||isPointOnSegment(position2,position1,position3)){
                outputmap["elevation"](index[0],index[1]) = 0;
            }
        }


    }

    bool Narrowpassagedetection::isPointOnSegment(const grid_map::Position A, const grid_map::Position B, const grid_map::Position C)
    {
        double vectorAB_x = B[0] - A[0];
        double vectorAB_y = B[1] - A[1];
        double vectorAC_x = C[0] - A[0];
        double vectorAC_y = C[1] - A[1];

        double dotProduct = vectorAB_x * vectorAC_x + vectorAB_y * vectorAC_y;

        double lengthAB = calculateDistance(A, B);

        double lengthAC = calculateDistance(A, C);

        // 判断点 C 是否在线段 AB 上
        return ((dotProduct/(lengthAB*lengthAC) > 0.95)&& lengthAC<lengthAB && lengthAC!=0.0);
    }


    bool Narrowpassagedetection::isPointOnSegment(const grid_map::Position A, const grid_map::Position B)
    {
        double vectorOA_x = A[0] - 0;
        double vectorOA_y = A[1] - 0;
        double vectorOB_x = B[0] - 0;
        double vectorOB_y = B[1] - 0;

        double dotProduct = vectorOA_x * vectorOB_x + vectorOA_y * vectorOB_y;

        grid_map::Position O(0,0);
        double lengthOB = calculateDistance(O, B);
        double lenghtOA = calculateDistance(O,A);
        // 判断点 C 是否在线段 AB 上
        return (dotProduct/(lenghtOA*lengthOB)>0.99999);
    }

    void Narrowpassagedetection::classification(std::vector<ray_buffer_type> &buffer1, std::vector<ray_buffer_type> &buffer2, const std::vector<ray_buffer_type> &data_)
    {
        std::vector<ray_buffer_type> data_buffer = data_;
        std::sort(data_buffer.begin(),data_buffer.end(),Narrowpassagedetection::compareByPose);
        buffer1.push_back(data_buffer[0]);
        int size= data_buffer.size();
        for(int i=0; i<size;i++){
            for (int j=0; j<data_buffer.size();j++){
                if(std::abs(data_buffer[j].position[0]-buffer1.back().position[0])<0.25&&std::abs(data_buffer[j].position[1]-buffer1.back().position[1])<0.25){
                    buffer1.push_back(data_buffer[j]);
                    data_buffer.erase(data_buffer.begin()+j);
                    break;
                }
                if(j==(data_buffer.size()-1)){
                    buffer2 = data_buffer;
                }
            }

        }
    }


    bool Narrowpassagedetection::compareByPose(const ray_buffer_type &a, const ray_buffer_type &b){

        return (a.position[0] != b.position[0]) ? (a.position[0] < b.position[0]) : (a.position[1] < b.position[1]);
    
    }


}