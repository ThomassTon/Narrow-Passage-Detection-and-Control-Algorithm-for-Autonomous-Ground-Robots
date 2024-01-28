#include"Narrowpassagedetection.hpp"
#include <unistd.h>


namespace narrow_passage_detection{

    bool show=false;

    Narrowpassagedetection::Narrowpassagedetection(ros::NodeHandle& nodeHandle):nh(nodeHandle){

        map_sub = nh.subscribe("/elevation_mapping/elevation_map",1, &Narrowpassagedetection::map_messageCallback,this);
        pose_sub = nh.subscribe("/odom",1,&Narrowpassagedetection::pose_messageCallback,this);
        maxduration.fromSec(1.00);
        setupTimers();
        initialize();
    }

    void Narrowpassagedetection::map_messageCallback(const grid_map_msgs::GridMap& msg)
    {

    // ROS_INFO("Received message: \n\n\n\n\n\n\n\n\n\n\n\n");
        grid_map::GridMapRosConverter::fromMessage(msg, elevationmap);
        outputmap = elevationmap;
        grid_data = outputmap["elevation"];

        outputmap.erase("elevation");
        outputmap.erase("upper_bound");
        outputmap.erase("lower_bound");
        getmap=true;
        show = false;
        // std::stringstream ss;
        // for (const auto& str : elevationmap.getBasicLayers()) {
        //     ss << str << " ";
        // }

        // std::cout << ss.str() << std::endl;
        // /*OUTPUT grid data */
        // std::cout << grid_data<<"\n\n\n\n\n" << std::endl;

        std::ofstream outputfile("/home/haolei/Documents/uperbound.txt");
        if (outputfile.is_open()){
            outputfile << grid_data;
            outputfile.close();
            // const float minValue = elevationmap.get("elevation").minCoeffOfFinites();
            // const float maxValue = elevationmap.get("elevation").maxCoeffOfFinites();
            // ROS_INFO("save success\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
            // std::cout<<"min:  "<<minValue<<"    "<<"max:  "<<maxValue<<"\n\n\n\n\n\n\n\n\n\n\n"<<std::endl;
        }
    }

    void Narrowpassagedetection::pose_messageCallback(const nav_msgs::Odometry &pose)
    {
        pose_msg = pose;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(pose_msg.pose.pose.orientation, quat);
         
        tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);

        // std::cout<<"roll:  "<<roll<<"  pitch:  "<<pitch<<"  yaw: "<<yaw<<std::endl;
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
        // std::vector<std::string> layer;
        // layer.push_back("output");
        // elevationmap.setBasicLayers(layer);
        // outputmap.setFrameId("world_id");
        // grid_map::Length len;
        // len(0)=len(1)=5.0;
        // outputmap.setGeometry(len,0.05);
        mapUpdateTimer_.start();


        map_pub = nh.advertise<grid_map_msgs::GridMap>("/narrow_passage_map", 1);
    }

    void Narrowpassagedetection::narrowmap_pub(){
        ROS_INFO("publish test publish test\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
        // cv::imshow("test",input_img);
        // cv::imwrite("/home/haolei/Documents/test.jpg",input_img);
        // std::ofstream outputfile("/home/haolei/Documents/img.txt");
        // if (outputfile.is_open()){
        //     outputfile << input_img;
        //     outputfile.close();

        //     ROS_INFO("save success\n\n\n\n\n\n\n\n\n\n\n\n\n\n");

        // }
        // std::cout<<input_img(cv::Range(90,91),cv::Range::all())<<std::endl;
        // show=true;
        // sleep(5);
        if(generate_output())
        {   
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

        // const grid_map::Matrix& grid_data (outputmap["elevation"]);
        // std::cout<<grid_data<<std::endl;
        // elevationmap.erase("elevation");
        // if(result){
        //     return true;
        // }
        // return false;
        return true;
    }

    void Narrowpassagedetection::computegradient(){
        int len1= input_img.rows;
        int len2= input_img.cols;
        cv::imwrite("/home/haolei/Documents/input.jpg",input_img);

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


        cv::imwrite("/home/haolei/Documents/gradient.jpg",gradient);
        std::ofstream outputfile2("/home/haolei/Documents/gradient.txt");
        if (outputfile2.is_open()){
            outputfile2 << gradient;
            outputfile2.close();
            ROS_INFO("save success\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
        }
        std::ofstream outputfile("/home/haolei/Documents/input.txt");
        if (outputfile.is_open()){
            outputfile << input_img;
            outputfile.close();
            ROS_INFO("save success\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
        }
        std::cout<<"len1 : "<<len1<<"  len2: "<<len2<<"\n\n\n\n\n\n\n\n\n\n\n\n\n"<<std::endl;
        std::cout<<"datatype : "<<magnitude.type()<<"\n\n\n\n\n\n\n\n\n\n\n\n\n"<<std::endl;
        for(int i=0;i<gradient.rows;i++)  
        {  
            for(int j=0;j<gradient.cols;j++)  
            {  
                //magnitude.at<Vec3b>(i,j)[0]=magnitude.at<Vec3b>(i,j)[0]/div*div+div/2;  
                // std::cout<<magnitude.at<uchar>(i,j)<<std::endl;
                if(gradient.at<uchar>(i,j)< uchar(200))
                {
                    gradient.at<uchar>(i,j)=uchar(0);
                }
                else{
                    gradient.at<uchar>(i,j) = uchar(255);
                }
            }  
        }
        convert_from_gradient();

        cv::imwrite("/home/haolei/Documents/gradient_2.jpg",gradient);
        std::ofstream outputfile3("/home/haolei/Documents/gradient_2.txt");

        if (outputfile3.is_open()){
            outputfile3 << gradient;
            outputfile3.close();
            ROS_INFO("save success\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
        }
        cv::imwrite("/home/haolei/Documents/gradient_3.jpg",input_img);

    }

    void Narrowpassagedetection::convert_from_gradient(){
        // std::cout<< elevationmap.getStartIndex()[0] <<std::endl;
        // for(int i=0;i<gradient.rows;i++)  
        // {  
        //     for(int j=0;j<gradient.cols;j++)  
        //     {  
        //         //magnitude.at<Vec3b>(i,j)[0]=magnitude.at<Vec3b>(i,j)[0]/div*div+div/2;  
        //         // std::cout<<magnitude.at<uchar>(i,j)<<std::endl;
        //         // if(gradient.at<uchar>(i,j) == uchar(255)&&input_img.at<uchar>(i,j) > uchar(70))
        //         // {
        //         //         input_img.at<uchar>(i,j) = uchar(255);
        //         // }
        //         // else{
        //         //         input_img.at<uchar>(i,j) = uchar(0);
        //         // }
        //         if(gradient.at<uchar>(i,j) == uchar(0)){
        //             // std::cout<<grid_data(i+65,j+47)<<std::endl;
        //             grid_data(i+65,j+47) = 0;
        //         }
                
        //     }  
        // }

        for (grid_map::GridMapIterator iterator(elevationmap);!iterator.isPastEnd(); ++iterator ){
            const grid_map::Index index(*iterator);
            const float& value = grid_data(index(0), index(1));
            const grid_map::Index imageIndex(iterator.getUnwrappedIndex());
            if(gradient.at<uchar>(imageIndex(0),imageIndex(1))== uchar(0)&&std::isfinite(value)){
                grid_data(index(0), index(1)) = NAN;
            }
        }



    }
}