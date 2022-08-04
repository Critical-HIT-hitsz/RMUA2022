#include <builder/trt_builder.hpp>
#include <common/ilogger.hpp>
#include <common/cuda_tools.hpp>
#include <infer/trt_infer.hpp>
#include <common/ilogger.hpp>
#include <common/preprocess_kernel.cuh>
#include <common/trt_tensor.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include "robot_msgs/SentryData.h"
#include "robot_msgs/SharePath.h"
#include <time.h>
#include <algorithm>
#include <thread>
#include <signal.h>

#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.h>

#include "trrt_detector.h"
#include "visualize.h"
#include <rsort.h>
#include <mutex>

using namespace std;

const std::string class_names[7] = {"red1", "red2", "blue1", "blue2","die1", "die2","background"};
int team_pose[4] = {0};
bool first_flag = true;
bool ID_switch_flag = false;
cv::Point ID_swich_point;
cv::Mat master_frame,slave_frame;
cv::Mat master_undisort_frame,slave_undisort_frame;
cv::Mat master_warp_frame,slave_warp_frame;
cv::Mat mapx_master, mapy_master, mapx_slave, mapy_slave;
cv::Mat master_H,slave_H;
std::vector<detbox> master_detection_list;
std::vector<detbox> slave_detection_list;
std::vector<YoloBox> master_yolo_boxes;
std::vector<YoloBox> slave_yolo_boxes;
int master_sentry_id;
int slave_sentry_id;
//Camera params 
std::vector<cv::Scalar> colors{cv::Scalar(0,0,255),cv::Scalar(255,0,0),cv::Scalar(0,0,0)};



cv::Mat master_instrinsic = (cv::Mat_<double>(3,3) << 552.715667942477, 0., 578.733089360504, 
                0.,553.031272490833, 509.815324025131, 
                0., 0., 1. );
cv::Mat master_disortion = (cv::Mat_<double>(1, 5) <<  -0.00202994240321718,-0.0144495139448957,0,0,0.00380485745278735);

cv::Mat slave_instrinsic = (cv::Mat_<double>(3,3) << 551.761226999728, 0, 625.026564672521,
                                                                    0,552.116053613643,491.015888823710,
                                                                    0,0,1);
cv::Mat slave_disortion = (cv::Mat_<double>(1, 5) << -0.003705270824610,-0.015134281146654,0,0,0.003768462291391);


//Set the initial car ID    
void switch_ID_callback(int event,int x,int y,int flags,void *ustc)
{
    if(event == cv::EVENT_LBUTTONDOWN)
    {
        ID_swich_point= cv::Point(x,448-y);
        ID_switch_flag = true;
    }
}
//Calculte rotate box points according to the x,y,width,height and angle
Eigen:: VectorXd get_rbox_point(detbox detection,int y_rows)
{       
    // 这个地方长宽有点小bug 但能用
        double width = 60;
        double height = 50;

        double x = detection.x_pos;
        double y  = detection.y_pos;
        double cosA = cos(detection.car_angle);
        double sinA = sin(detection.car_angle);
        

        double  x1 = x - 0.5 * width;
        double  y1 = y - 0.5 * height;

        double x0 = x + 0.5 * width;
        double y0 = y1;

        double x2 = x1;
        double y2 = y + 0.5 * height;
        double x3 = x0;
        double y3 = y2;

        Eigen::VectorXd rbox_point(8);
        double x0n = (x0 - x) * cosA - (y0-y) * sinA + x;
        double y0n = (x0 - x) * sinA + (y0-y) * cosA + y;
        rbox_point(0) = x0n;
        rbox_point(1) = y_rows- y0n;

        double x1n = (x1 - x) * cosA - (y1-y) * sinA + x;
        double y1n = (x1 - x) * sinA + (y1-y) * cosA + y;
        rbox_point(2) = x1n;
        rbox_point(3) = y_rows-y1n;

        double x2n = (x2 - x) * cosA - (y2-y) * sinA + x;
        double y2n = (x2 - x) * sinA + (y2-y) * cosA + y;
        rbox_point(4) = x2n;
        rbox_point(5) = y_rows - y2n;


        double x3n = (x3 - x) * cosA - (y3-y) * sinA +  x; 
        double y3n = (x3 - x) * sinA + (y3-y) * cosA + y;
        rbox_point(6) = x3n;
        rbox_point(7) = y_rows -y3n;
        return rbox_point;
}

std::mutex mutex1, mutex2;
bool team_locate_flags[2] = {false,false};
int locate_cnt1 = 0, locate_cnt2 = 0;
//Get the pose from the robot
void TeamPathCallback(const robot_msgs::SharePath::ConstPtr &msg)
{
    if(msg->id == 1)
    {
        locate_cnt1 = 0;
        team_locate_flags[0] = true;
        team_pose[0] = msg->pos_x[0];
        team_pose[1] = msg->pos_y[0];
    }
    else if(msg->id == 2)
    {
        locate_cnt2 = 0;
        team_locate_flags[1] = true;
        team_pose[2] = msg->pos_x[0];
        team_pose[3] = msg->pos_y[0];
    }
    else if(msg->id == 0)
    {

       team_pose[0] = msg->pos_x[0];
       team_pose[1] = msg->pos_y[0];
       team_pose[2] = msg->pos_x[1];
       team_pose[3] = msg->pos_y[2];
    }
    if(locate_cnt1>10)
    {
        team_locate_flags[0] = false;
        std::cout<<"Car 1 location lost!"<<std::endl;
    }
    if(locate_cnt2>10)
    {
        team_locate_flags[1] = false;
        std::cout<<"Car 2 location lost!"<<std::endl;
    }
   
}
void MastercompressedImageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
{

    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr_compressed->image;
    master_frame = image.clone();
    cv::remap(master_frame, master_undisort_frame, mapx_master, mapy_master, cv::INTER_LINEAR);
    std::lock_guard<std::mutex> guard (mutex1, std::adopt_lock);
    cv::warpPerspective(master_undisort_frame, master_warp_frame, master_H, cv::Size(900, 480));
}
void SlavecompressedImageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr_compressed->image;
    slave_frame = image.clone();
    cv::remap(slave_frame, slave_undisort_frame, mapx_slave, mapy_slave, cv::INTER_LINEAR);
    std::lock_guard<std::mutex> guard (mutex2, std::adopt_lock);
    cv::warpPerspective(slave_undisort_frame, slave_warp_frame, slave_H, cv::Size(900, 480));
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "sentry_node");
    ros::NodeHandle ros_nh_;

    //Change the image topic here
    ros::Subscriber master_image_sub = ros_nh_.subscribe("/mvsua_cam/image_raw1/compressed",1,MastercompressedImageCallback);
    ros::Subscriber slave_image_sub = ros_nh_.subscribe("/mvsua_cam/image_raw2/compressed",1,SlavecompressedImageCallback);
    ros::Publisher sentry_pub = ros_nh_.advertise<robot_msgs::SentryData>("/wireless/sentry_send", 1);
    ros::Publisher sentry_zmq_pub = ros_nh_.advertise<robot_msgs::SentryData>("/zmq/sentry_send", 1);

    ros::Subscriber path_sub = ros_nh_.subscribe<robot_msgs::SharePath>("/wireless/team_path",10,TeamPathCallback);

    //Define and get params

    cv::initUndistortRectifyMap(master_instrinsic, master_disortion, cv::Mat(), master_instrinsic, cv::Size(1280, 1024), CV_16SC2, mapx_master, mapy_master);
    cv::initUndistortRectifyMap(slave_instrinsic, slave_disortion, cv::Mat(), slave_instrinsic, cv::Size(1280, 1024), CV_16SC2, mapx_slave, mapy_slave);

    std::string master_H_file,slave_H_file;
    bool v_yolo_flag,v_track_map,v_track_msg,v_origin_flag= false;
    int color_flag = 0;
    bool fix_id_flag = true;
    ros_nh_.getParam("visualize_Yolo", v_yolo_flag);
    ros_nh_.getParam("visualize_TrackMap", v_track_map);
    ros_nh_.getParam("Ouput_TrackMsg", v_track_msg);
    ros_nh_.getParam("visualize_Origin", v_origin_flag);
    ros_nh_.getParam("color_flag", color_flag);
    ros_nh_.getParam("fix_flag", fix_id_flag);
    //color flag: 0 for red, 2 for blue

    //Set Sentry ID
    if(color_flag ==2)
    {
        master_sentry_id  = 3;
        slave_sentry_id = 1;
    }
    if(color_flag ==0)
    {
        master_sentry_id = 4;
        slave_sentry_id = 2;
    }
    //Read Homography matrix
    master_H_file = ros::package::getPath("tools")+"/homo_mat/master_H.yaml"; 
    slave_H_file = ros::package::getPath("tools")+"/homo_mat/slave_H.yaml";
    cv::FileStorage fs(master_H_file , cv::FileStorage::READ);
    fs["Homography"] >> master_H;
    fs.release();
    cv::FileStorage fs2(slave_H_file , cv::FileStorage::READ);
    fs2["Homography"] >> slave_H;
    fs2.release();


    if(!fix_id_flag)
    {
        std::cout<<"FIX ID CLOSE!"<<std::endl;
    }

    
    //Init tensorrt Model

    std::string onnx_file = ros::package::getPath("sentry_beta")+"/model/0521fixmosaic_sim.onnx";
    std::string model_file = ros::package::getPath("sentry_beta")+"/model/0521fixmosaic_sim.trtmodel";
    //Create Engine
    trrt_detector infer_engine(model_file,onnx_file,640);


    std::vector<std::vector<detbox>> mixout_detection;
    std::vector<detbox> tracking_result;

    double delta_t = 0;
    double start_time_flag = cv::getTickCount();
    double last_time_flag = cv::getTickCount();
    cv::namedWindow("Tracked Map");
    cv::setMouseCallback("Tracked Map", switch_ID_callback, 0);

    //Init the tracker
    R_SORTER mot_tracker(color_flag);
    //Init Output Filter waiting for the first tracking result
    OutPutFilter trackerfilter;
    bool filter_init_flag = true;
    bool first_frame_flag = true;
    while(ros::ok())
    {
        //Calculate the times when sentry lost the data from robot
        locate_cnt1++;
        locate_cnt2++;

        ros::spinOnce();  
        master_detection_list.clear();
        master_yolo_boxes.clear();

        //Infer and get the detection
        if(!master_warp_frame.empty())
            infer_engine.infer(master_warp_frame,master_sentry_id,master_yolo_boxes,master_detection_list);
        slave_yolo_boxes.clear();
        slave_detection_list.clear();
        if(!slave_warp_frame.empty())
            infer_engine.infer(slave_warp_frame,slave_sentry_id,slave_yolo_boxes,slave_detection_list);

        cv::Mat master_track_map = cv::imread(ros::package::getPath("sentry_beta")+"/background.png");
        cv::Mat slave_track_map = cv::imread(ros::package::getPath("sentry_beta")+"/background.png");
        cv::Mat tracked_map = slave_track_map.clone();

        
        start_time_flag = cv::getTickCount();
        mixout_detection.clear();
        mixout_detection.resize(2);
        delta_t  = (double)(start_time_flag - last_time_flag)/cv::getTickFrequency();
        tracking_result.clear();

      
        //Init four tracker in the first frame
        if(first_frame_flag)
        {
            //Choose a camera that has fewer results
            if(master_detection_list.size()<slave_detection_list.size())//
            {
                mixout_detection[0] = master_detection_list;
                
                std::cout<<"Matster Detection Size: " << master_detection_list.size()<<std::endl;
                std::cout<<"Slave Detection Size: " <<  slave_detection_list.size()<<std::endl;
                //Try to init the tracker
                tracking_result = mot_tracker.UpdateWithTwoCam(mixout_detection,1,first_frame_flag,team_pose,team_locate_flags);
                std::cout<<"FLAG: "<<first_frame_flag<<std::endl;
                //Check if trackers are available
                std::cout<<mot_tracker.CheckStartTracker(tracking_result)<<std::endl;
                std::cout<<"Output Result "<<std::endl;
                for (size_t i = 0; i < tracking_result.size(); i++)
                {
                    
                    std::cout<< tracking_result[i].x_pos<< " ";
                    std::cout<< tracking_result[i].y_pos<< " ";
                    std::cout<< tracking_result[i].car_angle<< " ";
                    std::cout<< tracking_result[i].car_type<< " ";
                    std::cout<< tracking_result[i].tracker_id<< " "<<std::endl;
                }

                if(mixout_detection[0].size()>=0&&mot_tracker.CheckStartTracker(tracking_result))
                {
                    first_frame_flag = false;
                }
                //Not available, check the other camera
                if(first_frame_flag)
                {
                    mixout_detection[0] = slave_detection_list;
                    tracking_result = mot_tracker.UpdateWithTwoCam(mixout_detection,1,first_frame_flag,team_pose,team_locate_flags);
                    if(mixout_detection[0].size()>=0&&mot_tracker.CheckStartTracker(tracking_result))
                    {
                        first_frame_flag = false;
                    } 
                }
            }
            //Same as above
            else if(master_detection_list.size()>=slave_detection_list.size())
            {
                mixout_detection[0] = slave_detection_list;
                std::cout<<"Matster Detection Size: " << master_detection_list.size()<<std::endl;
                std::cout<<"Slave Detection Size: " <<  slave_detection_list.size()<<std::endl;
                tracking_result = mot_tracker.UpdateWithTwoCam(mixout_detection,1,first_frame_flag,team_pose,team_locate_flags);
                std::cout<<"FLAG: "<<first_frame_flag<<std::endl;
                std::cout<<mot_tracker.CheckStartTracker(tracking_result)<<std::endl;
                std::cout<<"Output Result "<<std::endl;
                for (size_t i = 0; i < tracking_result.size(); i++)
                {
                    
                    std::cout<< tracking_result[i].x_pos<< " ";
                    std::cout<< tracking_result[i].y_pos<< " ";
                    std::cout<< tracking_result[i].car_angle<< " ";
                    std::cout<< tracking_result[i].car_type<< " ";
                    std::cout<< tracking_result[i].tracker_id<< " "<<std::endl;
                }

                if(mixout_detection[0].size()>=0&&mot_tracker.CheckStartTracker(tracking_result))
                {
                    first_frame_flag = false;
                }
                if(first_frame_flag)
                {
                    mixout_detection[0] = master_detection_list;
                    tracking_result = mot_tracker.UpdateWithTwoCam(mixout_detection,1,first_frame_flag,team_pose,team_locate_flags);
                    if(mixout_detection[0].size()>=0&&mot_tracker.CheckStartTracker(tracking_result))
                    {
                        first_frame_flag = false;
                    } 
                }
            }
            
           
        }

        //Init the tracker successfully
        else
        {
            bool master_available,slave_available;
            master_available = slave_available = true;
            //Mix the detection in one vector
            if(master_available) mixout_detection[0] = master_detection_list;
            if(slave_available)  mixout_detection[1] = slave_detection_list;

            //Update the tracker
            tracking_result = mot_tracker.UpdateWithTwoCam(mixout_detection,delta_t,first_frame_flag,team_pose,team_locate_flags);
        }
        if(tracking_result.size()>0&&ID_switch_flag==true)
        {
            //Swich the initial ID
            mot_tracker.switch_trackid(ID_swich_point);
        }
            ID_switch_flag = false;

        //Init the ouput filter using the tracking result
        if(filter_init_flag)
        {
            bool filter_init_state = trackerfilter.InitOutputFilter(tracking_result);
            if(filter_init_flag == true)
            {
                filter_init_flag = false;
            }
        }



        //Publish sentry data
        if(!first_frame_flag)
        {
            //Using ouput filter to reduce the delay
            trackerfilter.TrackerPredict(delta_t,tracking_result);
            trackerfilter.TrackerFixUpdate(tracking_result);
            robot_msgs::SentryData msg;
            for(int i = 0;i<tracking_result.size();i++)
            {
                msg.lable[i] = tracking_result[i].car_type*10+tracking_result[i].tracker_id;
                msg.x[i] = (uint16_t)(tracking_result[i].x_pos*10);
                msg.y[i] = (uint16_t)(tracking_result[i].y_pos*10);
                msg.yaw[i] = (int16_t)(tracking_result[i].car_angle*100);
            }
            sentry_pub.publish(msg); 
            sentry_zmq_pub.publish(msg);
        }


        //Visualize
        cv::Mat yolo_master_out;
        cv::Mat yolo_slave_out; 
        //visualize Yolo Box
        if(v_yolo_flag)
        {
            if(master_sentry_id==3&&slave_sentry_id==1)
            {
               
                cv::flip(master_warp_frame,yolo_master_out,0);
                cv::flip(slave_warp_frame,yolo_slave_out,1);
                for(int i = 0;i<master_yolo_boxes.size();i++)
                {   
                    cv::Scalar color = colors[master_yolo_boxes[i].cate/2];
                    cv::rectangle(yolo_master_out,cv::Point(master_yolo_boxes[i].x1,480-master_yolo_boxes[i].y1),cv::Point(master_yolo_boxes[i].x2,480-master_yolo_boxes[i].y2),\
                    cv::Scalar(0,255,0),2);
                    float center_x = (master_yolo_boxes[i].x1+master_yolo_boxes[i].x2)*0.5;
                    float center_y = 480-(master_yolo_boxes[i].y1+master_yolo_boxes[i].y2)*0.5;
                    float angle = master_yolo_boxes[i].angle_;
                    cv::arrowedLine(yolo_master_out,cv::Point(center_x,center_y),cv::Point(center_x+55*cos(angle),center_y+55*sin(angle)),cv::Scalar(0,255,0),4);
                    cv::putText(yolo_master_out,class_names[master_yolo_boxes[i].cate], cv::Point(int(center_x),int(center_y)),1,1.5,color,2);                    cv::putText(yolo_master_out,class_names[master_yolo_boxes[i].cate], cv::Point(int(center_x),int(center_y)),1,1.5,color,2);
                    cv::putText(yolo_master_out,std::to_string(master_yolo_boxes[i].score), cv::Point(int(center_x+10),int(center_y+10)),1,1.5,cv::Scalar(255,255,255),2);
                }
                
                for(int i = 0;i<slave_yolo_boxes.size();i++)
                {
                    cv::Scalar color = colors[slave_yolo_boxes[i].cate/2];
                    cv::rectangle(yolo_slave_out,cv::Point(900-slave_yolo_boxes[i].x1,slave_yolo_boxes[i].y1),cv::Point(900-slave_yolo_boxes[i].x2,slave_yolo_boxes[i].y2),\
                    cv::Scalar(0,255,0),2);
            
                    float center_x = 900-(slave_yolo_boxes[i].x1+slave_yolo_boxes[i].x2)*0.5;
                    float center_y = (slave_yolo_boxes[i].y1+slave_yolo_boxes[i].y2)*0.5;
                    float angle = -slave_yolo_boxes[i].angle_+M_PI;
                    cv::arrowedLine(yolo_slave_out,cv::Point(center_x,center_y),cv::Point(center_x+55*cos(angle),center_y-55*sin(angle)),cv::Scalar(0,255,0),4);
                    cv::putText(yolo_slave_out,class_names[slave_yolo_boxes[i].cate], cv::Point(int(center_x),int(center_y)),1,1.5,color,2);
                    cv::putText(yolo_slave_out,std::to_string(slave_yolo_boxes[i].score), cv::Point(int(center_x+10),int(center_y+10)),1,1.5,cv::Scalar(255,255,255),2);

                }

            }
            else if(master_sentry_id==4&&slave_sentry_id==2)
            {
                cv::flip(slave_warp_frame,yolo_slave_out,-1);
                yolo_master_out = master_warp_frame.clone();
                //cv::flip(slave_warp_frame,yolo_slave_out,-1);
                for(int i = 0;i<slave_yolo_boxes.size();i++)
                {   
                    cv::Scalar color = colors[slave_yolo_boxes[i].cate/2];
                    cv::rectangle(yolo_slave_out,cv::Point(900-slave_yolo_boxes[i].x1,480-slave_yolo_boxes[i].y1),cv::Point(900-slave_yolo_boxes[i].x2,480-slave_yolo_boxes[i].y2),\
                    cv::Scalar(0,255,0),2);
                    float center_x = 900-(slave_yolo_boxes[i].x1+slave_yolo_boxes[i].x2)*0.5;
                    float center_y = 480-(slave_yolo_boxes[i].y1+slave_yolo_boxes[i].y2)*0.5;
                    float angle = slave_yolo_boxes[i].angle_+M_PI;
                    cv::arrowedLine(yolo_slave_out,cv::Point(center_x,center_y),cv::Point(center_x+55*cos(angle),center_y-55*sin(angle)),cv::Scalar(0,255,0),4);
                    cv::putText(yolo_slave_out,class_names[slave_yolo_boxes[i].cate], cv::Point(int(center_x),int(center_y)),1,1.5,color,2);
                }
                for(int i = 0;i<master_yolo_boxes.size();i++)
                {
                    cv::Scalar color = colors[master_yolo_boxes[i].cate/2];
                    cv::rectangle(yolo_master_out,cv::Point(master_yolo_boxes[i].x1,master_yolo_boxes[i].y1),cv::Point(master_yolo_boxes[i].x2,master_yolo_boxes[i].y2),\
                    cv::Scalar(0,255,0),2);
            
                    float center_x = (master_yolo_boxes[i].x1+master_yolo_boxes[i].x2)*0.5;
                    float center_y = (master_yolo_boxes[i].y1+master_yolo_boxes[i].y2)*0.5;
                    float angle = master_yolo_boxes[i].angle_;
                    cv::arrowedLine(yolo_master_out,cv::Point(center_x,center_y),cv::Point(center_x+55*cos(angle),center_y-55*sin(angle)),cv::Scalar(0,255,0),4);
                    cv::putText(yolo_master_out,class_names[master_yolo_boxes[i].cate], cv::Point(int(center_x),int(center_y)),1,1.5,color,2);
                }
            }
           
        }
        //Output tracker result
        if(v_track_msg)
        {
            for (size_t i = 0; i < tracking_result.size(); i++)
            {
                std::cout<<"Output Result "<<std::endl;
                std::cout<< tracking_result[i].x_pos<< " ";
                std::cout<< tracking_result[i].y_pos<< " ";
                std::cout<< tracking_result[i].car_angle<< " ";
                std::cout<< tracking_result[i].car_type<< " ";
                std::cout<< tracking_result[i].tracker_id<< " "<<std::endl;
            }

        }
        //Ouput track map
        if(v_track_map)
        {
            for(int i = 0;i<tracking_result.size();i++)
            {
                Eigen::VectorXd det_result(get_rbox_point(tracking_result[i],448));

                cv::Scalar color = colors[tracking_result[i].car_type/2];
                
                cv::line(tracked_map,cv::Point(int(det_result[0]),int(det_result[1])),cv::Point(int(det_result[2]),int(det_result[3])),color,4);
                cv::line(tracked_map,cv::Point(int(det_result[2]),int(det_result[3])),cv::Point(int(det_result[4]),int(det_result[5])),color,4);
                cv::line(tracked_map,cv::Point(int(det_result[4]),int(det_result[5])),cv::Point(int(det_result[6]),int(det_result[7])),color,4);
                cv::line(tracked_map,cv::Point(int(det_result[6]),int(det_result[7])),cv::Point(int(det_result[0]),int(det_result[1])),color,4);
                
                double det_x  = (int)(tracking_result[i].x_pos);
                double det_y = (int)(tracking_result[i].y_pos);
                double angle = tracking_result[i].car_angle;

                cv::circle(tracked_map, cv::Point(det_x,448-det_y),4,cv::Scalar(255,0,255),-1);
                cv::arrowedLine(tracked_map,cv::Point(det_x,448-det_y),cv::Point(det_x+55*cos(angle),(448-det_y)-55*sin(angle)),cv::Scalar(0,255,0),4);
                cv::putText(tracked_map,class_names[tracking_result[i].car_type], cv::Point(int(det_result[0]),int(det_result[1])),1,1.1,cv::Scalar(0,0,0),1);
                cv::putText(tracked_map,class_names[tracking_result[i].tracker_id], cv::Point(det_x-20,448-det_y),1,1.5,cv::Scalar(0,255,255),2);
            }
        }
        
        if(!master_warp_frame.empty()&&!slave_warp_frame.empty()&&v_yolo_flag)
        {
            //std::cout<<"Visualize"<<std::endl;
           
            cv::imshow("Matser frame",yolo_master_out);
            cv::imshow("Slave frame",yolo_slave_out);
            
        }
        //Output origin img
        cv::Mat master_show, slave_show;
        if(!master_frame.empty()&&!slave_frame.empty()&&v_origin_flag)
        {
            cv::resize(master_frame, master_show, cv::Size(640, 512));
            cv::resize(slave_frame, slave_show, cv::Size(640, 512));
            cv::imshow("Master",master_show);
            cv::imshow("Slave",slave_show);
        }

        cv::imshow("Tracked Map",tracked_map);
        last_time_flag = start_time_flag;
        char key = cv::waitKey(1);
        //Use the data from the robots to fix the tracker
        if(key == 'q')
        {
            mot_tracker.fix_flag = true;
            std::cout<<"Fix Mode Open!"<<std::endl;
        }
        //Close the fix mode
        else if(key == 'w')
        {
            mot_tracker.fix_flag = false;
            std::cout<<"Fix Mode Close!"<<std::endl;
        }
        
        //std::cout<<"==============================="<<std::endl;
    }


    return 0;
}

