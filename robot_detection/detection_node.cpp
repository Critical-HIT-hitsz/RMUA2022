#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include "gimbal_control.h"
#include "armor_detection.h"
#include "robot_msgs/GimbalAngle.h"
#include "robot_msgs/ArmorDetection.h"
#include "robot_msgs/ZMQdata.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/tf.h>
#include "kalman_filter.h"
#include <signal.h>
#include <image_transport/image_transport.h>
#include "cv_bridge/cv_bridge.h"
void SignalHandler(int signal) {
    ros::shutdown();
}

robot_detection::ArmorDetection armor_detector_;
robot_detection::GimbalContrl gimbal_control_;

double last_pitch_;
double last_yaw_;
double delta_pitch, delta_yaw;
double pitch_raw_angle_;
double yaw_raw_angle_;
double yaw_ecd_angle_;
double gimbal_global_angle_;
int missed_cnt_ = 0;
int detected_cnt_ = 0;
int sentry_miss_cnt_ = 0;
double bullet_speed_;

std::vector<cv::Point2d> sentry_enemy_;
int current_sentry_id_ = 0;
bool change_target_flag_ = false;
cv::Point2d current_sentry_enemy_;
bool received_enemy_info_ = false;
std::string color_str;
cv::Point3d self_pose_;
bool received_self_pose_ = false;
int last_detected_cnt_ = 60;
int sentry_cnt_ = 200;

ros::Subscriber gimbal_angle_sub_;
ros::Subscriber sentry_pose_sub_;
ros::Subscriber robot_pose_sub_;
image_transport::Subscriber image_sub_;
ros::Publisher enemy_info_pub_;
ros::Publisher armor_detection_pub_;

bool converge_flag_ = false;
bool estimate_converge_flag_ = false;
bool switch_flag_ = false;
bool switch_process_flag_ = false;
bool detected_enemy_ = false;
bool last_detected_enemy_ = false;

std::unique_ptr<robot_detection::KalmanFilter> pitch_kf_;
std::unique_ptr<robot_detection::KalmanFilter> yaw_kf_;
clock_t image_timestamp_;
clock_t image_last_timestamp_;
bool first_frame = true;

void GimbalAngleCallback(const robot_msgs::GimbalAngle::ConstPtr &msg)
{
    static int bias_count_ = 0;
    static double gimbal_global_angle_bias_;
    pitch_raw_angle_ = msg->pitch_angle;
    yaw_raw_angle_ = msg->yaw_angle;
    yaw_ecd_angle_ = msg->yaw_ecd_angle;
    bullet_speed_ = msg->bullet_speed;
    if(bullet_speed_ > 1e-5)
        gimbal_control_.set_velocity(bullet_speed_);
    
    if(fabs(yaw_ecd_angle_) < 2.0)
    {
        bias_count_++;
        if(bias_count_ > 250)
        {
            // 校准云台偏置
            bias_count_ = 0;
            gimbal_global_angle_bias_ = yaw_raw_angle_ - self_pose_.z * 180.0 / 3.14159265357 - yaw_ecd_angle_;
        }
    }
    else
        bias_count_ = 0;
    double delta = yaw_raw_angle_ - gimbal_global_angle_bias_;
    int round = delta / 360.0;
    delta = delta - round * 360.0;
    if(delta >= 180.0) gimbal_global_angle_ = -(360.0 - delta) * 3.14159265357 / 180.0;
    else if(delta <= -180.0) gimbal_global_angle_ = (360.0 + delta) * 3.14159265357 / 180.0;
    else gimbal_global_angle_ = delta * 3.14159265357 / 180.0;
}

void EskfPoseCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    self_pose_.x = msg->pose.position.x;
    self_pose_.y = msg->pose.position.y;
    self_pose_.z = tf::getYaw(msg->pose.orientation);
    received_self_pose_ = true;
}

void SentryPoseCallback(const robot_msgs::ZMQdata::ConstPtr &msg)
{
    sentry_cnt_ = 0;
    if(color_str == "blue")
    {
        sentry_enemy_[0].x = msg->robot_pose_x[0] / 100.0;
        sentry_enemy_[0].y = msg->robot_pose_y[0] / 100.0;
        sentry_enemy_[1].x = msg->robot_pose_x[1] / 100.0;
        sentry_enemy_[1].y = msg->robot_pose_y[1] / 100.0;
    }
    else if(color_str == "red")
    {
        sentry_enemy_[0].x = msg->robot_pose_x[2] / 100.0;
        sentry_enemy_[0].y = msg->robot_pose_y[2] / 100.0;
        sentry_enemy_[1].x = msg->robot_pose_x[3] / 100.0;
        sentry_enemy_[1].y = msg->robot_pose_y[3] / 100.0;
    }
    if((sentry_enemy_[0].x != 0. && sentry_enemy_[0].y != 0.) &&
       (sentry_enemy_[1].x != 0. && sentry_enemy_[1].y != 0.))
    {
        received_enemy_info_ = true;
        if(sentry_miss_cnt_ > 600)
        {
            current_sentry_id_ = 1 - current_sentry_id_;
            sentry_miss_cnt_ = 0;
        }
    }
    else if((sentry_enemy_[0].x != 0. && sentry_enemy_[0].y != 0.) ||
            (sentry_enemy_[1].x != 0. && sentry_enemy_[1].y != 0.))
    {
        received_enemy_info_ = true;
        if(sentry_enemy_[0].x != 0. && sentry_enemy_[0].y != 0.)
            current_sentry_id_ = 0;
        else
            current_sentry_id_ = 1;
    }
    else
    {
        received_enemy_info_ = false;
    }
}

void ImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    // cv::imshow("img", image);
    // int key = cv::waitKey(1);
    // static int cnt = 0;
    // if(key == 'q')
    // {
    //     char str[10];
    //     sprintf(str, "./%d.jpg", cnt);
    //     cv::imwrite(str, image);
    //     cnt++;
    // }
    // return;

    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image.clone();

    current_sentry_enemy_ = sentry_enemy_[current_sentry_id_];
    if(current_sentry_enemy_.x == 0. || current_sentry_enemy_.y == 0.)
        received_enemy_info_ = false;
    if(sentry_cnt_ > 100)
        received_enemy_info_ = false;
    else
        sentry_cnt_++;
    image_timestamp_ = std::clock();
    double delta_t = (double)(image_timestamp_ - image_last_timestamp_) / CLOCKS_PER_SEC;

    cv::Point3f target_3d_;
    cv::Point3f target_r_;
    double armor_dist = 0.0;
    
    if(!image.empty())
    {
        armor_detector_.DetectArmor(image, detected_enemy_, target_3d_, target_r_);
    }
    double pitch, yaw;
    robot_msgs::GimbalAngle gimbal_angle_;
    robot_msgs::ArmorDetection armor_detection_;
    
    armor_dist = std::sqrt(target_3d_.x * target_3d_.x + target_3d_.y * target_3d_.y + target_3d_.z * target_3d_.z);
    double predict_time = armor_dist / gimbal_control_.init_v_ / 1000.0;

    if (detected_enemy_) 
    {
        sentry_miss_cnt_ = 0;
        double rvec_norm = sqrt(target_r_.x*target_r_.x + target_r_.y*target_r_.y + target_r_.z*target_r_.z);
        armor_detection_.detected_enemy = true;
        gimbal_control_.Transform(target_3d_, delta_pitch, delta_yaw);
        double distance = target_3d_.z;
        gimbal_angle_.distance = target_3d_.z;
        armor_detection_.distance = distance;

        pitch = delta_pitch;
        yaw = delta_yaw + yaw_raw_angle_;
        missed_cnt_ = 20;

        if(first_frame) {
            Eigen::Vector3d pitch_reset, yaw_reset; 
            pitch_reset << 0, 0, 0;
            yaw_reset << yaw_raw_angle_, 0, 0;

            pitch_kf_->Reset(pitch_reset);
            yaw_kf_->Reset(yaw_reset);
            first_frame = false;
            return; 
        }
        
        detected_cnt_++;

        if(std::fabs(delta_yaw) < 2.0 && !converge_flag_)
            converge_flag_ = true;
        else if(std::fabs(delta_yaw) > 2.5 && converge_flag_)
            converge_flag_ = false;
        
        if(std::abs(yaw - yaw_kf_->GetState().x()) < 1.0 && !estimate_converge_flag_)
            estimate_converge_flag_ = true;
        else if(std::abs(yaw - yaw_kf_->GetState().x()) > 1.5 && estimate_converge_flag_)
            estimate_converge_flag_ = false;


        if(std::fabs(yaw - last_yaw_) > 5.0 && last_detected_enemy_) 
            switch_flag_ = true;
        else 
            switch_flag_ = false;
        
        if(switch_flag_)
            switch_process_flag_ = true;
        
        if(switch_process_flag_) {
            pitch_kf_->StateUpdate(delta_t, false);
            pitch_kf_->MeasurementUpdate(pitch, false);
            yaw_kf_->StateUpdate(delta_t, false);
            yaw_kf_->MeasurementUpdate(yaw, false);
            
            if(converge_flag_) {
                switch_process_flag_ = false;
            }
        }
        else {
            pitch_kf_->StateUpdate(delta_t, true);
            pitch_kf_->MeasurementUpdate(pitch, true);
            yaw_kf_->StateUpdate(delta_t, true);
            yaw_kf_->MeasurementUpdate(yaw, true);
        }

        gimbal_angle_.yaw_mode = 1;
        last_detected_cnt_ = 0;
        if(gimbal_angle_.distance > 6000 || gimbal_angle_.distance == 0){
            if(received_enemy_info_ && received_self_pose_)
            {
                double angle = std::atan2(current_sentry_enemy_.y - self_pose_.y, current_sentry_enemy_.x - self_pose_.x);
                double angle_delta = angle - gimbal_global_angle_;
                if(angle_delta > CV_PI) angle_delta -= 2 * CV_PI;
                if(angle_delta < -CV_PI) angle_delta += 2 * CV_PI;
                gimbal_angle_.yaw_mode = 1;
                gimbal_angle_.yaw_angle = angle_delta * 0.1 * 180.0/CV_PI + yaw_raw_angle_;
            }
            else
            {
                gimbal_angle_.yaw_mode = 0;
            }
        }
        gimbal_angle_.yaw_original = yaw;

        gimbal_angle_.yaw_angle = yaw_kf_->GetPredictState(predict_time).x();
        gimbal_angle_.pitch_angle = pitch_kf_->GetPredictState(predict_time).x();
        

        gimbal_angle_.yaw_original = yaw;
        detected_cnt_ = 20;
    }
    else if (!detected_enemy_ && missed_cnt_ > 0) 
    {
        sentry_miss_cnt_ = 0;
        missed_cnt_--;
        gimbal_angle_.yaw_mode = 1;
        last_detected_cnt_ = 0;
        if(gimbal_angle_.distance > 6000 || gimbal_angle_.distance == 0){
            if(received_enemy_info_ && received_self_pose_)
            {
                double angle = std::atan2(current_sentry_enemy_.y - self_pose_.y, current_sentry_enemy_.x - self_pose_.x);
                double angle_delta = angle - gimbal_global_angle_;
                if(angle_delta > CV_PI) angle_delta -= 2 * CV_PI;
                if(angle_delta < -CV_PI) angle_delta += 2 * CV_PI;
                gimbal_angle_.yaw_mode = 1;
                gimbal_angle_.yaw_angle = angle_delta * 0.1 * 180.0/CV_PI + yaw_raw_angle_;
            }
            else
            {
                gimbal_angle_.yaw_mode = 0;
            }
        }

        pitch_kf_->PredictWithVelocity(delta_t);
        yaw_kf_->PredictWithVelocity(delta_t);
        gimbal_angle_.yaw_angle = yaw_kf_->GetPredictState(predict_time).x();
        gimbal_angle_.pitch_angle = pitch_kf_->GetPredictState(predict_time).x();
        gimbal_angle_.yaw_original = yaw_raw_angle_ + 10000.0;
    }
    else
    {
        sentry_miss_cnt_++;
        converge_flag_ = false;
        estimate_converge_flag_ = false;
        switch_flag_ = false;
        switch_process_flag_ = false;

        if(received_self_pose_ && received_enemy_info_)
        {
            if(last_detected_cnt_ < 60) last_detected_cnt_++;
            double angle = std::atan2(current_sentry_enemy_.y - self_pose_.y, current_sentry_enemy_.x - self_pose_.x);
            double angle_delta = angle - gimbal_global_angle_;
            if(angle_delta > CV_PI) angle_delta -= 2 * CV_PI;
            if(angle_delta < -CV_PI) angle_delta += 2 * CV_PI;
            gimbal_angle_.yaw_mode = 1;
            if(angle_delta < 1 && last_detected_cnt_ < 50)
            {
                gimbal_angle_.yaw_angle = yaw_raw_angle_;
            }
            else
            {
                gimbal_angle_.yaw_angle = angle_delta * 0.1 * 180.0/CV_PI + yaw_raw_angle_;
            }
        }
        else
        {
            gimbal_angle_.yaw_mode = 0;
            gimbal_angle_.yaw_angle = delta_yaw + yaw_raw_angle_;
        }
        gimbal_angle_.pitch_angle = delta_pitch;
        gimbal_angle_.yaw_original = yaw_raw_angle_ + 10000.0;
        Eigen::Vector3d pitch_reset, yaw_reset; 
        pitch_reset << 0, 0, 0;
        yaw_reset << yaw_raw_angle_, 0, 0;

        pitch_kf_->Reset(pitch_reset);
        yaw_kf_->Reset(yaw_reset); 
        detected_cnt_ = 0;
        predict_time = 0.0;
    }

    if(detected_enemy_ == 1) {
        armor_detection_.detected_enemy = true;
    }
    else {
        armor_detection_.detected_enemy = false;
    }
    
    enemy_info_pub_.publish(gimbal_angle_);
    
    armor_detection_.yaw_angle = delta_yaw;
    armor_detection_.gimbal_angle = yaw_ecd_angle_;
    armor_detection_.distance = target_3d_.z;
    armor_detection_pub_.publish(armor_detection_);

    // update last state
    image_last_timestamp_ = image_timestamp_;
    last_pitch_ = pitch;
    last_yaw_ = yaw;
    last_detected_enemy_ = detected_enemy_;

    std::cout << "miss_cnt: " << missed_cnt_ << std::endl;
}

int main(int argc, char** argv)
{
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);
    ros::init(argc, argv, "robot_detection_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle ros_nh_;
    sentry_enemy_.resize(2);
    enemy_info_pub_ = ros_nh_.advertise<robot_msgs::GimbalAngle>("cmd_gimbal_angle", 1);
    armor_detection_pub_ = ros_nh_.advertise<robot_msgs::ArmorDetection>("armor_detection_info", 1);
    gimbal_angle_sub_ = ros_nh_.subscribe<robot_msgs::GimbalAngle>("gimbal_angle_info", 1, GimbalAngleCallback);
    sentry_pose_sub_ = ros_nh_.subscribe("robot_zmq_pose", 1, SentryPoseCallback);
    robot_pose_sub_ = ros_nh_.subscribe("eskf_pose", 1, EskfPoseCallback);
    
    image_transport::ImageTransport it(ros_nh_);
    image_sub_ = it.subscribe("/mvsua_cam/image_raw1", 1, ImageCallback);

    //kalmanfilter init
    pitch_kf_ = std::make_unique<robot_detection::KalmanFilter>();
    yaw_kf_ = std::make_unique<robot_detection::KalmanFilter>();
    pitch_kf_->Init(0.01, 5, 0.5, 0.0005);    
    yaw_kf_->Init(0.01, 5, 0.5, 0.0005);

    std::string robot_version, yaml_filename;
    bool color, enable_debug;
    double light_min_angle, light_max_angle_diff, light_aspect_ratio, armor_max_angle, armor_max_aspect_ratio, armor_min_aspect_ratio;
    int blue_threshold, red_threshold, red_brightness_threshold, blue_brightness_threshold, robot_id;
    cv::Mat intrinsic_matrix, distortion_coeffs;
    ros_nh_.param("robot_id", robot_id, 2);
    ros_nh_.getParam("robot_version", robot_version);
    if(robot_version == "rmua")
        yaml_filename = ros::package::getPath("robot_detection") + "/camera_param/camera_param_rmua" + std::to_string(robot_id) + ".yaml";
    else
        yaml_filename = ros::package::getPath("robot_detection") + "/camera_param/camera_param_icra" + std::to_string(robot_id) + ".yaml";
    std::cout << yaml_filename.data() << std::endl;
    cv::FileStorage fs(yaml_filename , cv::FileStorage::READ);
    fs["cameraMatrix"] >> intrinsic_matrix;
    fs["distCoeffs"] >> distortion_coeffs;
    fs.release();
    ros_nh_.getParam("enemy_color", color_str);
    if(color_str == "red")
        color = true;
    else if(color_str == "blue")
        color = false;
    ros_nh_.param("enable_debug", enable_debug, true);
    ros_nh_.param("light_min_angle", light_min_angle, 60.0);
    ros_nh_.param("light_max_angle_diff", light_max_angle_diff, 20.0);
    ros_nh_.param("light_aspect_ratio", light_aspect_ratio, 1.5);
    ros_nh_.param("armor_max_angle", armor_max_angle, 25.0);
    ros_nh_.param("armor_max_aspect_ratio", armor_max_aspect_ratio, 3.0);
    ros_nh_.param("armor_min_aspect_ratio", armor_min_aspect_ratio, 1.2);
    ros_nh_.param("blue_threshold", blue_threshold, 100);
    ros_nh_.param("red_threshold", red_threshold, 50);
    ros_nh_.param("red_brightness_threshold", red_brightness_threshold, 55);
    ros_nh_.param("blue_brightness_threshold", blue_brightness_threshold, 150);
    armor_detector_.LoadParam(color, 
                               enable_debug,
                               light_min_angle, 
                               light_max_angle_diff,
                               light_aspect_ratio, 
                               armor_max_angle, 
                               armor_max_aspect_ratio, 
                               armor_min_aspect_ratio,
                               blue_threshold, 
                               red_threshold,
                               red_brightness_threshold,
                               blue_brightness_threshold,
                               intrinsic_matrix, distortion_coeffs);

    double gimbal_offset_x, gimbal_offset_y, gimbal_offset_z, bullet_init_v, bullet_init_k;
    ros_nh_.param("gimbal_offset_x", gimbal_offset_x, 0.0);
    ros_nh_.param("gimbal_offset_y", gimbal_offset_y, 160.0);
    ros_nh_.param("gimbal_offset_z", gimbal_offset_z, 130.0);
    ros_nh_.param("bullet_init_v", bullet_init_v, 16.0);
    ros_nh_.param("bullet_init_k", bullet_init_k, 0.036);
    if(robot_id == 1) bullet_init_k = 0.001;
    else if(robot_id == 2) bullet_init_k = 0.036;
    gimbal_control_.Init(gimbal_offset_x, gimbal_offset_y, gimbal_offset_z, bullet_init_v, bullet_init_k);
    
    ros::spin();
    return 0;
}