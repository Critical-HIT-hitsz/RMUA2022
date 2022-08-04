#ifndef CHASSIS_EXECUTOR_H
#define CHASSIS_EXECUTOR_H

#include <ros/ros.h>
#include "robot_msgs/OdomGoalMsg.h"
#include "robot_msgs/RobotFlag.h"
#include <geometry_msgs/PoseStamped.h>
#include "log_executor.hpp"

class Chassis_executor
{
    public:
    enum class ChassisStatus
    {
        IDLE,
        RUNNING
    };
    typedef std::shared_ptr<Chassis_executor> Ptr;
    Chassis_executor(Log_executor::Ptr &log_exe_ptr):
      log_exe_ptr_(log_exe_ptr)
    {
        getback = nh_.subscribe("reach", 1, &Chassis_executor::GetBackCallback, this);
        set_goal_pub = nh_.advertise<robot_msgs::OdomGoalMsg>("goal", 1);
        status_ = ChassisStatus::IDLE;
    }
    Chassis_executor()
    {
        getback = nh_.subscribe("reach", 1, &Chassis_executor::GetBackCallback, this);
        set_goal_pub = nh_.advertise<robot_msgs::OdomGoalMsg>("goal", 1);
        status_ = ChassisStatus::IDLE;
    }
    ~Chassis_executor(){}
    void Stop()//紧急停止
    {
        SendDataToPlan(0, plan_goal_.x, plan_goal_.y, plan_goal_.yaw);
        // status_ = ChassisStatus::IDLE;
    }
    void Goto(double pos_x, double pos_y, double pos_theta, bool Immediately)//正常前向导航
    {
        // std::cout<<"Goto_____"<<std::endl;
        // std::cout <<"     "<<Immediately << std::endl;
        if(Immediately) force_wait_time_++; // 规划强制打断模式设置打断频率
        else force_wait_time_ = 10;
        if(force_wait_time_ >= 5) force_wait_time_ = 0;
        else return;
        // std::cout<<"333333333333"<<std::endl;
        if(status_ == ChassisStatus::RUNNING && Immediately == false) return;
        // std::cout<<"send_data: "<<std::endl;
        SendDataToPlan(1, pos_x, pos_y, pos_theta);
    }
    void GotoBack(double pos_x, double pos_y, double pos_theta, bool Immediately)//正常后向导航
    {
        if(Immediately) force_wait_time_++; // 规划强制打断模式设置打断频率
        else force_wait_time_ = 10;
        if(force_wait_time_ >= 5) force_wait_time_ = 0;
        else return;
        if(status_ == ChassisStatus::RUNNING && Immediately == false) return;
        SendDataToPlan(11, pos_x, pos_y, pos_theta);
    }
    void GotoSwing(double pos_x, double pos_y, double pos_theta, bool Immediately)//摆尾前向导航
    {
        if(Immediately) force_wait_time_++; // 规划强制打断模式设置打断频率
        else force_wait_time_ = 10;
        if(force_wait_time_ >= 5) force_wait_time_ = 0;
        else return;
        if(status_ == ChassisStatus::RUNNING && Immediately == false) return;
        SendDataToPlan(2, pos_x, pos_y, pos_theta);
    }
    void GotoSwingBack(double pos_x, double pos_y, double pos_theta, bool Immediately)//摆尾后向导航
    {
        if(Immediately) force_wait_time_++; // 规划强制打断模式设置打断频率
        else force_wait_time_ = 10;
        if(force_wait_time_ >= 5) force_wait_time_ = 0;
        else return;
        if(status_ == ChassisStatus::RUNNING && Immediately == false) return;
        SendDataToPlan(12, pos_x, pos_y, pos_theta);
    }
    void Rotate(double pos_theta)//原地旋转一个角度
    {
        if(status_ == ChassisStatus::RUNNING) return;
        SendDataToPlan(3, plan_goal_.x, plan_goal_.y, plan_goal_.yaw);
    }
    void Spin()//原地自转
    {
        SendDataToPlan(4, plan_goal_.x, plan_goal_.y, plan_goal_.yaw);
    }
    void IDLE()
    {
        status_ = ChassisStatus::IDLE;
    }
    ChassisStatus status_;
    cv::Point3d to_visual_goal = {0,0,0};
    private:
    void GetBackCallback(const robot_msgs::RobotFlag::ConstPtr back_data)
    {
        if(back_data->flag ) status_ = ChassisStatus::IDLE;
        else
        {
            status_ = ChassisStatus::RUNNING;
        }
    }
    void SendDataToPlan(uint8_t command, double pos_x, double pos_y, double pos_theta)
    {
        // str.clear();
        // str.str("");
        // str << "Plan command: " << (int)command << ", " << pos_x << "," << pos_y << "," << pos_theta;
        // log_exe_ptr_->print(str);
        plan_goal_.mode = command;
        plan_goal_.x = pos_x;
        plan_goal_.y = pos_y;
        plan_goal_.yaw = pos_theta;
        to_visual_goal.x = pos_x;
        to_visual_goal.y = pos_y;
        to_visual_goal.z = pos_theta;
        // std::cout << "mode_; " << plan_goal_.mode << std::endl;
        // std::cout << "plan_goal_.x" << plan_goal_.x << std::endl;
        // std::cout << "plan_goal_.y" << plan_goal_.y << std::endl;
        // std::cout << "plan_goal_.yaw" << plan_goal_.yaw << std::endl;
        set_goal_pub.publish(plan_goal_);
        status_ = ChassisStatus::RUNNING;
    }
    robot_msgs::OdomGoalMsg plan_goal_;
    ros::NodeHandle nh_;
    ros::Subscriber getback;
    ros::Publisher set_goal_pub;
    int force_wait_time_ = 0;
    Log_executor::Ptr log_exe_ptr_;
    std::stringstream str;
};

#endif