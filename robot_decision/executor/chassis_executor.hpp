#ifndef CHASSIS_EXECUTOR_H
#define CHASSIS_EXECUTOR_H

#include <ros/ros.h>
#include "robot_msgs/OdomMsg.h"
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
        getback = nh_.subscribe("decision_goal_pose_back", 1, &Chassis_executor::GetBackCallback, this);
        set_goal_pub = nh_.advertise<robot_msgs::OdomMsg>("decision_goal_pose", 1);
        status_ = ChassisStatus::IDLE;
    }
    ~Chassis_executor(){}
    void Stop()
    {
        if(status_ == ChassisStatus::RUNNING)
            status_ = ChassisStatus::IDLE;
        SendDataToPlan(0, plan_goal_.position_x, plan_goal_.position_y, plan_goal_.angular_theta);
    }
    void Goto(double pos_x, double pos_y, double pos_theta, bool Immediately)
    {
        if(Immediately) force_wait_time_++;
        else force_wait_time_ = 10;
        if(force_wait_time_ >= 5) force_wait_time_ = 0;
        else return;
        if(status_ == ChassisStatus::RUNNING && Immediately == false) return;
        SendDataToPlan(1, pos_x, pos_y, pos_theta);
    }
    void GotoBack(double pos_x, double pos_y, double pos_theta, bool Immediately)
    {
        if(Immediately) force_wait_time_++; 
        else force_wait_time_ = 10;
        if(force_wait_time_ >= 5) force_wait_time_ = 0;
        else return;
        if(status_ == ChassisStatus::RUNNING && Immediately == false) return;
        SendDataToPlan(11, pos_x, pos_y, pos_theta);
    }
    void GotoSwing(double pos_x, double pos_y, double pos_theta, bool Immediately)
    {
        if(Immediately) force_wait_time_++;
        else force_wait_time_ = 10;
        if(force_wait_time_ >= 5) force_wait_time_ = 0;
        else return;
        if(status_ == ChassisStatus::RUNNING && Immediately == false) return;
        SendDataToPlan(2, pos_x, pos_y, pos_theta);
    }
    void GotoSwingBack(double pos_x, double pos_y, double pos_theta, bool Immediately)
    {
        if(Immediately) force_wait_time_++;
        else force_wait_time_ = 10;
        if(force_wait_time_ >= 5) force_wait_time_ = 0;
        else return;
        if(status_ == ChassisStatus::RUNNING && Immediately == false) return;
        SendDataToPlan(12, pos_x, pos_y, pos_theta);
    }
    void Rotate(double pos_theta)
    {
        if(status_ == ChassisStatus::RUNNING) return;
        SendDataToPlan(3, plan_goal_.position_x, plan_goal_.position_y, pos_theta);
    }
    void Spin()
    {
        SendDataToPlan(4, plan_goal_.position_x, plan_goal_.position_y, plan_goal_.angular_theta);
    }
    ChassisStatus status_;
    private:
    void GetBackCallback(const robot_msgs::OdomMsg::ConstPtr back_data)
    {
        if(back_data->position_x == 1) status_ = ChassisStatus::RUNNING;
        if(back_data->angular_theta == 1)
        {
            status_ = ChassisStatus::IDLE;
        }
    }
    void SendDataToPlan(uint8_t command, double pos_x, double pos_y, double pos_theta)
    {
        // str.clear();
        // str.str("");
        // str << "Plan command: " << (int)command << ", " << pos_x << "," << pos_y << "," << pos_theta;
        // log_exe_ptr_->print(str);
        plan_goal_.command = command;
        plan_goal_.position_x = pos_x;
        plan_goal_.position_y = pos_y;
        plan_goal_.angular_theta = pos_theta;
        set_goal_pub.publish(plan_goal_);
        status_ = ChassisStatus::RUNNING;
    }
    robot_msgs::OdomMsg plan_goal_;
    ros::NodeHandle nh_;
    ros::Subscriber getback;
    ros::Publisher set_goal_pub;
    int force_wait_time_;
    Log_executor::Ptr log_exe_ptr_;
    std::stringstream str;
};

#endif