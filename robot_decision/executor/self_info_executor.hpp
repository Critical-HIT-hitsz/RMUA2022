#ifndef SELF_INFO_EXECUTOR_H
#define SELF_INFO_EXECUTOR_H

#include <ros/ros.h>
#include "robot_msgs/TeamInfo.h"

class Self_Info_executor
{
    public:
    typedef std::shared_ptr<Self_Info_executor> Ptr;
    Self_Info_executor()
    {
        self_info_pub_ = nh_.advertise<robot_msgs::TeamInfo>("self_info", 1);
    }
    ~Self_Info_executor(){}
    void reset()
    {
        attack_enemy_ = 0;
        being_attacked_ = 4;
        to_buff_ = 6;
        pose_x_ = 0;
        pose_y_ = 0;
        pose_x_enemy_ = 0;
        pose_y_enemy_ = 0;
    }
    void Attack_Enemy(uint8_t id)
    {
        attack_enemy_ = id;
    }
    void Being_Attacked(uint8_t armor)
    {
        being_attacked_ = armor;
    }
    void Goto_Buff(uint8_t buff)
    {
        to_buff_ = buff;
    }
    void set_self_pos(double pos_x, double pos_y)
    {
        pose_x_ = (uint16_t)(pos_x * 100.0);
        pose_y_ = (uint16_t)(pos_y * 100.0);
    }
    void set_enemy_pos(double pos_x, double pos_y)
    {
        pose_x_enemy_ = (uint16_t)(pos_x * 100.0);
        pose_y_enemy_ = (uint16_t)(pos_y * 100.0);
    }
    void update()
    {
        robot_msgs::TeamInfo self_info;
        self_info.attack_enemy = attack_enemy_;
        self_info.being_attacked = being_attacked_;
        self_info.to_buff = to_buff_;
        self_info.pose_x_teammate = pose_x_;
        self_info.pose_y_teammate = pose_y_;
        self_info.pose_x_enemy = pose_x_enemy_;
        self_info.pose_y_enemy = pose_y_enemy_;
        self_info_pub_.publish(self_info);
    }
    private:
    ros::NodeHandle nh_;
    ros::Publisher self_info_pub_;
    uint8_t attack_enemy_;
    uint8_t being_attacked_;
    uint8_t to_buff_;
    uint16_t pose_x_;
    uint16_t pose_y_;
    uint16_t pose_x_enemy_;
    uint16_t pose_y_enemy_;
};

#endif
