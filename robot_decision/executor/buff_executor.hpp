#ifndef BUFF_EXECUTOR_H
#define BUFF_EXECUTOR_H

#include <ros/ros.h>
#include "robot_msgs/GameBuff.h"

class Buff_executor
{
    public:
    typedef std::shared_ptr<Buff_executor> Ptr;
    Buff_executor()
    {
        buff_pub_ = nh_.advertise<robot_msgs::GameBuff>("buff", 1);
    }
    ~Buff_executor(){}
    void reset()
    {
        for(int i = 0; i < 6; i++)
        {
            buff_state_[i] = true;
        }
    }
    void set_buff(int id, bool state)
    {
        if(id > 5)
        {
            std::cout << "buff error" << std::endl;
            return;
        }
        buff_state_[id] = state;
    }
    void update()
    {
        robot_msgs::GameBuff buff_state;
        for(int i = 0; i < 6; i++)
        {
            buff_state.zone[i].active = buff_state_[i];
        }
        buff_pub_.publish(buff_state);
    }
    private:
    ros::NodeHandle nh_;
    ros::Publisher buff_pub_;
    bool buff_state_[6];        // true: cannot go through
};

#endif