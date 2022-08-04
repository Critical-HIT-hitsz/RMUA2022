#ifndef DECISION_NODE_H
#define DECISION_NODE_H

#include <ros/ros.h>
#include "behavior_tree.h"
#include "chassis_executor.hpp"
#include "buff_executor.hpp"
#include "self_info_executor.hpp"
#include "blackboard.hpp"
#include "add_blood.hpp"
#include "add_bullet.hpp"
#include "attack_enemy.hpp"
#include "game_start.hpp"
#include "leave_buff.hpp"
#include "patrol.hpp"
#include "retreat.hpp"
#include "support_teammate.hpp"
#include "log_executor.hpp"
#include "global_map.hpp"
#include <thread>

namespace robot_decision
{
    class decision_node
    {
    public:
        decision_node();
        ~decision_node()
        {
            if (decision_thread_.joinable()) {
                decision_thread_running_ = false;
                decision_thread_.join();
            }
            delete root_node_;
        }
        void ExecuteLoop();
        void visualize();
    private:
        SequenceNode* root_node_;
        Blackboard::Ptr blackboard_;
        Chassis_executor::Ptr chassis_exe_;
        Self_Info_executor::Ptr self_info_exe_;
        Buff_executor::Ptr buff_exe_;
        Log_executor::Ptr log_exe_;
        bool visualize_flag_;
        // cv::Mat map_;
        std::thread decision_thread_;
        bool decision_thread_running_;
        int loop_rate_;
    };
}

#endif