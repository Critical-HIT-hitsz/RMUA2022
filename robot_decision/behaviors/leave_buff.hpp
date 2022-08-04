#ifndef LEAVE_BUFF_H
#define LEAVE_BUFF_H

#include "behavior_tree.h"
#include "self_info_executor.hpp"
#include "buff_executor.hpp"
#include "log_executor.hpp"

namespace robot_decision
{
    class LeaveBuffBehavior : public ActionNode
    {
    public:
        LeaveBuffBehavior(std::string name,
                          int level,
                          const Blackboard::Ptr &blackboard_ptr,
                          Chassis_executor::Ptr &chassis_exe_ptr,
                          Log_executor::Ptr &log_exe_ptr,
                          Buff_executor::Ptr &buff_exe_ptr,
                          Self_Info_executor::Ptr &self_info_exe_ptr) : ActionNode::ActionNode(name, level, blackboard_ptr, chassis_exe_ptr, log_exe_ptr, buff_exe_ptr, self_info_exe_ptr)
        {}
        //躲避buff区以免踩到惩罚区和敌方加弹区
        BehaviorState Update()
        {
            // std::cout << "leave_buff" << std::endl;
            int remain_time = blackboard_ptr_->game_status_.remaining_time;
            if ((remain_time < 130 && remain_time > 120) || (remain_time < 70 && remain_time > 60))
            {
                for (int i = 0; i < blackboard_ptr_->buff_pos_.size(); i++)
                {
                    double distance = sqrt(pow(blackboard_ptr_->buff_pos_[i].x - blackboard_ptr_->self_pose_.x, 2) +
                                           pow(blackboard_ptr_->buff_pos_[i].y - blackboard_ptr_->self_pose_.y, 2));
                    if (distance < 0.2)
                    { // 与buff区距离过小
                        // log_exe_ptr_->print(str);
                        std::cout << "leave_buff" << std::endl;
                        GoPatrol(); // 到最近的巡逻点以离开buff区
                        return BehaviorState::SUCCESS;
                    }
                }
            }
            return BehaviorState::FAILURE;
        }

    private:
        void GoPatrol()
        {
            int index = 0;
            double distance = 100.0;
            for (int i = 0; i < sizeof(blackboard_ptr_->patrol_pos) / sizeof(blackboard_ptr_->patrol_pos[0]); i++)
            {
                double delta_x = blackboard_ptr_->self_pose_.x - blackboard_ptr_->patrol_pos[i][0];
                double delta_y = blackboard_ptr_->self_pose_.y - blackboard_ptr_->patrol_pos[i][1];
                double dis = std::sqrt((delta_x * delta_x) + (delta_y * delta_y));
                if (dis < distance)
                { //搜寻最近的巡逻点用于躲避buff区
                    distance = dis;
                    index = i;
                }
            }
            if (blackboard_ptr_->teammate_pose_.x != 0 && blackboard_ptr_->teammate_pose_.y != 0)
            { //判断队友的位置
                double delta_x = blackboard_ptr_->teammate_pose_.x - blackboard_ptr_->patrol_pos[index][0];
                double delta_y = blackboard_ptr_->teammate_pose_.y - blackboard_ptr_->patrol_pos[index][1];
                double dis = std::sqrt((delta_x * delta_x) + (delta_y * delta_y));
                if (dis < 0.2)
                {               // 与当前巡逻点距离过小
                    index += 1; //选择下一个巡逻点
                    if (index >= 6)
                        index = 0; //溢出处理
                }
            }
            chassis_exe_ptr_->Goto(blackboard_ptr_->patrol_pos[index][0], blackboard_ptr_->patrol_pos[index][1],
                                   blackboard_ptr_->patrol_pos[index][2], false);
        }
        std::stringstream str;
    };
}

#endif