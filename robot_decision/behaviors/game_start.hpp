#ifndef GAME_START_H
#define GAME_START_H

#include "behavior_tree.h"
#include "self_info_executor.hpp"
#include "buff_executor.hpp"
#include "log_executor.hpp"

namespace robot_decision
{
    class GameStartBehavior : public ActionNode
    {
        public:
        GameStartBehavior(std::string name, 
                          int level, 
                          const Blackboard::Ptr &blackboard_ptr, 
                          Chassis_executor::Ptr &chassis_exe_ptr, 
                          Log_executor::Ptr &log_exe_ptr,
                          Buff_executor::Ptr &buff_exe_ptr,
                          Self_Info_executor::Ptr &self_info_exe_ptr):
            ActionNode::ActionNode(name, level, blackboard_ptr, chassis_exe_ptr, log_exe_ptr, buff_exe_ptr, self_info_exe_ptr)
        {}
        BehaviorState Update()
        {
            if(blackboard_ptr_->game_status_.game_status == 4 && blackboard_ptr_->game_status_.remaining_time > 0)
            {
                if(blackboard_ptr_->game_status_.remaining_time <= 170 && blackboard_ptr_->global_map_.received_)
                {
                    if(blackboard_ptr_->global_map_.trust_ == true && blackboard_ptr_->global_map_trust_ == true) blackboard_ptr_->global_map_trust_ = true;
                    else
                    {
                        blackboard_ptr_->global_map_trust_ = false;
                    }   
                }
                // game start
                if(blackboard_ptr_->game_robot_HP_.self_hp_ <= 0) 
                {
                    if(blackboard_ptr_->game_status_.remaining_time >= 170)
                        return BehaviorState::FAILURE;
                    else
                        return  BehaviorState::SUCCESS;
                }
                // if(blackboard_ptr_->planning_info_.received_ && blackboard_ptr_->planning_info_.flag == true)
                // {
                //     chassis_exe_ptr_->IDLE();
                // }
                std::cout << std::endl;
                return BehaviorState::FAILURE;//FAILURE时才继续执行后面的决策
            }
            else
            {
                // std::cout << "game_not_start"<<std::endl;
                chassis_exe_ptr_->Stop();
                return BehaviorState::SUCCESS;
            }
        }
    };
}

#endif