#ifndef ADD_BLOOG_H
#define ADD_BLOOD_H

#include "behavior_tree.h"
#include "self_info_executor.hpp"
#include "buff_executor.hpp"
#include "log_executor.hpp"

namespace robot_decision
{
    class AddBloodBehavior : public ActionNode
    {
        public:
        AddBloodBehavior(std::string name, 
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
            self_info_exe_ptr_->set_self_pos(blackboard_ptr_->self_pose_.x, blackboard_ptr_->self_pose_.y);
            if(blackboard_ptr_->self_blood_buff_status_ && blackboard_ptr_->team_info_.to_buff != blackboard_ptr_->self_blood_buff_id_){
                if(blackboard_ptr_->self_hp_ < blackboard_ptr_->robot_status_.max_hp * blackboard_ptr_->force_add_blood_){
                    go_buff_time_++;
                    if(go_buff_time_ <= blackboard_ptr_->loop_rate_*blackboard_ptr_->go_buff_time_){
                        Go2Buff(blackboard_ptr_->self_blood_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    else if(go_buff_time_ > blackboard_ptr_->loop_rate_*blackboard_ptr_->go_buff_time_*2) go_buff_time_ = 0;
                }
                else go_buff_time_ = 0;
            }
            else go_buff_time_ = 0;
            return BehaviorState::FAILURE;
        }
        private:
        void Go2Buff(int id)
        {
            buff_exe_ptr_->set_buff(id, false);
            self_info_exe_ptr_->Goto_Buff(id);
            double angle;
            if(blackboard_ptr_->self_pose_.x < blackboard_ptr_->buff_pos_[id].x)
                angle = 0;
            else
                angle = 3.14;
            if(id == 1) angle = 0;
            if(id == 4) angle = 3.14;
            chassis_exe_ptr_->Goto(blackboard_ptr_->buff_pos_[id].x, blackboard_ptr_->buff_pos_[id].y, angle, false);
        }
        int go_buff_time_ = 0;
        std::stringstream str;
    };
}

#endif