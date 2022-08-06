#ifndef ADD_BULLET_H
#define ADD_BULLET_H

#include "behavior_tree.h"
#include "self_info_executor.hpp"
#include "buff_executor.hpp"
#include "log_executor.hpp"

namespace robot_decision
{
    class AddBulletBehavior : public ActionNode
    {
        public:
        AddBulletBehavior(std::string name, 
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
            if(blackboard_ptr_->game_status_.remaining_time >= 170)
            {
                buff_exe_ptr_->set_buff(blackboard_ptr_->self_blood_buff_id_, false);
                buff_exe_ptr_->set_buff(blackboard_ptr_->enemy_blood_buff_id_, false);
                if(blackboard_ptr_->robot_status_.id == 1)
                {
                    if((blackboard_ptr_->self_bullet_buff_id_ == 0 || 
                        blackboard_ptr_->self_bullet_buff_id_ == 1 || 
                        blackboard_ptr_->self_bullet_buff_id_ == 3) && 
                        blackboard_ptr_->self_bullet_buff_status_)
                    {
                        if(blackboard_ptr_->self_bullet_buff_id_ == 0 && blackboard_ptr_->no_shoot_id_ == 1 &&
                           blackboard_ptr_->game_status_.remaining_time <= 175){
                            buff_exe_ptr_->set_buff(blackboard_ptr_->no_shoot_id_, false);
                        }
                        Go2Buff(blackboard_ptr_->self_bullet_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    if(blackboard_ptr_->self_bullet_buff_id_ == 2 && 
                       (blackboard_ptr_->game_buff_.zone[4].type == 4 ||
                        blackboard_ptr_->game_buff_.zone[4].type == 5 || 
                        blackboard_ptr_->game_buff_.zone[4].type == 6) && 
                        blackboard_ptr_->self_bullet_buff_status_)
                    {
                        Go2Buff(blackboard_ptr_->self_bullet_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                }
                else if(blackboard_ptr_->robot_status_.id == 2)
                {
                    if(blackboard_ptr_->self_bullet_buff_id_ == 4 || 
                       blackboard_ptr_->self_bullet_buff_id_ == 5 && 
                       blackboard_ptr_->self_bullet_buff_status_)
                    {
                        Go2Buff(blackboard_ptr_->self_bullet_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    if(blackboard_ptr_->self_bullet_buff_id_ == 2 && 
                       (blackboard_ptr_->game_buff_.zone[4].type == 3 || 
                        blackboard_ptr_->game_buff_.zone[4].type == 1) && 
                       blackboard_ptr_->self_bullet_buff_status_)
                    {
                        Go2Buff(blackboard_ptr_->self_bullet_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    if(blackboard_ptr_->enemy_bullet_buff_status_ && 
                       blackboard_ptr_->enemy_bullet_buff_id_ == 5)
                    {
                        double dis = std::sqrt(pow(blackboard_ptr_->self_pose_.x-6.73,2) + pow(blackboard_ptr_->self_pose_.y-1.63,2));
                        if(dis < 0.2) chassis_exe_ptr_->Spin();
                        else chassis_exe_ptr_->Goto(6.73, 1.63, 0, false);
                        return BehaviorState::SUCCESS;
                    }
                    if(blackboard_ptr_->enemy_blood_buff_status_ && 
                       (blackboard_ptr_->enemy_blood_buff_id_ == 4 || 
                       blackboard_ptr_->enemy_blood_buff_id_ == 5))
                    {
                        Go2Buff(blackboard_ptr_->enemy_blood_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    if(blackboard_ptr_->armor_detection_.detected_enemy == false){
                        double dis = std::sqrt(pow(blackboard_ptr_->self_pose_.x-6.68,2) + pow(blackboard_ptr_->self_pose_.y-2.83,2));
                        if(dis < 0.2) chassis_exe_ptr_->Spin();
                        else chassis_exe_ptr_->Goto(6.68, 2.83, 0, false);
                        return BehaviorState::SUCCESS;
                    }
                }
                else if(blackboard_ptr_->robot_status_.id == 101)
                {
                    if((blackboard_ptr_->self_bullet_buff_id_ == 2 || 
                        blackboard_ptr_->self_bullet_buff_id_ == 4 || 
                        blackboard_ptr_->self_bullet_buff_id_ == 5) && 
                        blackboard_ptr_->self_bullet_buff_status_)
                    {
                        if(blackboard_ptr_->self_bullet_buff_id_ == 5 && blackboard_ptr_->no_shoot_id_ == 4 &&
                           blackboard_ptr_->game_status_.remaining_time <= 175){
                            buff_exe_ptr_->set_buff(blackboard_ptr_->no_shoot_id_, false);
                        }
                        Go2Buff(blackboard_ptr_->self_bullet_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    if(blackboard_ptr_->self_bullet_buff_id_ == 3 && 
                       (blackboard_ptr_->game_buff_.zone[1].type == 2 ||
                        blackboard_ptr_->game_buff_.zone[1].type == 5 || 
                        blackboard_ptr_->game_buff_.zone[1].type == 6) && 
                        blackboard_ptr_->self_bullet_buff_status_)
                    {
                        Go2Buff(blackboard_ptr_->self_bullet_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                }
                else if(blackboard_ptr_->robot_status_.id == 102)
                {
                    if((blackboard_ptr_->self_bullet_buff_id_ == 0 || 
                       blackboard_ptr_->self_bullet_buff_id_ == 1) &&
                       blackboard_ptr_->self_bullet_buff_status_)
                    {
                        Go2Buff(blackboard_ptr_->self_bullet_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    if(blackboard_ptr_->self_bullet_buff_id_ == 3 && 
                       (blackboard_ptr_->game_buff_.zone[1].type == 1 ||
                       blackboard_ptr_->game_buff_.zone[1].type == 3) &&
                       blackboard_ptr_->self_bullet_buff_status_)
                    {
                        Go2Buff(blackboard_ptr_->self_bullet_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    if(blackboard_ptr_->enemy_bullet_buff_status_ && 
                       blackboard_ptr_->enemy_bullet_buff_id_ == 0)
                    {
                        double dis = std::sqrt(pow(blackboard_ptr_->self_pose_.x-1.35,2) + pow(blackboard_ptr_->self_pose_.y-2.85,2));
                        if(dis < 0.2) chassis_exe_ptr_->Spin();
                        else chassis_exe_ptr_->Goto(1.35, 2.85, 0, false);
                        return BehaviorState::SUCCESS;
                    }
                    if(blackboard_ptr_->enemy_blood_buff_status_ && 
                       (blackboard_ptr_->enemy_blood_buff_id_ == 0 || 
                       blackboard_ptr_->enemy_blood_buff_id_ == 1))
                    {
                        Go2Buff(blackboard_ptr_->enemy_blood_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    if(blackboard_ptr_->armor_detection_.detected_enemy == false){
                        double dis = std::sqrt(pow(blackboard_ptr_->self_pose_.x-1.4,2) + pow(blackboard_ptr_->self_pose_.y-1.65,2));
                        if(dis < 0.2) chassis_exe_ptr_->Spin();
                        else chassis_exe_ptr_->Goto(1.40, 1.65, 0, false);
                        return BehaviorState::SUCCESS;
                    }
                }
            }
            if(blackboard_ptr_->game_status_.remaining_time < 170 && blackboard_ptr_->self_bullet_buff_status_){
                if(blackboard_ptr_->remain_bullets_num_ < blackboard_ptr_->force_add_bullet_ && 
                   blackboard_ptr_->team_info_.to_buff != blackboard_ptr_->self_bullet_buff_id_){
                    go_buff_time_++;
                    if(go_buff_time_ <= blackboard_ptr_->loop_rate_*blackboard_ptr_->go_buff_time_){
                        buff_exe_ptr_->set_buff(blackboard_ptr_->self_blood_buff_id_, false);
                        Go2Buff(blackboard_ptr_->self_bullet_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    else if(go_buff_time_ >= blackboard_ptr_->loop_rate_*blackboard_ptr_->go_buff_time_*2) go_buff_time_ = 0;
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
