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
            // 比赛刚开始，进行抢buff操作
            // std::cout << "add_bullet" <<std::endl;
            if(blackboard_ptr_->game_buff_.self_bullet_buff_status_ == false)    go_buff_time_ = 0;
            if(blackboard_ptr_->game_status_.remaining_time >= 170)
            {
                // 先把自己和敌方加血区设为活动区
                buff_exe_ptr_->set_buff(blackboard_ptr_->game_buff_.self_bullet_buff_id_, false);
                buff_exe_ptr_->set_buff(blackboard_ptr_->game_buff_.enemy_blood_buff_id_, false);
                blackboard_ptr_->buff_active_[blackboard_ptr_->game_buff_.self_bullet_buff_id_] = false;
                blackboard_ptr_->buff_active_[blackboard_ptr_->game_buff_.enemy_blood_buff_id_] = false;
                // 分情况讨论
                if(blackboard_ptr_->self_id_ == 1)//红1， 是否加弹的条件
                {
                    if((blackboard_ptr_->game_buff_.self_bullet_buff_id_ == 0 || 
                        blackboard_ptr_->game_buff_.self_bullet_buff_id_ == 1 || 
                        blackboard_ptr_->game_buff_.self_bullet_buff_id_ == 3) && 
                        blackboard_ptr_->game_buff_.self_bullet_buff_status_ &&
                       blackboard_ptr_->check_block(blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.self_bullet_buff_id_].x, blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.self_bullet_buff_id_].y) == false)
                    {
                        Go2Buff(blackboard_ptr_->game_buff_.self_bullet_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    if(blackboard_ptr_->game_buff_.self_bullet_buff_id_ == 2 && 
                       (blackboard_ptr_->game_buff_.enemy_bullet_buff_id_ == 4 ||
                        blackboard_ptr_->game_buff_.no_shoot_id_ == 4 || 
                        blackboard_ptr_->game_buff_.no_move_id_ == 4) && 
                        blackboard_ptr_->game_buff_.self_bullet_buff_status_ &&
                       blackboard_ptr_->check_block(blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.self_bullet_buff_id_].x, blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.self_bullet_buff_id_].y) == false)
                    {
                        Go2Buff(blackboard_ptr_->game_buff_.self_bullet_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    if( blackboard_ptr_->game_buff_.enemy_blood_buff_id_ == 3 &&
                        blackboard_ptr_->game_buff_.enemy_blood_buff_status_ &&
                        (blackboard_ptr_->game_buff_.self_bullet_buff_id_ == 4 ||
                         blackboard_ptr_->game_buff_.self_bullet_buff_id_ == 5 ||
                        (blackboard_ptr_->game_buff_.self_bullet_buff_id_ == 2 && 
                        (blackboard_ptr_->game_buff_.enemy_blood_buff_id_ == 4 || 
                        blackboard_ptr_->game_buff_.self_blood_buff_id_ == 4))) &&
                        blackboard_ptr_->game_buff_.self_bullet_buff_status_ &&
                        blackboard_ptr_->check_block(blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.enemy_blood_buff_id_].x, blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.enemy_blood_buff_id_].y) == false)
                    {
                        Go2Buff(blackboard_ptr_->game_buff_.enemy_blood_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                }
                else if(blackboard_ptr_->self_id_ == 2)//红2
                {
                    if((blackboard_ptr_->game_buff_.self_bullet_buff_id_ == 4 || 
                       blackboard_ptr_->game_buff_.self_bullet_buff_id_ == 5) && 
                       blackboard_ptr_->game_buff_.self_bullet_buff_status_ &&
                       blackboard_ptr_->check_block(blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.self_bullet_buff_id_].x, blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.self_bullet_buff_id_].y) == false)
                    {
                        Go2Buff(blackboard_ptr_->game_buff_.self_bullet_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    if(blackboard_ptr_->game_buff_.self_bullet_buff_id_ == 2 && 
                       (blackboard_ptr_->game_buff_.enemy_blood_buff_id_ == 4 || 
                        blackboard_ptr_->game_buff_.self_blood_buff_id_ == 4) && 
                        blackboard_ptr_->game_buff_.self_bullet_buff_status_ &&
                        blackboard_ptr_->check_block(blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.self_bullet_buff_id_].x, blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.self_bullet_buff_id_].y) == false)
                    {
                        Go2Buff(blackboard_ptr_->game_buff_.self_bullet_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    // 如果敌方加血在5 6，先去刷掉，再进入通用阻挡位置
                    if(blackboard_ptr_->game_buff_.enemy_blood_buff_status_ && 
                       (blackboard_ptr_->game_buff_.enemy_blood_buff_id_ == 4 || 
                       blackboard_ptr_->game_buff_.enemy_blood_buff_id_ == 5) &&
                       blackboard_ptr_->check_block(blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.enemy_blood_buff_id_].x, blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.enemy_blood_buff_id_].y) == false)
                    {
                        Go2Buff(blackboard_ptr_->game_buff_.enemy_blood_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    if(blackboard_ptr_->game_buff_.enemy_bullet_buff_status_ && 
                       blackboard_ptr_->game_buff_.enemy_bullet_buff_id_ == 5)
                    {
                        if(blackboard_ptr_->check_block(1.35, 2.85) == false)
                        {
                            double dis = std::sqrt(pow(blackboard_ptr_->self_pose_.x-1.35,2) + pow(blackboard_ptr_->self_pose_.y-2.85,2));
                            if(dis < 0.2) chassis_exe_ptr_->Spin();
                            else chassis_exe_ptr_->Goto(1.35, 2.85, 0, false);
                            return BehaviorState::SUCCESS;
                        }
                    }
                    if(blackboard_ptr_->team_info_.attack_enemy != 0 && blackboard_ptr_->team_info_.received_)   return BehaviorState::FAILURE;
                    // 在未识别到敌方的其他情况蹲门口进行进攻
                    if(blackboard_ptr_->armor_detection_.detected_enemy == false)
                    {
                        if(blackboard_ptr_->check_block(1.40, 1.65) == false)
                        {
                            double dis = std::sqrt(pow(blackboard_ptr_->self_pose_.x-1.40,2) + pow(blackboard_ptr_->self_pose_.y-1.65,2));
                            if(dis < 0.2) chassis_exe_ptr_->Spin();
                            else chassis_exe_ptr_->Goto(1.40, 1.65, 0, false);
                            return BehaviorState::SUCCESS;
                        }
                    }
                }
                else if(blackboard_ptr_->self_id_ == 101)//蓝1
                {
                    // 蓝1
                    // 如果是3 5 6是加弹，直接去，因为蓝1有弹
                    if((blackboard_ptr_->game_buff_.self_bullet_buff_id_ == 2 || 
                        blackboard_ptr_->game_buff_.self_bullet_buff_id_ == 4 || 
                        blackboard_ptr_->game_buff_.self_bullet_buff_id_ == 5) && 
                        blackboard_ptr_->game_buff_.self_bullet_buff_status_ &&
                        blackboard_ptr_->check_block(blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.self_bullet_buff_id_].x, blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.self_bullet_buff_id_].y) == false)
                    {
                        Go2Buff(blackboard_ptr_->game_buff_.self_bullet_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    // 如果是4加弹，如果2是惩罚区或敌方加弹，蓝1去加弹
                    if(blackboard_ptr_->game_buff_.self_bullet_buff_id_ == 3 && 
                       (blackboard_ptr_->game_buff_.enemy_bullet_buff_id_ == 1 ||
                        blackboard_ptr_->game_buff_.no_shoot_id_ == 1 || 
                        blackboard_ptr_->game_buff_.no_move_id_ == 1) && 
                        blackboard_ptr_->game_buff_.self_bullet_buff_status_ &&
                        blackboard_ptr_->check_block(blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.self_bullet_buff_id_].x, blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.self_bullet_buff_id_].y) == false)
                    {
                        Go2Buff(blackboard_ptr_->game_buff_.self_bullet_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    if( blackboard_ptr_->game_buff_.enemy_blood_buff_id_ == 2 &&
                        blackboard_ptr_->game_buff_.enemy_blood_buff_status_ &&
                        (blackboard_ptr_->game_buff_.self_bullet_buff_id_ == 0 ||
                         blackboard_ptr_->game_buff_.self_bullet_buff_id_ == 1 ||
                         (blackboard_ptr_->game_buff_.self_bullet_buff_id_ == 3 && 
                         (blackboard_ptr_->game_buff_.enemy_blood_buff_id_ == 1 || 
                        blackboard_ptr_->game_buff_.self_blood_buff_id_ == 1))) &&
                        blackboard_ptr_->game_buff_.self_bullet_buff_status_ &&
                        blackboard_ptr_->check_block(blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.enemy_blood_buff_id_].x, blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.enemy_blood_buff_id_].y) == false)
                    {
                        Go2Buff(blackboard_ptr_->game_buff_.enemy_blood_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                }
                else if(blackboard_ptr_->self_id_ == 102)//蓝2
                {
                    // 蓝2
                    // 如果是1 2是加弹，直接去，因为近
                    if((blackboard_ptr_->game_buff_.self_bullet_buff_id_ == 0 || 
                       blackboard_ptr_->game_buff_.self_bullet_buff_id_ == 1) &&
                       blackboard_ptr_->game_buff_.self_bullet_buff_status_ &&
                       blackboard_ptr_->check_block(blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.self_bullet_buff_id_].x, blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.self_bullet_buff_id_].y) == false)
                    {
                        Go2Buff(blackboard_ptr_->game_buff_.self_bullet_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    // 如果是4加弹，如果2不是惩罚区或敌方加弹，蓝2去加弹
                    if(blackboard_ptr_->game_buff_.self_bullet_buff_id_ == 3 && 
                       (blackboard_ptr_->game_buff_.self_blood_buff_id_ == 1 ||
                       blackboard_ptr_->game_buff_.enemy_blood_buff_id_ == 1) &&
                       blackboard_ptr_->game_buff_.self_bullet_buff_status_ &&
                       blackboard_ptr_->check_block(blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.self_bullet_buff_id_].x, blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.self_bullet_buff_id_].y) == false)
                    {
                        Go2Buff(blackboard_ptr_->game_buff_.self_bullet_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    // 如果敌方加血在1 2，先去刷掉，再进入通用阻挡位置
                    if(blackboard_ptr_->game_buff_.enemy_blood_buff_status_ && 
                       (blackboard_ptr_->game_buff_.enemy_blood_buff_id_ == 0 || 
                       blackboard_ptr_->game_buff_.enemy_blood_buff_id_ == 1) &&
                       blackboard_ptr_->check_block(blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.enemy_blood_buff_id_].x, blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.enemy_blood_buff_id_].y) == false)
                    {
                        Go2Buff(blackboard_ptr_->game_buff_.enemy_blood_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    // 如果敌方加弹区在我们家，则进行阻挡
                    if(blackboard_ptr_->game_buff_.enemy_bullet_buff_status_ && 
                       blackboard_ptr_->game_buff_.enemy_bullet_buff_id_ == 0)
                    {
                        if(blackboard_ptr_->check_block(6.73, 1.63) == false)
                        {
                            double dis = std::sqrt(pow(blackboard_ptr_->self_pose_.x-6.73,2) + pow(blackboard_ptr_->self_pose_.y-1.63,2));
                            if(dis < 0.2) chassis_exe_ptr_->Spin();
                            else chassis_exe_ptr_->Goto(6.73, 1.63, 3.14, false);
                            return BehaviorState::SUCCESS;
                        }
                    }
                    if(blackboard_ptr_->team_info_.attack_enemy != 0)   return BehaviorState::FAILURE;
                    // 在未识别到敌方的其他情况蹲门口进行进攻
                    if(blackboard_ptr_->armor_detection_.detected_enemy == false)
                    {
                        if(blackboard_ptr_->check_block(6.68, 2.83) == false)
                        {
                            double dis = std::sqrt(pow(blackboard_ptr_->self_pose_.x-6.68,2) + pow(blackboard_ptr_->self_pose_.y-2.83,2));
                            if(dis < 0.2) chassis_exe_ptr_->Spin();
                            else chassis_exe_ptr_->Goto(6.68, 2.83, 3.14, false);
                            return BehaviorState::SUCCESS;
                        }
                    }
                }
            }
            if(blackboard_ptr_->game_status_.remaining_time < 170 && blackboard_ptr_->game_buff_.self_bullet_buff_status_){
                // 如果子弹数少于10并且队友没有去加弹
                if(blackboard_ptr_->game_buff_.remain_bullets_num_ < blackboard_ptr_->force_add_bullet_ && 
                   blackboard_ptr_->team_info_.to_buff != blackboard_ptr_->game_buff_.self_bullet_buff_id_){
                    go_buff_time_++;
                    if(blackboard_ptr_->check_block(blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.self_blood_buff_id_].x, blackboard_ptr_->buff_pos_[blackboard_ptr_->game_buff_.self_blood_buff_id_].y) == false)
                    {
                        go_buff_time_++;
                        if(go_buff_time_ <= blackboard_ptr_->loop_rate_*blackboard_ptr_->go_buff_time_){ // 设置buff超时时间
                            buff_exe_ptr_->set_buff(blackboard_ptr_->game_buff_.self_blood_buff_id_, false);
                            Go2Buff(blackboard_ptr_->game_buff_.self_bullet_buff_id_);
                            return BehaviorState::SUCCESS;
                        }
                        else if(go_buff_time_ >= blackboard_ptr_->loop_rate_*blackboard_ptr_->go_buff_time_*2) go_buff_time_ = 0;
                    }
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
            blackboard_ptr_->buff_active_[id] = false;
            self_info_exe_ptr_->Goto_Buff(id);
            double angle;
            if(blackboard_ptr_->self_pose_.x < blackboard_ptr_->buff_pos_[id].x)
                angle = 0;
            else
                angle = 3.14;
            if(id == 5) angle = 0;
            if(id == 0) angle = 3.14;            
            chassis_exe_ptr_->Goto(blackboard_ptr_->buff_pos_[id].x, blackboard_ptr_->buff_pos_[id].y, angle, false);
        }
        int go_buff_time_ = 0;
        std::stringstream str;
    };
}

#endif
