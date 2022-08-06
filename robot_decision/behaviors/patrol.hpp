#ifndef PATROL_H
#define PATROL_H

#include "behavior_tree.h"
#include "self_info_executor.hpp"
#include "buff_executor.hpp"
#include "log_executor.hpp"

namespace robot_decision
{
    class PatrolBehavior : public ActionNode
    {
        public:
        PatrolBehavior(std::string name, 
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
            if(blackboard_ptr_->self_bullet_buff_status_ && blackboard_ptr_->team_info_.to_buff != blackboard_ptr_->self_bullet_buff_id_)
            {
                go_bullet_buff_time_++;
                if(go_bullet_buff_time_ <= blackboard_ptr_->loop_rate_*blackboard_ptr_->go_buff_time_){
                    buff_exe_ptr_->set_buff(blackboard_ptr_->self_blood_buff_id_, false);
                    Go2Buff(blackboard_ptr_->self_bullet_buff_id_);
                    return BehaviorState::SUCCESS;
                }
                else if(go_bullet_buff_time_ >= blackboard_ptr_->loop_rate_*blackboard_ptr_->go_buff_time_*2) go_bullet_buff_time_ = 0;
            }
            else go_bullet_buff_time_ = 0;
            if(blackboard_ptr_->self_blood_buff_status_ && blackboard_ptr_->team_info_.to_buff != blackboard_ptr_->self_blood_buff_id_)
            {
                if(blackboard_ptr_->self_hp_ < blackboard_ptr_->robot_status_.max_hp * 0.9){
                    go_blood_buff_time_++;
                    if(go_blood_buff_time_ <= blackboard_ptr_->loop_rate_*blackboard_ptr_->go_buff_time_){
                        Go2Buff(blackboard_ptr_->self_blood_buff_id_);
                        return BehaviorState::SUCCESS;
                    }
                    else if(go_blood_buff_time_ >= blackboard_ptr_->loop_rate_*blackboard_ptr_->go_buff_time_*2) go_blood_buff_time_ = 0;
                }
                else go_blood_buff_time_ = 0;
            }
            else go_blood_buff_time_ = 0;
            bool spin_flag = false;
            double delta_x = blackboard_ptr_->self_pose_.x-blackboard_ptr_->patrol_pos[blackboard_ptr_->last_patrol_point_][0];
            double delta_y = blackboard_ptr_->self_pose_.y-blackboard_ptr_->patrol_pos[blackboard_ptr_->last_patrol_point_][1];
            double distance = std::sqrt((delta_x*delta_x) + (delta_y*delta_y));
            if(blackboard_ptr_->last_patrol_point_ < 6 && blackboard_ptr_->last_patrol_point_ >= 0){
                if(distance < 0.3) patrol_time_++;
                else patrol_time_ = 0;
            }
            else patrol_time_ = 100;
            if(patrol_time_ >= 2 && patrol_time_ < blackboard_ptr_->loop_rate_ * 5){
                spin_flag = true;
            }
            if(patrol_time_ >= blackboard_ptr_->loop_rate_ * 5){
                patrol_time_ = 0;
                GoPatrol();
            }
            if(spin_flag){
                chassis_exe_ptr_->Spin();
                return BehaviorState::SUCCESS;
            }
            chassis_exe_ptr_->Goto(blackboard_ptr_->patrol_pos[index_][0], blackboard_ptr_->patrol_pos[index_][1], 
                                   blackboard_ptr_->patrol_pos[index_][2], false);
            return BehaviorState::SUCCESS;
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
        void GoPatrol()
        {
            int index = 0;
            double distance = 100.0;
            for(int i = 0; i < sizeof(blackboard_ptr_->patrol_pos)/sizeof(blackboard_ptr_->patrol_pos[0]); i++){
                double delta_x = blackboard_ptr_->self_pose_.x-blackboard_ptr_->patrol_pos[i][0];
                double delta_y = blackboard_ptr_->self_pose_.y-blackboard_ptr_->patrol_pos[i][1];
                double dis = std::sqrt((delta_x*delta_x)+(delta_y*delta_y));
                if(dis < distance){
                    distance = dis;
                    index = i;
                }
            }
            if(distance < 0.5){
                index += 1;
                if(index >= 6) index = 0;
            }
            if(blackboard_ptr_->team_info_received_){
                if(blackboard_ptr_->teammate_pose_.x != 0 && blackboard_ptr_->teammate_pose_.y != 0){
                    double delta_x = blackboard_ptr_->teammate_pose_.x-blackboard_ptr_->patrol_pos[index][0];
                    double delta_y = blackboard_ptr_->teammate_pose_.y-blackboard_ptr_->patrol_pos[index][1];
                    double dis = std::sqrt((delta_x*delta_x)+(delta_y*delta_y));
                    if(distance < 0.8){
                        index += 1;
                        if(index >= 6) index = 0;
                    }
                }
            }
            blackboard_ptr_->last_patrol_point_ = index;
            index_ = index;
        }
        cv::Point3f patrol_pos_;
        int go_blood_buff_time_;
        int go_bullet_buff_time_;
        int patrol_time_;
        int index_;
        std::stringstream str;
    };
}

#endif