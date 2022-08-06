#ifndef RETREAT_H
#define RETREAT_H

#include "behavior_tree.h"
#include "self_info_executor.hpp"
#include "buff_executor.hpp"
#include "log_executor.hpp"

namespace robot_decision
{
    class RetreatBehavior : public ActionNode
    {
        public:
        RetreatBehavior(std::string name, 
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
            if(blackboard_ptr_->self_hp_ < blackboard_ptr_->robot_status_.max_hp * 0.2 || 
               (blackboard_ptr_->remain_bullets_num_ < blackboard_ptr_->force_add_bullet_ && 
               blackboard_ptr_->self_bullet_buff_status_ == false))
            {
                std::vector<cv::Point2d> retreat_points;
                double retreat_x[4] = {0.8, 0.8, 7.28, 7.28};
                double retreat_y[4] = {0.8, 3.98, 0.5, 3.68};
                for(int i = 0; i < 4; i++){
                    double dis_1 = 0.0, dis_2 = 0.0, dis_3 = 0.0, dis_4 = 0.0, dis_5 = 0.0, pos_x = 0.0, pos_y = 0.0;
                    if(blackboard_ptr_->team_info_received_){
                        dis_1 = std::sqrt(std::pow(blackboard_ptr_->teammate_pose_.x - retreat_x[i], 2) + 
                                          std::pow(blackboard_ptr_->teammate_pose_.y - retreat_y[i], 2));
                    }
                    else dis_1 = 100;

                    if(blackboard_ptr_->sentry_pose_received_){
                        pos_x = blackboard_ptr_->sentry_enemy_pose_[0].x;
                        pos_y = blackboard_ptr_->sentry_enemy_pose_[0].y;
                        if(pos_x > 0 && pos_x < 8.08 && pos_y > 0 && pos_y < 4.48)
                            dis_2 = std::sqrt(std::pow(pos_x - retreat_x[i], 2) + std::pow(pos_y - retreat_y[i], 2));
                        else dis_2 = 100;
                        pos_x = blackboard_ptr_->sentry_enemy_pose_[1].x;
                        pos_y = blackboard_ptr_->sentry_enemy_pose_[1].y;
                        if(pos_x > 0 && pos_x < 8.08 && pos_y > 0 && pos_y < 4.48)
                            dis_3 = std::sqrt(std::pow(pos_x - retreat_x[i], 2) + std::pow(pos_y - retreat_y[i], 2));
                        else dis_3 = 100;
                        pos_x = blackboard_ptr_->sentry_die_pose[0].x;
                        pos_y = blackboard_ptr_->sentry_die_pose[0].y;
                        if(pos_x > 0 && pos_x < 8.08 && pos_y > 0 && pos_y < 4.48)
                            dis_4 = std::sqrt(std::pow(pos_x - retreat_x[i], 2) + std::pow(pos_y - retreat_y[i], 2));
                        else dis_4 = 100;
                        pos_x = blackboard_ptr_->sentry_die_pose[1].x;
                        pos_y = blackboard_ptr_->sentry_die_pose[1].y;
                        if(pos_x > 0 && pos_x < 8.08 && pos_y > 0 && pos_y < 4.48)
                            dis_5 = std::sqrt(std::pow(pos_x - retreat_x[i], 2) + std::pow(pos_y - retreat_y[i], 2));
                        else dis_5 = 100;
                    }
                    else dis_2 = dis_3 = dis_4 = dis_5 = 100;
                    
                    if(dis_1 > 3.0 && dis_2 > 3.0 && dis_3 > 3.0 && dis_4 > 3.0 && dis_5 > 3.0)
                        retreat_points.push_back(cv::Point2d(retreat_x[i], retreat_y[i]));
                }
                if(retreat_points.size() == 0) return BehaviorState::FAILURE;
                double min_dis = 100;
                cv::Point2d final_retreat_point;
                for(int i = 0; i < retreat_points.size(); i++){
                    double dis = std::sqrt(std::pow(blackboard_ptr_->self_pose_.x-retreat_points[i].x, 2)+
                                           std::pow(blackboard_ptr_->self_pose_.y-retreat_points[i].y, 2));
                    if(dis < min_dis){
                        min_dis = dis;
                        final_retreat_point = retreat_points[i];
                    }
                }
                double angle = atan2(blackboard_ptr_->self_pose_.y-final_retreat_point.y, blackboard_ptr_->self_pose_.x-final_retreat_point.x);
                chassis_exe_ptr_->Goto(final_retreat_point.x, final_retreat_point.y, angle, true);
                return BehaviorState::SUCCESS;
            }
            return BehaviorState::FAILURE;
        }
    };
}

#endif