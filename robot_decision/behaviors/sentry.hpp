#ifndef SENTRY_H
#define SENTRY_H

#include "behavior_tree.h"
#include "self_info_executor.hpp"
#include "buff_executor.hpp"
#include "log_executor.hpp"

namespace robot_decision
{
    class SentryBehavior : public ActionNode
    {
        public:
        SentryBehavior(std::string name, 
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
            if(blackboard_ptr_->sentry_pose_received_)
            {
                double distance_1 = 100, distance_2 = 100;
                if(blackboard_ptr_->sentry_enemy_pose_[0].x != 0. && blackboard_ptr_->sentry_enemy_pose_[0].y != 0.)
                    distance_1 = std::sqrt(std::pow(blackboard_ptr_->sentry_enemy_pose_[0].x-blackboard_ptr_->self_pose_.x, 2) + 
                                           std::pow(blackboard_ptr_->sentry_enemy_pose_[0].y-blackboard_ptr_->self_pose_.y, 2));
                if(blackboard_ptr_->sentry_enemy_pose_[1].x != 0. && blackboard_ptr_->sentry_enemy_pose_[1].y != 0.)
                    distance_2 = std::sqrt(std::pow(blackboard_ptr_->sentry_enemy_pose_[1].x-blackboard_ptr_->self_pose_.x, 2) + 
                                           std::pow(blackboard_ptr_->sentry_enemy_pose_[1].y-blackboard_ptr_->self_pose_.y, 2));
                if(distance_1 != distance_2)
                {
                    double enemy_pos_x, enemy_pos_y, distance;
                    distance = std::min(distance_1, distance_2);
                    if(distance_1 < distance_2){
                        enemy_pos_x = blackboard_ptr_->sentry_enemy_pose_[0].x;
                        enemy_pos_y = blackboard_ptr_->sentry_enemy_pose_[0].y;
                    }
                    else{
                        enemy_pos_x = blackboard_ptr_->sentry_enemy_pose_[1].x;
                        enemy_pos_y = blackboard_ptr_->sentry_enemy_pose_[1].y;
                    }
                    cv::Point3d fight_pose;
                    bool force_plan = false;
                    if(distance < blackboard_ptr_->back_distance_){
                        fight_pose = calc_fight_pose(enemy_pos_x, enemy_pos_y);
                        if(fight_pose.x != 0. && fight_pose.y != 0.){
                            double dis = std::sqrt((last_fight_pose_.x-fight_pose.x)*(last_fight_pose_.x-fight_pose.x) + 
                                                (last_fight_pose_.y-fight_pose.y)*(last_fight_pose_.y-fight_pose.y));
                            if(dis > 0.2){
                                force_plan = true;
                                last_fight_pose_ = fight_pose;
                            }
                            chassis_exe_ptr_->GotoSwing(fight_pose.x, fight_pose.y, fight_pose.z, force_plan);
                        }
                    }
                    else if(distance > blackboard_ptr_->attack_distance_){
                        fight_pose = calc_fight_pose(enemy_pos_x, enemy_pos_y);
                        if(fight_pose.x != 0. && fight_pose.y != 0.){
                            if(distance > blackboard_ptr_->force_attack_distance_) 
                                force_plan = true;
                            double dis = std::sqrt((fight_pose.x - blackboard_ptr_->self_pose_.x)*(fight_pose.x - blackboard_ptr_->self_pose_.x) +
                                                (fight_pose.y - blackboard_ptr_->self_pose_.y)*(fight_pose.y - blackboard_ptr_->self_pose_.y));
                            if(dis < 0.3) chassis_exe_ptr_->GotoSwing(fight_pose.x, fight_pose.y, fight_pose.z, force_plan);
                            else chassis_exe_ptr_->Goto(fight_pose.x, fight_pose.y, fight_pose.z, force_plan);
                        }
                    }
                    return BehaviorState::SUCCESS;
                }
            }
            return BehaviorState::FAILURE;
        }
        private:
        cv::Point3d calc_fight_pose(double enemy_x, double enemy_y)
        {
            cv::Point3d fight_pose;
            int point_num = blackboard_ptr_->search_points_;
            double distance = 100.0f, r = blackboard_ptr_->fight_distance_;
            double new_goal_x, new_goal_y, goal_x = 0, goal_y = 0; 
            for(int i = 0; i < point_num; i++){
                new_goal_x = enemy_x + r * sin(CV_PI*2*i/point_num);
                new_goal_y = enemy_y + r * cos(CV_PI*2*i/point_num);
                if(new_goal_x < 0.3 || new_goal_x > 7.8) continue; 
                if(new_goal_y < 0.3 || new_goal_y > 4.2) continue;
                int barry_id;
                double self_pose[2] = {new_goal_x, new_goal_y};
                double enemy_pose[2] = {enemy_x, enemy_y};
                if(blackboard_ptr_->line_barriers_check(self_pose, enemy_pose, barry_id) == false){
                    double new_distance = sqrt((new_goal_x-blackboard_ptr_->self_pose_.x)*(new_goal_x-blackboard_ptr_->self_pose_.x) + 
                                               (new_goal_y-blackboard_ptr_->self_pose_.y)*(new_goal_y-blackboard_ptr_->self_pose_.y));
                    if(new_distance < distance){
                        distance = new_distance;
                        goal_x = new_goal_x;
                        goal_y = new_goal_y;
                    }
                }
            }
            fight_pose.x = goal_x;
            fight_pose.y = goal_y;
            fight_pose.z = atan2(enemy_y-goal_y, enemy_x-goal_x);
            if(std::fabs(fight_pose.z) < 0.05) fight_pose.z = 0;
            return fight_pose;
        }
        cv::Point3d last_fight_pose_;
        std::stringstream str;
    };
}

#endif