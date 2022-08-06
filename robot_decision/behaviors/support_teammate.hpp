#ifndef SUPPORT_TEAMMATE_H
#define SUPPORT_TEAMMATE_H

#include "behavior_tree.h"
#include "self_info_executor.hpp"
#include "buff_executor.hpp"
#include "log_executor.hpp"

namespace robot_decision
{
    class SupportBehavior : public ActionNode
    {
        public:
        SupportBehavior(std::string name, 
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
            if(blackboard_ptr_->team_info_.attack_enemy != 0 && blackboard_ptr_->team_info_received_)
            {
                cv::Point3d support_pose;
                if(blackboard_ptr_->team_info_.pose_x_teammate != 0 && blackboard_ptr_->team_info_.pose_y_teammate != 0){
                    support_pose = calc_support_pose(blackboard_ptr_->team_info_.pose_x_enemy/100.0, blackboard_ptr_->team_info_.pose_y_enemy/100.0,
                                                     blackboard_ptr_->team_info_.pose_x_teammate/100.0, blackboard_ptr_->team_info_.pose_y_teammate/100.0);
                    if(support_pose.x != 0 && support_pose.y != 0){
                        double dis = std::sqrt(pow(support_pose.x - last_support_pose_.x, 2) +
                                               pow(support_pose.y - last_support_pose_.y, 2));
                        bool force_plan = false;
                        if(dis > 0.3){ 
                            last_support_pose_ = support_pose;
                            force_plan = true;
                        }
                        chassis_exe_ptr_->Goto(support_pose.x, support_pose.y, support_pose.z, force_plan);
                        return BehaviorState::SUCCESS;
                    }
                }
            }
            return BehaviorState::FAILURE;
        }
        private:
        cv::Point3d calc_support_pose(double enemy_x, double enemy_y, double team_x, double team_y)
        {
            cv::Point3d fight_pose;
            int point_num = blackboard_ptr_->search_points_;
            double distance = 0.0, r = blackboard_ptr_->fight_distance_;
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
                    double new_distance = sqrt((new_goal_x-team_x)*(new_goal_x-team_x) + (new_goal_y-team_y)*(new_goal_y-team_y));
                    if(new_distance > distance){
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
        cv::Point3d last_support_pose_;
        std::stringstream str;
    };
}

#endif