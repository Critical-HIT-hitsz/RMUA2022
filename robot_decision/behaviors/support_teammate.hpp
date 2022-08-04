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
            std::cout<<"support_teammmate"<<std::endl;
            //队友识别敌方，进行支援
            if(blackboard_ptr_->team_info_.attack_enemy != 0 && blackboard_ptr_->team_info_.received_)
            {
                cv::Point3d support_pose;
                std::cout<<blackboard_ptr_->team_info_.teammate_enemy_pose_.x << "    "<<blackboard_ptr_->team_info_.teammate_enemy_pose_.y << std::endl;
                if(blackboard_ptr_->teammate_pose_.x != 0 && blackboard_ptr_->teammate_pose_.y != 0){
                    support_pose = calc_support_pose(blackboard_ptr_->team_info_.teammate_enemy_pose_.x, blackboard_ptr_->team_info_.teammate_enemy_pose_.y,
                                                     blackboard_ptr_->teammate_pose_.x, blackboard_ptr_->teammate_pose_.y);
                    if(support_pose.x != 0 && support_pose.y != 0){
                        double dis = std::sqrt(pow(support_pose.x - last_support_pose_.x, 2) +
                                               pow(support_pose.y - last_support_pose_.y, 2));
                        bool force_plan = false;
                        
                        if(dis > 0.3){ // 与上一次支援点距离过大，强制支援
                            last_support_pose_ = support_pose;
                            force_plan = true;
                        }
                        // std::cout<<"dis_: "<<dis<<std::endl;
                        // std::cout<<"force_Plan: "<<force_plan<<std::endl;
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
            // 以敌方机器人为圆心的路径点作为新的追击点
            int point_num = blackboard_ptr_->search_points_;//机器人搜索点个数
            double distance = 0, r = blackboard_ptr_->fight_distance_;//攻击距离设置
            double new_goal_x, new_goal_y, new_goal_z = 0, goal_x = 0, goal_y = 0, goal_z = 0;
            std::vector<cv::Point3d> goal_pose;
            for(int i = 0; i < point_num; i++){
                double self_pose[3] = {new_goal_x, new_goal_y, new_goal_z};
                double enemy_pose[2] = {enemy_x, enemy_y};
                new_goal_x = enemy_x + r * sin(CV_PI*2*i/point_num);//计算攻击点
                new_goal_y = enemy_y + r * cos(CV_PI*2*i/point_num);
                new_goal_z = atan2(enemy_y-new_goal_y, enemy_x-new_goal_x);
                int barry_id;//判断攻击点与敌方机器人之间是否有障碍
                if(new_goal_x < 0.3 || new_goal_x > 7.8) continue;  // 去除边界外的点
                if(new_goal_y < 0.4 || new_goal_y > 4.2) continue;
                if(blackboard_ptr_->check_block(new_goal_x,new_goal_y)) continue;
                if(blackboard_ptr_->transfer_to_enemy_coordinate_(self_pose,blackboard_ptr_->global_map_.sentry_enemy_pose_[0],blackboard_ptr_->global_map_.sentry_enemy_pose_[1])
                    && blackboard_ptr_->global_map_.received_) continue;
                if(blackboard_ptr_->line_barriers_check(self_pose, enemy_pose, barry_id) == false){
                    if(blackboard_ptr_->back_car_check(self_pose) == false) goal_pose.push_back(cv::Point3d(new_goal_x, new_goal_y, new_goal_z)); //存储不会暴露后装甲的目标点
                    double new_distance = sqrt((new_goal_x-team_x)*(new_goal_x-team_x) + (new_goal_y-team_y)*(new_goal_y-team_y));
                    if(new_distance > distance){ //最后选择距离队友攻击点最远的点
                        distance = new_distance;
                        goal_x = new_goal_x;
                        goal_y = new_goal_y;
                        goal_z = new_goal_z;
                    }
                }
            }
            // std::cout << "    goal_size: " << goal_pose.size() << std::endl;
            // if(goal_pose.size() > 0)
            // {
            //     double dis_ = 0;
            //     for(int i = 0; i < goal_pose.size(); i++)
            //     {
            //         double dis = sqrt((goal_pose[i].x-team_x)*(goal_pose[i].x-team_x) + (goal_pose[i].y-team_y)*(goal_pose[i].y-team_y));
            //         if(dis > dis_){ //在不会暴露后装甲的预选点中选择距离最小的点
            //             dis_ = dis;
            //             goal_x = goal_pose[i].x;
            //             goal_y = goal_pose[i].y;
            //             goal_z = goal_pose[i].z;
            //         }
            //     }
            // }
            fight_pose.x = goal_x;//若都没有找到，攻击点为(0,0)
            fight_pose.y = goal_y;
            fight_pose.z = atan2(enemy_y-goal_y, enemy_x-goal_x);
            if(std::fabs(fight_pose.z) < 0.05) fight_pose.z = 0;
            return fight_pose;
        }
        cv::Point3d last_support_pose_;
        double enemy_pose_x, enemy_pose_y;
        std::stringstream str;
    };
}

#endif