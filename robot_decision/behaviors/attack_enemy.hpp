#ifndef ATTACK_ENEMY_H
#define ATTACK_ENEMY_H

#include "behavior_tree.h"
#include "self_info_executor.hpp"
#include "buff_executor.hpp"
#include "log_executor.hpp"

namespace robot_decision
{
    class AttackEnemyBehavior : public ActionNode
    {
        public:
        AttackEnemyBehavior(std::string name, 
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
            // 自己有进攻目标，攻击敌方
            std::cout << "attack_enemy" << std::endl << std::endl;
            if(blackboard_ptr_->armor_detection_.detected_enemy) // 自己识别到目标进行进攻、追击
            {
                if(blackboard_ptr_->self_id_ < 100)
                {
                    self_info_exe_ptr_->Attack_Enemy(blackboard_ptr_->armor_detection_.detected_id_+2);//明确自己有进攻目标
                }
                else
                {
                    self_info_exe_ptr_->Attack_Enemy(blackboard_ptr_->armor_detection_.detected_id_);//明确自己有进攻目标
                }
                self_info_exe_ptr_->set_enemy_pos(blackboard_ptr_->armor_detection_.detected_enemy_pose_.x, blackboard_ptr_->armor_detection_.detected_enemy_pose_.y);
                cv::Point3d fight_pose;
                bool force_plan = false;//true为切换前进位置，false为不切换前进位置
                if(blackboard_ptr_->armor_detection_.distance < blackboard_ptr_->back_distance_*1000){ // 若距离过小，远离
                    fight_pose = calc_fight_pose(blackboard_ptr_->armor_detection_.detected_enemy_pose_.x, blackboard_ptr_->armor_detection_.detected_enemy_pose_.y);
                    if(fight_pose.x != 0. && fight_pose.y != 0.){
                        double dis = std::sqrt((last_fight_pose_.x-fight_pose.x)*(last_fight_pose_.x-fight_pose.x) + 
                                               (last_fight_pose_.y-fight_pose.y)*(last_fight_pose_.y-fight_pose.y));
                        if(dis > 0.2){//若上时刻位置点据此刻位置点较远，则切换位置点
                            force_plan = true;
                            last_fight_pose_ = fight_pose;
                        }
                        chassis_exe_ptr_->GotoSwing(fight_pose.x, fight_pose.y, fight_pose.z, force_plan);//摆尾后退
                    }
                }
                else if(blackboard_ptr_->armor_detection_.distance > blackboard_ptr_->attack_distance_*1000){ // 若距离过大，追击
                    fight_pose = calc_fight_pose(blackboard_ptr_->armor_detection_.detected_enemy_pose_.x, blackboard_ptr_->armor_detection_.detected_enemy_pose_.y);
                    if(fight_pose.x != 0. && fight_pose.y != 0.){
                        if(blackboard_ptr_->armor_detection_.distance > blackboard_ptr_->force_attack_distance_*1000)
                        {
                            force_plan = true;//距离太远强制追击
                        }
                        double dis = std::sqrt((fight_pose.x - blackboard_ptr_->self_pose_.x)*(fight_pose.x - blackboard_ptr_->self_pose_.x) +
                                               (fight_pose.y - blackboard_ptr_->self_pose_.y)*(fight_pose.y - blackboard_ptr_->self_pose_.y));
                        if(dis < 0.6) chassis_exe_ptr_->GotoSwing(fight_pose.x, fight_pose.y, fight_pose.z, force_plan);//规划点在附近，摆尾前进
                        else chassis_exe_ptr_->Goto(fight_pose.x, fight_pose.y, fight_pose.z, force_plan);
                    }
                }
                return BehaviorState::SUCCESS;
            }
            return BehaviorState::FAILURE;
        }


        cv::Point3d calc_fight_pose(double enemy_x, double enemy_y)
        {
            cv::Point3d fight_pose;
            // 以敌方机器人为圆心的路径点作为新的追击点
            int point_num = blackboard_ptr_->search_points_;//机器人搜索点个数
            double distance = 100.0f, r = blackboard_ptr_->fight_distance_;//攻击距离设置
            double new_goal_x, new_goal_y, new_goal_z, goal_x = 0, goal_y = 0, goal_z = 0; 
            std::vector<cv::Point3d> goal_pose; 
            for(int i = 0; i < point_num; i++){
                new_goal_x = enemy_x + r * sin(CV_PI*2*i/point_num);//计算攻击点
                new_goal_y = enemy_y + r * cos(CV_PI*2*i/point_num);
                new_goal_z = atan2(enemy_y - new_goal_y, enemy_x - new_goal_x);
                int barry_id;//判断攻击点与敌方机器人之间是否有障碍
                double self_pose[3] = {new_goal_x, new_goal_y, new_goal_z};
                double enemy_pose[2] = {enemy_x, enemy_y};
                if(new_goal_x < 0.3 || new_goal_x > 7.8) continue;  // 去除边界外的点
                if(new_goal_y < 0.4 || new_goal_y > 4.2) continue;
                if(blackboard_ptr_->check_block(new_goal_x,new_goal_y)) continue;
                if(blackboard_ptr_->line_barriers_check(self_pose, enemy_pose, barry_id) == false){
                    if(blackboard_ptr_->back_car_check(self_pose) == false) goal_pose.push_back(cv::Point3d(new_goal_x,new_goal_y,new_goal_z));
                    double new_distance = sqrt((new_goal_x-blackboard_ptr_->self_pose_.x)*(new_goal_x-blackboard_ptr_->self_pose_.x) + 
                                               (new_goal_y-blackboard_ptr_->self_pose_.y)*(new_goal_y-blackboard_ptr_->self_pose_.y));
                    if(new_distance < distance){ //最后选择距离最小的点
                        distance = new_distance;
                        goal_x = new_goal_x;
                        goal_y = new_goal_y;
                    }
                }
            }
            // if(goal_pose.size() > 0)
            // {
            //     double dis_ = 100.0f;
            //     for(int i = 0; i < goal_pose.size(); i++)
            //     {
            //         double dis = std::sqrt(std::pow(goal_pose[i].x - blackboard_ptr_->self_pose_.x,2) + std::pow(goal_pose[i].y - blackboard_ptr_->self_pose_.y,2));
            //         if(dis < dis_)
            //         {
            //             goal_x = goal_pose[i].x;
            //             goal_y = goal_pose[i].y;
            //             goal_z = goal_pose[i].z;
            //         }
            //     }
            // }
            fight_pose.x = goal_x;//若都没有找到，攻击点为(0,0)
            fight_pose.y = goal_y;
            fight_pose.z = atan2(enemy_y - goal_y, enemy_x - goal_x);
            if(std::fabs(fight_pose.z) < 0.05) fight_pose.z = 0;
            return fight_pose;
        }
        
        cv::Point3d last_fight_pose_;
        std::stringstream str;
    };
}

#endif