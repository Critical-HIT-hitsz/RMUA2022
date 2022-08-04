#ifndef GLOBAL_MAP_H
#define GLOBAL_MAP_H

#include "behavior_tree.h"
#include "self_info_executor.hpp"
#include "buff_executor.hpp"
#include "log_executor.hpp"

namespace robot_decision
{
    class GlobalmapBehavior : public ActionNode
    {
    public:
        GlobalmapBehavior(std::string name,
                          int level,
                          const Blackboard::Ptr &blackboard_ptr,
                          Chassis_executor::Ptr &chassis_exe_ptr,
                          Log_executor::Ptr &log_exe_ptr,
                          Buff_executor::Ptr &buff_exe_ptr,
                          Self_Info_executor::Ptr &self_info_exe_ptr) : ActionNode::ActionNode(name, level, blackboard_ptr, chassis_exe_ptr, log_exe_ptr, buff_exe_ptr, self_info_exe_ptr)
        {
        }
        BehaviorState Update()
        {
            double self_point[3] = {0, 0, 0};
            double enemy1_point[2] = {0, 0};
            double enemy2_point[2] = {0, 0};
            bad_fight_point[0] = 0;
            bad_fight_point[1] = 0;
            enemy1_point[0] = blackboard_ptr_->global_map_.sentry_enemy_pose_[0].x;
            enemy1_point[1] = blackboard_ptr_->global_map_.sentry_enemy_pose_[0].y;
            enemy2_point[0] = blackboard_ptr_->global_map_.sentry_enemy_pose_[1].x;
            enemy2_point[1] = blackboard_ptr_->global_map_.sentry_enemy_pose_[1].y;
            // std::cout << "enemy1_point:" << enemy1_point[0] << "," << enemy1_point[1] << std::endl;
            // std::cout << "enemy2_point:" << enemy2_point[0] << "," << enemy2_point[1] << std::endl;
            self_point[0] = blackboard_ptr_->self_pose_.x;
            self_point[1] = blackboard_ptr_->self_pose_.y;
            self_point[2] = blackboard_ptr_->gimbal_global_angle_;
            double vector_self[2] = {0, 0};
            vector_self[0] = std::cos(self_point[2]);
            vector_self[1] = std::sin(self_point[2]);
            double vector_enemy1[2] = {0, 0};
            vector_enemy1[0] = self_point[0] - enemy1_point[0];
            vector_enemy1[1] = self_point[1] - enemy1_point[1];
            double vector_enemy2[2] = {0, 0};
            vector_enemy2[0] = self_point[0] - enemy2_point[0];
            vector_enemy2[1] = self_point[1] - enemy2_point[1];
            double dis1 = 0;
            double cos_angle1 = 0;
            // std::cout<<"back_flag" << back_flag << std::endl;
            back_flag = false;
            if (blackboard_ptr_->game_robot_HP_.enemy_hp1_ != 0)
            {
                dis1 = std::sqrt(std::pow(vector_enemy1[0], 2) + std::pow(vector_enemy1[1], 2));
                cos_angle1 = (vector_self[0] * vector_enemy1[0] + vector_self[1] * vector_enemy1[1]) / dis1;
            }
            double dis2 = 0;
            double cos_angle2 = 0;
            if (blackboard_ptr_->game_robot_HP_.enemy_hp2_ != 0)
            {
                dis2 = std::sqrt(std::pow(vector_enemy2[0], 2) + std::pow(vector_enemy2[1], 2));
                cos_angle2 = (vector_self[0] * vector_enemy2[0] + vector_self[1] * vector_enemy2[1]) / dis2;
            }
            // if ((dis1 < 3.5 && cos_angle1 > 0.707) || (dis2 < 3.5 && cos_angle2 > 0.707))
            // {
            //     back_flag = true;
            // }
            if(blackboard_ptr_->robot_damage_.received_ && blackboard_ptr_->robot_damage_.damage_type_ == 0 && blackboard_ptr_->robot_damage_.damage_source_ == 2)
            {
                back_flag = true;
            }
            // std::cout << "dis1:" << dis1 << " cos_angle1:" << cos_angle1 << std::endl;
            // std::cout << "dis2:" << dis2 << " cos_angle2:" << cos_angle2 << std::endl;
            if (back_flag)
            {
                bad_fight_point[0] = self_point[0];
                bad_fight_point[1] = self_point[1];
                // std::cout << "bad_point" << bad_fight_point[0] << "," << bad_fight_point[1] << std::endl;
            
                cv::Point3d hide_point = calc_hide_pose(self_point[0], self_point[1], enemy1_point[0], enemy1_point[1], enemy2_point[0], enemy2_point[1]);
                bool force_plan = false;
                double dis = std::sqrt(std::pow((hide_point.x - self_point[0]), 2) + std::pow((hide_point.y - self_point[1]), 2));
                if (hide_point.x != 0 && hide_point.y != 0)
                {
                    if (dis > 0.6)
                    {
                        force_plan = true;
                        chassis_exe_ptr_->Goto(hide_point.x, hide_point.y, hide_point.z, force_plan);
                        return BehaviorState::SUCCESS;
                    }
                    else
                    {
                        chassis_exe_ptr_->GotoSwing(hide_point.x, hide_point.y, hide_point.z, force_plan);
                        return BehaviorState::SUCCESS;
                    }
                }
                // return BehaviorState::FAILURE;
                
            }


            std::cout << "global_map" << std::endl;
            double self_x = blackboard_ptr_->self_pose_.x;
            double self_y = blackboard_ptr_->self_pose_.y;
            double teammate_x = blackboard_ptr_->teammate_pose_.x;
            double teammate_y = blackboard_ptr_->teammate_pose_.y;
            double enemy1_x = blackboard_ptr_->global_map_.sentry_enemy_pose_[0].x;
            double enemy1_y = blackboard_ptr_->global_map_.sentry_enemy_pose_[0].y;
            double enemy2_x = blackboard_ptr_->global_map_.sentry_enemy_pose_[1].x;
            double enemy2_y = blackboard_ptr_->global_map_.sentry_enemy_pose_[1].y;
            if(blackboard_ptr_->game_robot_HP_.enemy_hp1_ > 0 && blackboard_ptr_->game_robot_HP_.enemy_hp2_ > 0)    two_enemy_flag = true;
            else    two_enemy_flag = false;
            attack_mode = 1;
            //二打一或二打二
            if (blackboard_ptr_->global_map_.received_ && blackboard_ptr_->global_map_trust_ && blackboard_ptr_->game_robot_HP_.teammate_hp_ > 0 &&
                teammate_x != 0 && teammate_y != 0 && ((enemy1_x != 0 && enemy1_y != 0) || (enemy2_x != 0 && enemy2_y != 0)))
            {
                double attack_enemy_x, attack_enemy_y, another_enemy_x, another_enemy_y;
                if ((enemy1_x != 0 && enemy1_y != 0) &&
                    (enemy2_x != 0 && enemy2_y != 0) && two_enemy_flag)//围攻在场地中间的敌方1号或2号
                {
                    double enemy_enemy_dis = std::sqrt(std::pow(enemy1_x-enemy2_x,2)+std::pow(enemy1_y-enemy2_y,2));
                    if (enemy_enemy_dis <= 1.5)
                    {
                        attack_enemy_x = (enemy1_x + enemy2_x)*0.5;
                        attack_enemy_y = (enemy1_y + enemy2_y)*0.5;
                        attack_mode = 2;
                    }
                    else if ((enemy1_x > 2.45 && enemy1_x < 5.63 && last_enemy_id == 1) || (enemy1_x > 2.45 && enemy1_x < 5.63 && (enemy2_x <= 2.45 || enemy2_x >= 5.63)))
                    {
                        attack_enemy_x = enemy1_x;
                        attack_enemy_y = enemy1_y;
                        another_enemy_x = enemy2_x;
                        another_enemy_y = enemy2_y;
                        last_enemy_id = 1;
                    }
                    else if (enemy2_x > 2.45 && enemy2_x < 5.63)
                    {
                        attack_enemy_x = enemy2_x;
                        attack_enemy_y = enemy2_y;
                        another_enemy_x = enemy1_x;
                        another_enemy_y = enemy1_y;
                        last_enemy_id = 2;
                    }
                    else
                    {
                        if(blackboard_ptr_->self_id_%100 == 1)
                        {
                            attack_enemy_x = enemy1_x;
                            attack_enemy_y = enemy1_y;
                            another_enemy_x = enemy2_x;
                            another_enemy_y = enemy2_y;
                        }
                        else
                        {
                            attack_enemy_x = enemy2_x;
                            attack_enemy_y = enemy2_y;
                            another_enemy_x = enemy1_x;
                            another_enemy_y = enemy1_y;
                        }
                    }
                }
                else if (blackboard_ptr_->game_robot_HP_.enemy_hp1_ > 0 &&
                         blackboard_ptr_->game_robot_HP_.enemy_hp2_ <= 0)
                {
                    attack_enemy_x = enemy1_x;
                    attack_enemy_y = enemy1_y;
                    another_enemy_x = enemy2_x;
                    another_enemy_y = enemy2_y;
                }
                else if (blackboard_ptr_->game_robot_HP_.enemy_hp1_ <= 0 &&
                         blackboard_ptr_->game_robot_HP_.enemy_hp2_ > 0)
                {
                    attack_enemy_x = enemy2_x;
                    attack_enemy_y = enemy2_y;
                    another_enemy_x = enemy1_x;
                    another_enemy_y = enemy1_y;
                }
                if(blackboard_ptr_->self_id_ < 100)
                {
                    int enemy_id;
                    if(attack_enemy_x == enemy1_x && attack_enemy_y == enemy1_y)    enemy_id = 1;
                    else enemy_id = 2;
                    self_info_exe_ptr_->Attack_Enemy(enemy_id+2);//明确自己有进攻目标
                }
                else
                {
                    int enemy_id;
                    if(attack_enemy_x == enemy1_x && attack_enemy_y == enemy1_y)    enemy_id = 1;
                    else enemy_id = 2;
                    self_info_exe_ptr_->Attack_Enemy(enemy_id);//明确自己有进攻目标
                }
                self_info_exe_ptr_->set_enemy_pos(attack_enemy_x, attack_enemy_y);
                
                cv::Point3d goal_pose;
                if(blackboard_ptr_->team_info_.team_mode == 1)
                {
                    goal_pose = calc_support_pose(attack_enemy_x, attack_enemy_y, another_enemy_x, another_enemy_y, self_x, self_y, teammate_x, teammate_y);
                    double dis = std::sqrt(std::pow(self_x - attack_enemy_x, 2) + std::pow(self_y - attack_enemy_y, 2));
                    if (dis < blackboard_ptr_->back_distance_)
                    {
                        if (!GoPoseBack(goal_pose, self_x, self_y))
                        {
                            self_info_exe_ptr_->set_team_mode(0);//0发给队友，表示此刻自己进入下一节点
                            return BehaviorState::FAILURE;
                        }
                        else
                        {
                            self_info_exe_ptr_->set_team_mode(2);//2表示此刻自己正在配合队友进攻，但暂未使用
                            return BehaviorState::SUCCESS;
                        }
                            
                    }
                    else if (dis > blackboard_ptr_->attack_distance_)
                    {
                        if (!GoPoseAttack(goal_pose, self_x, self_y))
                        {
                            self_info_exe_ptr_->set_team_mode(0);
                            return BehaviorState::FAILURE;
                        }
                        else
                        {
                            self_info_exe_ptr_->set_team_mode(2);
                            return BehaviorState::SUCCESS;
                        }
                    }
                    self_info_exe_ptr_->set_team_mode(2);
                    return BehaviorState::SUCCESS;
                }   
                else
                {
                    goal_pose = calc_fight_pose(attack_enemy_x, attack_enemy_y, another_enemy_x, another_enemy_y, self_x, self_y);
                    double dis = std::sqrt(std::pow(self_x - attack_enemy_x, 2) + std::pow(self_y - attack_enemy_y, 2));
                    if (dis < blackboard_ptr_->back_distance_)
                    {
                        if (!GoPoseBack(goal_pose, self_x, self_y))
                        {
                            self_info_exe_ptr_->set_team_mode(0);
                            return BehaviorState::FAILURE;
                        }
                        else
                        {
                            self_info_exe_ptr_->set_team_mode(1);//1发给队友表示自己正在主攻
                            return BehaviorState::SUCCESS;
                        }
                    }
                    else if (dis > blackboard_ptr_->attack_distance_)
                    {
                        if (!GoPoseAttack(goal_pose, self_x, self_y))
                        {
                            self_info_exe_ptr_->set_team_mode(0);
                            return BehaviorState::FAILURE;
                        }
                        else
                        {
                            self_info_exe_ptr_->set_team_mode(1);
                            return BehaviorState::SUCCESS;
                        }
                    }
                    self_info_exe_ptr_->set_team_mode(1);
                    return BehaviorState::SUCCESS;
                }
            }
            else if (blackboard_ptr_->game_robot_HP_.teammate_hp_ <= 0 && blackboard_ptr_->global_map_.received_ && blackboard_ptr_->global_map_trust_ &&
                     (blackboard_ptr_->game_robot_HP_.enemy_hp1_ > 0 || blackboard_ptr_->game_robot_HP_.enemy_hp2_ > 0)) //一打一或一打二
            {
                double attack_enemy_x, attack_enemy_y;
                cv::Point3d goal_pose;
                //敌方两辆车都存活且己方只有自己时，攻击敌方血量较少的车；若双方都只存活一辆车，则主动进攻敌车
                if (blackboard_ptr_->game_robot_HP_.enemy_hp1_ >= blackboard_ptr_->game_robot_HP_.enemy_hp2_ && blackboard_ptr_->game_robot_HP_.enemy_hp2_ > 0)
                {
                    attack_enemy_x = blackboard_ptr_->global_map_.sentry_enemy_pose_[1].x;
                    attack_enemy_y = blackboard_ptr_->global_map_.sentry_enemy_pose_[1].y;
                }
                else if (blackboard_ptr_->game_robot_HP_.enemy_hp2_ >= blackboard_ptr_->game_robot_HP_.enemy_hp1_ && blackboard_ptr_->game_robot_HP_.enemy_hp1_ > 0)
                {
                    attack_enemy_x = blackboard_ptr_->global_map_.sentry_enemy_pose_[0].x;
                    attack_enemy_y = blackboard_ptr_->global_map_.sentry_enemy_pose_[0].y;
                }
                else if (blackboard_ptr_->game_robot_HP_.enemy_hp1_ > 0 && blackboard_ptr_->game_robot_HP_.enemy_hp2_ <= 0)
                {
                    attack_enemy_x = blackboard_ptr_->global_map_.sentry_enemy_pose_[0].x;
                    attack_enemy_y = blackboard_ptr_->global_map_.sentry_enemy_pose_[0].y;
                }
                else if (blackboard_ptr_->game_robot_HP_.enemy_hp2_ > 0 && blackboard_ptr_->game_robot_HP_.enemy_hp1_ <= 0)
                {
                    attack_enemy_x = blackboard_ptr_->global_map_.sentry_enemy_pose_[1].x;
                    attack_enemy_y = blackboard_ptr_->global_map_.sentry_enemy_pose_[1].y;
                }
                if(blackboard_ptr_->self_id_ < 100)
                {
                    int enemy_id;
                    if(attack_enemy_x == enemy1_x && attack_enemy_y == enemy1_y)    enemy_id = 1;
                    else enemy_id = 2;
                    self_info_exe_ptr_->Attack_Enemy(enemy_id+2);//明确自己有进攻目标
                }
                else
                {
                    int enemy_id;
                    if(attack_enemy_x == enemy1_x && attack_enemy_y == enemy1_y)    enemy_id = 1;
                    else enemy_id = 2;
                    self_info_exe_ptr_->Attack_Enemy(enemy_id);//明确自己有进攻目标
                }
                self_info_exe_ptr_->set_enemy_pos(attack_enemy_x, attack_enemy_y);

                goal_pose = calc_attack_pose(attack_enemy_x, attack_enemy_y);
                double dis = std::sqrt(std::pow(self_x - attack_enemy_x, 2) + std::pow(self_y - attack_enemy_y, 2));
                if (dis < blackboard_ptr_->back_distance_)
                {
                    if (!GoPoseBack(goal_pose, self_x, self_y))
                        return BehaviorState::FAILURE;
                    else
                        return BehaviorState::SUCCESS;
                }
                else if (dis > blackboard_ptr_->attack_distance_)
                {
                    if (!GoPoseAttack(goal_pose, self_x, self_y))
                        return BehaviorState::FAILURE;
                    else
                        return BehaviorState::SUCCESS;
                }
                return BehaviorState::SUCCESS;
            }
            return BehaviorState::FAILURE;
        }

    private:
        cv::Point3d calc_fight_pose(double enemy_x, double enemy_y, double another_x, double another_y, double self_x, double self_y) //适用于二打二或二打一时主动进攻
        {
            cv::Point3d fight_pose;
            // 以敌方机器人为圆心的路径点作为新的追击点
            int point_num = blackboard_ptr_->search_points_;                //机器人搜索点个数
            double distance = 100.0f, r = blackboard_ptr_->fight_distance_; //攻击距离设置
            if(attack_mode == 2)    
            {
                point_num = point_num * 2;
                r = 2.5;
            }
            std::cout<<"attack_mode"<<attack_mode<<std::endl;
            double new_goal_x, new_goal_y, goal_x = 0, goal_y = 0;
            for (int i = 0; i < point_num; i++)
            {
                new_goal_x = enemy_x + r * sin(CV_PI * 2 * i / point_num); //计算攻击点
                new_goal_y = enemy_y + r * cos(CV_PI * 2 * i / point_num);
                double self_pose[2] = {new_goal_x, new_goal_y};
                double enemy_pose[2] = {enemy_x, enemy_y};
                if (new_goal_x < 0.3 || new_goal_x > 7.8)
                    continue; // 去除边界外的点
                if (new_goal_y < 0.4 || new_goal_y > 4.2)
                    continue;
                if (blackboard_ptr_->check_block(new_goal_x, new_goal_y))
                    continue;
                int barry_id; //判断攻击点与敌方机器人之间是否有障碍
                if (attack_mode != 2 && two_enemy_flag && (((another_x-enemy_x)*(self_x-enemy_x)+(another_y-enemy_y)*(self_y-enemy_y))/std::pow(((self_x-enemy_x)*(self_x-enemy_x)+(self_y-enemy_y)*(self_y-enemy_y))*((another_x-enemy_x)*(another_x-enemy_x)+(another_y-enemy_y)*(another_y-enemy_y)),0.5) > 0.707 ||
                    ((another_x-enemy_x)*(self_x-enemy_x)+(another_y-enemy_y)*(self_y-enemy_y))/std::pow(((self_x-enemy_x)*(self_x-enemy_x)+(self_y-enemy_y)*(self_y-enemy_y))*((another_x-enemy_x)*(another_x-enemy_x)+(another_y-enemy_y)*(another_y-enemy_y)),0.5) < -0.707))
                    continue;
                if (blackboard_ptr_->line_barriers_check(self_pose, enemy_pose, barry_id) == false)
                {
                    double new_distance = sqrt((new_goal_x - self_x) * (new_goal_x - self_x) +
                                               (new_goal_y - self_y) * (new_goal_y - self_y));
                    if (new_distance < distance)
                    { //最后选择距离最小的点
                        distance = new_distance;
                        goal_x = new_goal_x;
                        goal_y = new_goal_y;
                    }
                }
            }
            fight_pose.x = goal_x; //若都没有找到，攻击点为(0,0)
            fight_pose.y = goal_y;
            fight_pose.z = atan2(enemy_y - goal_y, enemy_x - goal_x);
            std::cout<<"fight_pose.x: "<<fight_pose.x <<std::endl;
            if (std::fabs(fight_pose.z) < 0.05)
                fight_pose.z = 0;
            return fight_pose;
        }

        cv::Point3d calc_support_pose(double enemy_x, double enemy_y, double another_x, double another_y, double self_x, double self_y, double team_x, double team_y) //适用于二打二或二打一时支援进攻
        {
            cv::Point3d fight_pose;
            // 以敌方机器人为圆心的路径点作为新的追击点
            int point_num = blackboard_ptr_->search_points_;             //机器人搜索点个数
            double distance = 0.0, r = blackboard_ptr_->fight_distance_; //攻击距离设置
            if(attack_mode == 2)    
            {
                point_num = point_num * 2;
                r = 2.5;
            }
            double new_goal_x, new_goal_y, goal_x = 0, goal_y = 0;
            for (int i = 0; i < point_num; i++)
            {
                new_goal_x = enemy_x + r * sin(CV_PI * 2 * i / point_num); //计算攻击点
                new_goal_y = enemy_y + r * cos(CV_PI * 2 * i / point_num);
                if (new_goal_x < 0.3 || new_goal_x > 7.8)
                    continue; // 去除边界外的点
                if (new_goal_y < 0.4 || new_goal_y > 4.2)
                    continue;
                if (blackboard_ptr_->check_block(new_goal_x, new_goal_y))
                    continue;
                if (attack_mode != 2 && two_enemy_flag && ((another_x-enemy_x)*(self_x-enemy_x)+(another_y-enemy_y)*(self_y-enemy_y))/std::pow(((self_x-enemy_x)*(self_x-enemy_x)+(self_y-enemy_y)*(self_y-enemy_y))*((another_x-enemy_x)*(another_x-enemy_x)+(another_y-enemy_y)*(another_y-enemy_y)),0.5) > 0.707)
                    continue;
                int barry_id; //判断攻击点与敌方机器人之间是否有障碍
                double self_pose[2] = {new_goal_x, new_goal_y};
                double enemy_pose[2] = {enemy_x, enemy_y};
                if (blackboard_ptr_->line_barriers_check(self_pose, enemy_pose, barry_id) == false)
                {
                    double new_distance = sqrt((new_goal_x - team_x) * (new_goal_x - team_x) + (new_goal_y - team_y) * (new_goal_y - team_y));
                    if (new_distance > distance)
                    { //最后选择距离队友攻击点最远的点
                        distance = new_distance;
                        goal_x = new_goal_x;
                        goal_y = new_goal_y;
                    }
                }
            }
            fight_pose.x = goal_x; //若都没有找到，攻击点为(0,0)
            fight_pose.y = goal_y;
            fight_pose.z = atan2(enemy_y - goal_y, enemy_x - goal_x);
            if (std::fabs(fight_pose.z) < 0.05)
                fight_pose.z = 0;
            return fight_pose;
        }

        cv::Point3d calc_attack_pose(double enemy_x, double enemy_y) //适用于一打二和一打一
        {
            cv::Point3d fight_pose;
            // 以敌方机器人为圆心的路径点作为新的追击点
            int point_num = blackboard_ptr_->search_points_;                //机器人搜索点个数
            double distance = 100.0f, r = blackboard_ptr_->fight_distance_; //攻击距离设置
            double new_goal_x, new_goal_y, new_goal_z, goal_x = 0, goal_y = 0, goal_z = 0;
            std::vector<cv::Point3d> goal_pose;
            for (int i = 0; i < point_num; i++) 
            {
                new_goal_x = enemy_x + r * sin(CV_PI * 2 * i / point_num); //计算攻击点
                new_goal_y = enemy_y + r * cos(CV_PI * 2 * i / point_num);
                new_goal_z = atan2(enemy_y - new_goal_y, enemy_x - new_goal_x);
                int barry_id; //判断攻击点与敌方机器人之间是否有障碍
                double self_pose[3] = {new_goal_x, new_goal_y, new_goal_z};
                double enemy_pose[2] = {enemy_x, enemy_y};
                if (new_goal_x < 0.3 || new_goal_x > 7.8)
                    continue; // 去除边界外的点
                if (new_goal_y < 0.4 || new_goal_y > 4.2)
                    continue;
                if (blackboard_ptr_->check_block(new_goal_x, new_goal_y))
                    continue;
                // 筛掉被敌方集火的点
                if (blackboard_ptr_->game_robot_HP_.enemy_hp1_ > 0 && blackboard_ptr_->game_robot_HP_.enemy_hp2_ > 0)
                {
                    if (blackboard_ptr_->global_map_.received_ && blackboard_ptr_->transfer_to_enemy_coordinate_(self_pose, blackboard_ptr_->global_map_.sentry_enemy_pose_[0], blackboard_ptr_->global_map_.sentry_enemy_pose_[1]))
                        continue;
                }
                if (blackboard_ptr_->line_barriers_check(self_pose, enemy_pose, barry_id) == false)
                {
                    // 存储后装甲被障碍块保护或尾部无敌方机器人的点
                    if (blackboard_ptr_->back_car_check(self_pose) == false)
                    {
                        goal_pose.push_back(cv::Point3d(new_goal_x, new_goal_y, new_goal_z));
                    }
                    double new_distance = sqrt((new_goal_x - blackboard_ptr_->self_pose_.x) * (new_goal_x - blackboard_ptr_->self_pose_.x) +
                                               (new_goal_y - blackboard_ptr_->self_pose_.y) * (new_goal_y - blackboard_ptr_->self_pose_.y));
                    if (new_distance < distance)
                    { //最后选择距离最小的点
                        distance = new_distance;
                        goal_x = new_goal_x;
                        goal_y = new_goal_y;
                    }
                }
            }
            if (goal_pose.size() > 0)
            {
                double dis_ = 100.0f;
                for (int i = 0; i < goal_pose.size(); i++)
                {
                    if(blackboard_ptr_->check_block(goal_pose[i].x,goal_pose[i].y)) continue;
                    double dis = std::sqrt(std::pow(goal_pose[i].x - blackboard_ptr_->self_pose_.x, 2) + std::pow(goal_pose[i].y - blackboard_ptr_->self_pose_.y, 2));
                    if (dis < dis_)
                    {
                        goal_x = goal_pose[i].x;
                        goal_y = goal_pose[i].y;
                    }
                }
            }
            fight_pose.x = goal_x; //若都没有找到，攻击点为(0,0)
            fight_pose.y = goal_y;
            fight_pose.z = atan2(enemy_y - goal_y, enemy_x - goal_x);
            if (std::fabs(fight_pose.z) < 0.05)
                fight_pose.z = 0;
            return fight_pose;
        }

        bool GoPoseBack(cv::Point3d goal_pose, double self_x, double self_y)
        {
            if (goal_pose.x == 0 && goal_pose.y == 0)
            {
                return false;
            }

            //根据上一时刻目标点到此刻目标点的距离选择是否切换目标点
            double last_now_dis = std::sqrt((last_fight_pose_.x - goal_pose.x) * (last_fight_pose_.x - goal_pose.x) +
                                            (last_fight_pose_.y - goal_pose.y) * (last_fight_pose_.y - goal_pose.y));
            if (last_now_dis > 0.2)
            {
                force_plan = true;
                last_fight_pose_ = goal_pose;
            }

            //根据自己位置到目标点的距离选择前进方式
            double fight_pose_self_dis = std::sqrt((goal_pose.x - self_x) * (goal_pose.x - self_x) +
                                                   (goal_pose.y - self_y) * (goal_pose.y - self_y));
            if (fight_pose_self_dis < 0.6)
                chassis_exe_ptr_->GotoSwing(goal_pose.x, goal_pose.y, goal_pose.z, force_plan); //规划点在附近，摆尾前进
            else
                chassis_exe_ptr_->Goto(goal_pose.x, goal_pose.y, goal_pose.z, force_plan);
            return true;
        }

        bool GoPoseAttack(cv::Point3d goal_pose, double self_x, double self_y)
        {
            if (goal_pose.x == 0. && goal_pose.y == 0.)
                return false;
            force_plan = true; //距离太远强制追击
            double dis = std::sqrt((goal_pose.x - self_x) * (goal_pose.x - self_x) +
                                   (goal_pose.y - self_y) * (goal_pose.y - self_y));
            if (dis < 0.6)
                chassis_exe_ptr_->GotoSwing(goal_pose.x, goal_pose.y, goal_pose.z, force_plan); //规划点在附近，摆尾前进
            else
                chassis_exe_ptr_->Goto(goal_pose.x, goal_pose.y, goal_pose.z, force_plan);
            return true;
        }

        cv::Point3d calc_hide_pose(double self_x, double self_y, double enemy1_x, double enemy1_y, double enemy2_x, double enemy2_y)
        {
            cv::Point3d hide_pose;
            double goal_point_x = 0;
            double goal_point_y = 0;
            double goal_point_z = 0;
            double distance = 100.0f;
            int num_point = sizeof(hide_back_point) / sizeof(hide_back_point[0]);
            for (int i = 0; i < num_point; i++)
            {
                blackboard_ptr_->check_block(hide_back_point[i][0],hide_back_point[i][1]);
                double dis_1 = std::sqrt(std::pow((enemy1_x - hide_back_point[i][0]), 2) + std::pow((enemy1_y - hide_back_point[i][1]), 2));
                double dis_2 = std::sqrt(std::pow((enemy2_x - hide_back_point[i][0]), 2) + std::pow((enemy2_y - hide_back_point[i][1]), 2));
                if(dis_1 < 0.5 || dis_2 < 0.5)  continue;
                double new_dis = std::sqrt(std::pow((self_x - hide_back_point[i][0]), 2) + std::pow((self_y - hide_back_point[i][1]), 2));
                if (new_dis < distance)
                {
                    goal_point_x = hide_back_point[i][0];
                    goal_point_y = hide_back_point[i][1];
                    goal_point_z = hide_back_point[i][2];
                    distance = new_dis;
                }
            }
            hide_pose.x = goal_point_x;
            hide_pose.y = goal_point_y;
            hide_pose.z = goal_point_z;
            return hide_pose;
        }

        cv::Point3d last_fight_pose_ = {0, 0, 0};
        bool force_plan = false;
        bool two_enemy_flag = true;
        double bad_fight_point[2];
        int last_enemy_id = 1;
        int attack_mode;
        bool back_flag = false;
        double hide_back_point[12][3] =
            {{0.5, 2.0, 0.0},
             {7.58, 2.48, 3.14},
             {2.69, 0.5, 1.57},
             {5.38, 0.5, 1.57},
             {2.69, 3.98, -1.57},
             {5.38, 3.98, -1.57},
             {0.8,0.8,1.57},
             {0.8,3.98,0.0},
             {7.28,0.5,3.14},
             {7.28,3.68,-1.57}};
    };
}

#endif
