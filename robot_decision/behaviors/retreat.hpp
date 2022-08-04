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
            if(blackboard_ptr_->game_status_.remaining_time < 60 && blackboard_ptr_->game_buff_.remain_bullets_num_ >= 10 && blackboard_ptr_->game_robot_HP_.self_hp_ + blackboard_ptr_->game_robot_HP_.teammate_hp_ < blackboard_ptr_->game_robot_HP_.enemy_hp1_ +blackboard_ptr_->game_robot_HP_.enemy_hp2_ + 400)
            {
                return BehaviorState::FAILURE;
            }
            // std::cout<<"retreat"<<std::endl;
            double retreat_x[4] = {0.8, 0.8, 7.28, 7.28};
            double retreat_y[4] = {0.8, 3.98, 0.5, 3.68};
            double retreat_z[4] = {1.57, 0, 3.14, -1.57};
            bool flag_1, flag_2, flag_3, flag_4, flag_5;
            flag_1 = flag_2 = flag_3 = flag_4 = flag_5 = true;
            
            // int enemy_id = blackboard_ptr_->self_id_ % 100;
            // int enemy_hp;
            // if(enemy_id == 1)   enemy_hp = blackboard_ptr_->game_robot_HP_.enemy_hp1_;
            // else if(enemy_id == 2)  enemy_hp = blackboard_ptr_->game_robot_HP_.enemy_hp2_; 

            if(blackboard_ptr_->game_robot_HP_.self_hp_ < blackboard_ptr_->robot_status_.max_hp * 0.2 || 
              (blackboard_ptr_->game_buff_.remain_bullets_num_ < blackboard_ptr_->force_add_bullet_ && 
                blackboard_ptr_->game_buff_.self_bullet_buff_status_ == false) ||
                (blackboard_ptr_->robot_status_.shooter_enable == false && blackboard_ptr_->game_status_.remaining_time < 150)) // 是否撤退条件
            {
                // 撤退
                // 选择家为候选点，在候选点中筛选可以躲避的点
                std::vector<cv::Point3d> retreat_points;
                for(int i = 0; i < 4; i++)
                {
                    double pos_x = 0.0, pos_y = 0.0;
                    if(blackboard_ptr_->global_map_.received_)
                    {
                        pos_x = blackboard_ptr_->teammate_pose_.x;
                        pos_y = blackboard_ptr_->teammate_pose_.y;
                        if(pos_x > 0 && pos_x < 8.08 && pos_y > 0 && pos_y < 4.48)
                            flag_1 = check_retreat_flag(i,retreat_x[i],retreat_y[i],pos_x,pos_y);
                        else
                        {
                            flag_1 = true;
                        }
                        pos_x = blackboard_ptr_->global_map_.sentry_enemy_pose_[0].x;
                        pos_y = blackboard_ptr_->global_map_.sentry_enemy_pose_[0].y;
                        if(pos_x > 0 && pos_x < 8.08 && pos_y > 0 && pos_y < 4.48)
                            flag_2 = check_retreat_flag(i,retreat_x[i],retreat_y[i],pos_x,pos_y);
                        else
                        {
                            flag_2 = true;
                        }
                        pos_x = blackboard_ptr_->global_map_.sentry_enemy_pose_[1].x;
                        pos_y = blackboard_ptr_->global_map_.sentry_enemy_pose_[1].y;
                        if(pos_x > 0 && pos_x < 8.08 && pos_y > 0 && pos_y < 4.48)
                            flag_3 = check_retreat_flag(i,retreat_x[i],retreat_y[i],pos_x,pos_y);
                        else
                        {
                            flag_3 = true;
                        } 
                        pos_x = blackboard_ptr_->global_map_.sentry_die_pose_[0].x;
                        pos_y = blackboard_ptr_->global_map_.sentry_die_pose_[0].y;
                        if(pos_x > 0 && pos_x < 8.08 && pos_y > 0 && pos_y < 4.48)
                            flag_4 = check_retreat_flag(i,retreat_x[i],retreat_y[i],pos_x,pos_y);
                        else
                        {
                            flag_4 = true;
                        } 
                        pos_x = blackboard_ptr_->global_map_.sentry_die_pose_[1].x;
                        pos_y = blackboard_ptr_->global_map_.sentry_die_pose_[1].y;
                        if(pos_x > 0 && pos_x < 8.08 && pos_y > 0 && pos_y < 4.48)
                            flag_5 = check_retreat_flag(i,retreat_x[i],retreat_y[i],pos_x,pos_y);
                        else
                        {
                            flag_5 = true;
                        } 
                    }
                    else
                    {
                        flag_1 = flag_2 = flag_3 = flag_4= flag_5 = true;
                    } 

                    if(flag_1 && flag_2 && flag_3 && flag_4 && flag_5)//找合适的点为候选点，防止有其他机器人挡在撤退点处
                        retreat_points.push_back(cv::Point3d(retreat_x[i], retreat_y[i],retreat_z[i]));
                }
                // 在候选点中选择距离最近的点
                if(retreat_points.size() == 0) return BehaviorState::FAILURE;
                double min_dis = 100;
                cv::Point3d final_retreat_point;
                for(int i = 0; i < retreat_points.size(); i++){
                    double dis = std::sqrt(std::pow(blackboard_ptr_->self_pose_.x-retreat_points[i].x, 2)+
                                        std::pow(blackboard_ptr_->self_pose_.y-retreat_points[i].y, 2));
                    if(dis < min_dis){
                        min_dis = dis;
                        final_retreat_point = retreat_points[i];
                    }
                }
                bool retreat_flag = false;
                if(blackboard_ptr_->game_buff_.remain_bullets_num_ <= 5)   retreat_flag = true;
                if(min_dis <= 0.5) chassis_exe_ptr_->GotoSwing(final_retreat_point.x, final_retreat_point.y, final_retreat_point.z, retreat_flag);
                if(min_dis <= 0.2) chassis_exe_ptr_->Spin();
                else chassis_exe_ptr_->Goto(final_retreat_point.x, final_retreat_point.y, final_retreat_point.z, retreat_flag);
                return BehaviorState::SUCCESS;
            }
            return BehaviorState::FAILURE;
        }

        private:
        bool check_retreat_flag(int retreat_id, double retreat_x, double retreat_y, double car_x, double car_y)
        {
            double dis = std::sqrt(std::pow(car_x-retreat_x,2)+std::pow(car_y-retreat_y,2));
            if(dis > 2.7) return true;
            if(retreat_id == 0)
            {
                if(car_x < blackboard_ptr_->barries_pos[1][1])
                    return false;
                return true;
            }
            else if(retreat_id == 1)
            {
                if(car_y > blackboard_ptr_->barries_pos[0][3])
                    return false;
                return true;
            }
            else if(retreat_id == 2)
            {
                if(car_y < blackboard_ptr_->barries_pos[5][2])
                    return false;
                return true;
            }
            else
            {
                if(car_x > blackboard_ptr_->barries_pos[4][0])
                    return false;
                return true;
            }
        }

    };
}

#endif