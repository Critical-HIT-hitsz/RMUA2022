#ifndef CALLBACK_H
#define CALLBACK_H

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <numeric>
#include <opencv2/opencv.hpp>
#include "std_msgs/Bool.h"
#include "robot_msgs/GameStatus.h"
#include "robot_msgs/GameResult.h"
#include "robot_msgs/GameRobotHP.h"
#include "robot_msgs/GameBuff.h"
#include "robot_msgs/GameZone.h"
#include "robot_msgs/RobotStatus.h"
#include "robot_msgs/RobotDamage.h"
#include "robot_msgs/ArmorDetection.h"
#include <geometry_msgs/PoseStamped.h>
#include "robot_msgs/GlobalMap.h"
#include "robot_msgs/RobotLocation.h"
// #include "robot_msgs/ZMQdata.h"
#include "robot_msgs/SentryData.h"
#include "robot_msgs/GimbalAngle.h"
#include "robot_msgs/TeamInfo.h"
#include "robot_msgs/OdomGoalMsg.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "robot_msgs/SharePath.h"
// #include "robot_localization/Tag.h"

#include <mutex>

namespace robot_decision
{

    class Game_status
    {
    public:
        bool received_;
        std::mutex mutex_;
        int game_status;
        int remaining_time;
        Game_status() : received_(false) {}
        void Init()
        {}
        void GameStatusCallback(const robot_msgs::GameStatus::ConstPtr msg)
        {
            mutex_.lock();
            game_status = msg->game_status;
            remaining_time = msg->remaining_time;
            received_ = true;
            // std::cout<<"game_status: "<<game_status<<std::endl;
            // std::cout<<"remaining_time: "<<remaining_time<<std::endl;
            mutex_.unlock();
        }
    };

    class Game_result
    {
    public:
        bool received_;
        std::mutex mutex_;
        Game_result() : received_(false) {}
        void Init() {}
        void GameResultCallback(const robot_msgs::GameResult::ConstPtr msg)
        {
            mutex_.lock();
            received_ = true;
            // std::cout<<"game_result: "<<received_<<std::endl;
            mutex_.unlock();
        }
    };

    class Game_robot_HP
    {
    public:
        bool received_;
        std::mutex mutex_;
        int *self_id_ = NULL;
        int self_hp_;
        int teammate_hp_;
        int enemy_hp1_;
        int enemy_hp2_;
        Game_robot_HP() : received_(false) {}
        void Init(int *self_id) { self_id_ = self_id; }
        void GameRobotHPCallback(const robot_msgs::GameRobotHP::ConstPtr msg)
        {
            mutex_.lock();
            if (*self_id_ == 1)
            {
                self_hp_ = msg->red1;
                teammate_hp_ = msg->red2;
                enemy_hp1_ = msg->blue1;
                enemy_hp2_ = msg->blue2;
            }
            else if (*self_id_ == 2)
            {
                self_hp_ = msg->red2;
                teammate_hp_ = msg->red1;
                enemy_hp1_ = msg->blue1;
                enemy_hp2_ = msg->blue2;
            }
            else if (*self_id_ == 101)
            {
                self_hp_ = msg->blue1;
                teammate_hp_ = msg->blue2;
                enemy_hp1_ = msg->red1;
                enemy_hp2_ = msg->red2;
            }
            else if (*self_id_ == 102)
            {
                self_hp_ = msg->blue2;
                teammate_hp_ = msg->blue1;
                enemy_hp1_ = msg->red1;
                enemy_hp2_ = msg->red2;
            }
            received_ = true;
            mutex_.unlock();
        }

    };

    class Game_buff
    {
    public:
        int type_[6];
        int active_[6];
        bool received_;
        std::mutex mutex_;
        int *self_id_ = NULL;
        bool self_blood_buff_status_;
        bool self_bullet_buff_status_;
        bool enemy_blood_buff_status_;
        bool enemy_bullet_buff_status_;
        bool no_shoot_status_;
        bool no_move_status_;
        int self_blood_buff_id_;
        int self_bullet_buff_id_;
        int enemy_blood_buff_id_;
        int enemy_bullet_buff_id_;
        int no_shoot_id_;
        int no_move_id_;
        int remain_bullets_num_;
        int teammate_remain_bullets_num_;
        int enemy_remain_bullets_;
        int enemy1_remaining_bullets_;
        int enemy2_remaining_bullets_;
        Game_buff() : received_(false),
                      self_blood_buff_status_(false),
                      self_bullet_buff_status_(false),
                      enemy_blood_buff_status_(false),
                      enemy_bullet_buff_status_(false),
                      no_shoot_status_(false),
                      no_move_status_(false) {}
        void Init(int *self_id)
        {
            self_id_ = self_id;
        }
        void GameBuffCallback(const robot_msgs::GameBuff::ConstPtr msg)
        {
            mutex_.lock();
            if (*self_id_ < 100)
            {
                for (int i = 0; i < 6; i++)
                {
                    if (msg->zone[i].type == 3)
                    {
                        enemy_blood_buff_status_ = msg->zone[i].active;
                        enemy_blood_buff_id_ = i;
                        type_[i] = msg->zone[i].type;
                        active_[i] = msg->zone[i].active;
                    }
                    else if (msg->zone[i].type == 4)
                    {
                        enemy_bullet_buff_status_ = msg->zone[i].active;
                        enemy_bullet_buff_id_ = i;
                        type_[i] = msg->zone[i].type;
                        active_[i] = msg->zone[i].active;
                    }
                    else if (msg->zone[i].type == 1)
                    {
                        self_blood_buff_status_ = msg->zone[i].active;
                        self_blood_buff_id_ = i;
                        type_[i] = msg->zone[i].type;
                        active_[i] = msg->zone[i].active;
                    }
                    else if (msg->zone[i].type == 2)
                    {
                        self_bullet_buff_status_ = msg->zone[i].active;
                        self_bullet_buff_id_ = i;
                        type_[i] = msg->zone[i].type;
                        active_[i] = msg->zone[i].active;
                    }
                    else if (msg->zone[i].type == 5)
                    {
                        no_shoot_status_ = msg->zone[i].active;
                        no_shoot_id_ = i;
                        type_[i] = msg->zone[i].type;
                        active_[i] = msg->zone[i].active;
                    }
                    else
                    {
                        no_move_status_ = msg->zone[i].active;
                        no_move_id_ = i;
                        type_[i] = msg->zone[i].type;
                        active_[i] = msg->zone[i].active;
                    }
                }
            }
            else
            {
                for (int i = 0; i < 6; i++)
                {
                    if (msg->zone[i].type == 1)
                    {
                        enemy_blood_buff_status_ = msg->zone[i].active;
                        enemy_blood_buff_id_ = i;
                        type_[i] = msg->zone[i].type;
                        active_[i] = msg->zone[i].active;
                    }
                    else if (msg->zone[i].type == 2)
                    {
                        enemy_bullet_buff_status_ = msg->zone[i].active;
                        enemy_bullet_buff_id_ = i;
                        type_[i] = msg->zone[i].type;
                        active_[i] = msg->zone[i].active;
                    }
                    else if (msg->zone[i].type == 3)
                    {
                        self_blood_buff_status_ = msg->zone[i].active;
                        self_blood_buff_id_ = i;
                        type_[i] = msg->zone[i].type;
                        active_[i] = msg->zone[i].active;
                    }
                    else if (msg->zone[i].type == 4)
                    {
                        self_bullet_buff_status_ = msg->zone[i].active;
                        self_bullet_buff_id_ = i;
                        type_[i] = msg->zone[i].type;
                        active_[i] = msg->zone[i].active;
                    }
                    else if (msg->zone[i].type == 5)
                    {
                        no_shoot_status_ = msg->zone[i].active;
                        no_shoot_id_ = i;
                        type_[i] = msg->zone[i].type;
                        active_[i] = msg->zone[i].active;
                    }
                    else
                    {
                        no_move_status_ = msg->zone[i].active;
                        no_move_id_ = i;
                        type_[i] = msg->zone[i].type;
                        active_[i] = msg->zone[i].active;
                    }
                }
            }
            if (*self_id_ == 1)
            {
                remain_bullets_num_ = msg->red1;
                teammate_remain_bullets_num_ = msg->red2;
                enemy1_remaining_bullets_ = msg->blue1;
                enemy2_remaining_bullets_ = msg->blue2;
                enemy_remain_bullets_ = msg->blue1 + msg->blue2;
            }
            else if (*self_id_ == 2)
            {
                remain_bullets_num_ = msg->red2;
                teammate_remain_bullets_num_ = msg->red1;
                enemy1_remaining_bullets_ = msg->blue1;
                enemy2_remaining_bullets_ = msg->blue2;
                enemy_remain_bullets_ = msg->blue1 + msg->blue2;
            }
            else if (*self_id_ == 101)
            {
                remain_bullets_num_ = msg->blue1;
                teammate_remain_bullets_num_ = msg->blue2;
                enemy1_remaining_bullets_ = msg->red1;
                enemy2_remaining_bullets_ = msg->red2;
                enemy_remain_bullets_ = msg->red1 + msg->red2;
            }
            else if (*self_id_ == 102)
            {
                remain_bullets_num_ = msg->blue2;
                teammate_remain_bullets_num_ = msg->blue1;
                enemy1_remaining_bullets_ = msg->red1;
                enemy2_remaining_bullets_ = msg->red2;
                enemy_remain_bullets_ = msg->red1 + msg->red2;
            }
            received_ = true;
            mutex_.unlock();
        }
    };

    class Robot_status
    {
        public:  
        bool received_;
        std::mutex mutex_;
        int*self_id_=NULL;
        int max_hp;
        bool shooter_enable;
        Robot_status() : received_(false), shooter_enable(false){}
        void Init(int* self_id)
        {
            self_id_ = self_id;
        }
        void RobotStatusCallback(const robot_msgs::RobotStatus::ConstPtr msg)
        {
            mutex_.lock();
            *self_id_ =msg->id;
            max_hp = msg->max_hp;
            shooter_enable =msg->shooter_enable;
            received_ = true;
            mutex_.unlock();
        }
    };

    class Robot_damage
    {
    public:
        bool received_;
        std::mutex mutex_;
        int armor_hit_cnt_;
        int damage_source_;
        int damage_type_;
        Robot_damage() : received_(false), armor_hit_cnt_(0) {}
        void Init() {}
        void RobotDamageCallback(const robot_msgs::RobotDamage::ConstPtr msg)
        {
            mutex_.lock();
            damage_type_ = msg->damage_type;
            if (damage_type_ == 0)
            {
                armor_hit_cnt_++;
                damage_source_ = msg->damage_source;
            }
            received_ = true;
            mutex_.unlock();
        }
    };

    class Robot_pose
    {
    public:
        bool received_;
        std::mutex mutex_;
        cv::Point3d *self_pose_ = NULL;
        Robot_pose() {}
        void Init(cv::Point3d *self_pose)
        {
            self_pose_ = self_pose;
            received_ = false;
        }
        void EskfPoseCallback(const nav_msgs::Odometry::ConstPtr msg)
        {
            mutex_.lock();
            self_pose_->x = msg->pose.pose.position.x;
            self_pose_->y = msg->pose.pose.position.y;
            self_pose_->z = tf::getYaw(msg->pose.pose.orientation);
            received_ = true;
            // std::cout<<"self_pose_x: "<<self_pose_->x<<std::endl;
            // std::cout<<"self_pose_y: "<<self_pose_->y<<std::endl;
            // std::cout<<"self_pose_z: "<<self_pose_->z<<std::endl;
            mutex_.unlock();
        }

    private:
    };

    class Armor_detection
    {
    public:
        bool received_;
        bool detected_enemy;
        int detected_id_;
        std::mutex mutex_;
        double *gimbal_global_angle_ = NULL;
        double distance;
        cv::Point3d *self_pose_ = NULL;
        cv::Point2f detected_enemy_pose_;
        Armor_detection() : received_(false), detected_enemy(false) {}
        void Init(double *gimbal_global_angle, cv::Point3d *self_pose)
        {
            gimbal_global_angle_ = gimbal_global_angle;
            self_pose_ = self_pose;
        }
        void ArmorDetectionCallback(const robot_msgs::ArmorDetection::ConstPtr msg)
        {
            mutex_.lock();
            detected_enemy = msg->detected_enemy;
            if (msg->detected_enemy == true)
            {
                detected_id_ = msg->id;
                detected_enemy_pose_.x = msg->enemy_x;
                detected_enemy_pose_.y = msg->enemy_y;
                distance = msg->distance;
            }
            received_ = true;
            mutex_.unlock();
        }
    };

    class Global_map
    {
    public:
        bool received_;
        int *self_id_ = NULL;
        bool trust_;
        std::mutex mutex_;
        cv::Point3d sentry_enemy_pose_[2];
        cv::Point3d sentry_die_pose_[2];
        // cv::Point3d *teammate_pose_;
        Global_map() : received_(false), trust_(true) {}
        void Init(int *self_id, cv::Point3d *teammate_pose)
        {
            self_id_ = self_id;
            // teammate_pose_ = teammate_pose;
        }
        void GlobalMapCallback(const robot_msgs::GlobalMap::ConstPtr msg)
        {
            mutex_.lock();
            received_ = true;
            trust_ = msg->is_sentry_on;
            for (int i = 0; i < 6; i++)
            {
                if (*self_id_ > 100)
                {
                    switch (i)
                    {
                    case 0:
                        sentry_enemy_pose_[0].x = msg->msg2decision[i].x;
                        sentry_enemy_pose_[0].y = msg->msg2decision[i].y;
                        sentry_enemy_pose_[0].z = msg->msg2decision[i].yaw;
                        break;
                    case 1:
                        sentry_enemy_pose_[1].x = msg->msg2decision[i].x;
                        sentry_enemy_pose_[1].y = msg->msg2decision[i].y;
                        sentry_enemy_pose_[1].z = msg->msg2decision[i].yaw;
                        break;
                    // case 2:
                    //     if(*self_id_ == 101)
                    //     {
                    //         break;
                    //     }
                    //     teammate_pose_->x = msg->msg2decision[i].x;
                    //     teammate_pose_->y = msg->msg2decision[i].y;
                    //     teammate_pose_->z = msg->msg2decision[i].yaw;
                    //     break;
                    // case 3:
                    //     if(*self_id_ == 102)
                    //     {
                    //         break;
                    //     }
                    //     teammate_pose_->x = msg->msg2decision[i].x;
                    //     teammate_pose_->y = msg->msg2decision[i].y;
                    //     teammate_pose_->z = msg->msg2decision[i].yaw;
                    //     break;
                    case 4:
                        sentry_die_pose_[0].x = msg->msg2decision[i].x;
                        sentry_die_pose_[0].y = msg->msg2decision[i].y;
                        sentry_die_pose_[0].z = msg->msg2decision[i].yaw;
                        break;
                    case 5:
                        sentry_die_pose_[1].x = msg->msg2decision[i].x;
                        sentry_die_pose_[1].y = msg->msg2decision[i].y;
                        sentry_die_pose_[1].z = msg->msg2decision[i].yaw;
                        break;
                    default:
                        break;
                    }
                }
                else
                {
                    switch (i)
                    {
                    // case 0:
                    //     if(*self_id_ == 1)
                    //     {
                    //         break;
                    //     }
                    //     // teammate_pose_->x = msg->msg2decision[i].x;
                    //     // teammate_pose_->y = msg->msg2decision[i].y;
                    //     // teammate_pose_->z = msg->msg2decision[i].yaw;
                    // case 1:
                    //     if(*self_id_ == 2)
                    //     {
                    //         break;
                    //     }
                    //     // teammate_pose_->x = msg->msg2decision[i].x;
                    //     // teammate_pose_->y = msg->msg2decision[i].y;
                    //     // teammate_pose_->z = msg->msg2decision[i].yaw;
                    case 2:
                        sentry_enemy_pose_[0].x = msg->msg2decision[i].x;
                        sentry_enemy_pose_[0].y = msg->msg2decision[i].y;
                        sentry_enemy_pose_[0].z = msg->msg2decision[i].yaw;
                        break;
                    case 3:
                        sentry_enemy_pose_[1].x = msg->msg2decision[i].x;
                        sentry_enemy_pose_[1].y = msg->msg2decision[i].y;
                        sentry_enemy_pose_[1].z = msg->msg2decision[i].yaw;
                        break;
                    case 4:
                        sentry_die_pose_[0].x = msg->msg2decision[i].x;
                        sentry_die_pose_[0].y = msg->msg2decision[i].y;
                        sentry_die_pose_[0].z = msg->msg2decision[i].yaw;
                        break;
                    case 5:
                        sentry_die_pose_[1].x = msg->msg2decision[i].x;
                        sentry_die_pose_[1].y = msg->msg2decision[i].y;
                        sentry_die_pose_[1].z = msg->msg2decision[i].yaw;
                        break;
                    default:
                        break;
                    }
                }
            }
            mutex_.unlock();
        }
    };

    class Gimbal_angle
    {
    public:
        bool received_;
        std::mutex mutex_;
        int bias_count_;
        double *gimbal_global_angle_ = NULL;
        double gimbal_global_angle_bias_;
        cv::Point3d *self_pose_ = NULL;
        Gimbal_angle() : received_(false) {}
        void Init(double *gimbal_global_angle, cv::Point3d *self_pose)
        {
            gimbal_global_angle_ = gimbal_global_angle;
            self_pose_ = self_pose;
        }
        void GimbalAngleCallback(const robot_msgs::GimbalAngle::ConstPtr msg)
        {
            mutex_.lock();
            if (std::fabs(msg->yaw_ecd_angle) < 2.0)
            {
                bias_count_++;
                if (bias_count_ > 250)
                { //校准云台偏置
                    bias_count_ = 0;
                    gimbal_global_angle_bias_ = msg->yaw_angle - self_pose_->z * 180.0 / 3.14159265357 - msg->yaw_ecd_angle;
                }
            }
            else
                bias_count_ = 0;
            double delta = msg->yaw_angle - gimbal_global_angle_bias_;
            int round = delta / 360.0;
            delta = delta - round * 360.0;
            if (delta >= 180.0)
                *gimbal_global_angle_ = -(360.0 - delta) * 3.14159265357 / 180.0;
            else if (delta <= -180.0)
                *gimbal_global_angle_ = (360.0 + delta) * 3.14159265357 / 180.0;
            else
                *gimbal_global_angle_ = delta * 3.14159265357 / 180.0;
            received_ = true;
            mutex_.unlock();
        }
    };

    class TeamPose
    {
    public:
        TeamPose() : received_(false) {}
        ~TeamPose(){}

        bool received_;
        std::mutex mutex_;
        cv::Point3d *teammate_pose_;
        void Init(cv::Point3d *teammate_pose)
        {
            teammate_pose_ = teammate_pose;
        }
        void Callback(const robot_msgs::SharePathConstPtr& msg)
        {
            mutex_.lock();
            received_ = true;
            *teammate_pose_ = cv::Point3d(msg->pos_x[0]/100.0, msg->pos_y[0]/100.0, 0);
            mutex_.unlock();
        }
    };

    class Team_info
    {
    public:
        bool received_;
        std::mutex mutex_;
        int to_buff;
        int attack_enemy;
        int team_mode;
        cv::Point3d teammate_enemy_pose_;
        Team_info() : received_(false) {}
        void Init()
        {}
        void TeamInfoCallback(const robot_msgs::TeamInfo::ConstPtr msg)
        {
            mutex_.lock();
            to_buff = msg->to_buff;
            attack_enemy = msg->attack_enemy;
            team_mode = msg->being_attacked;//更改为，team_mode为1时表示队友主攻，自己需要辅助；其余情况各自进攻
            teammate_enemy_pose_.x = msg->pose_x_enemy / 100.0;
            teammate_enemy_pose_.y = msg->pose_y_enemy / 100.0;
            received_ = true;
            mutex_.unlock();
        }
    };

    class Planning_info
    {
    public:
        bool received_;
        bool flag;
        std::mutex mutex_;
        int *time_cnt_ = NULL;
        double no_go_points[5][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
        Planning_info() : received_(false), flag(false) {}
        void Init(int *time_cnt)
        {
            time_cnt_ = time_cnt;
        }
        void PlanningInfoCallback(const robot_msgs::OdomGoalMsg::ConstPtr msg)
        {
            mutex_.lock();
            flag = msg->mode;
            if (msg->mode == 0)
            {
                no_go_points[*time_cnt_][0] = 0;
                no_go_points[*time_cnt_][1] = 0;
            }
            else
            {
                no_go_points[*time_cnt_][0] = msg->x;
                no_go_points[*time_cnt_][1] = msg->y;
            }
            received_ = true;
            mutex_.unlock();
        }
    };

}

#endif