#ifndef BLACKBOARD_H
#define BLACKBOARD_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>
#include "robot_msgs/GameStatus.h"
#include "robot_msgs/GameResult.h"
#include "robot_msgs/GameRobotHP.h"
#include "robot_msgs/GameBuff.h"
#include "robot_msgs/RobotStatus.h"
#include "robot_msgs/RobotDamage.h"
#include "robot_msgs/ArmorDetection.h"
#include <geometry_msgs/PoseStamped.h>
#include "robot_msgs/ZMQdata.h"
#include "robot_msgs/GimbalAngle.h"
#include "robot_msgs/TeamInfo.h"

#include <mutex>

namespace robot_decision
{
    class Blackboard
    {
        public:
        typedef std::shared_ptr<Blackboard> Ptr;
        Blackboard():
            game_status_received_(false),
            game_result_received_(false),
            game_robot_HP_received_(false),
            game_buff_received_(false),
            robot_status_received_(false),
            robot_damage_received_(false),
            robot_pose_received_(false),
            armor_detection_received_(false),
            sentry_pose_received_(false),
            gimble_angle_received_(false),
            team_info_received_(false)
        {
            game_status_sub_ = nh_.subscribe("game_status", 1, &Blackboard::GameStatusCallback, this);
            game_result_sub_ = nh_.subscribe("game_result", 1, &Blackboard::GameResultCallback, this);
            game_robot_HP_sub_ = nh_.subscribe("game_robot_HP", 1, &Blackboard::GameRobotHPCallback, this);
            game_event_sub_ = nh_.subscribe("game_buff", 1, &Blackboard::GameBuffCallback, this);
            robot_status_sub_ = nh_.subscribe("robot_status", 1, &Blackboard::RobotStatusCallback, this);
            robot_damage_sub_ = nh_.subscribe("robot_damage", 1, &Blackboard::RobotDamageCallback, this);
            robot_pose_sub_ = nh_.subscribe("eskf_pose", 1, &Blackboard::EskfPoseCallback, this);
            armor_detection_sub_ = nh_.subscribe("armor_detection_info", 1, &Blackboard::ArmorDetectionCallback, this);
            sentry_pose_sub_ = nh_.subscribe("robot_zmq_pose", 1, &Blackboard::SentryPoseCallback, this);
            gimbal_angle_sub_ = nh_.subscribe("gimbal_angle_info", 1, &Blackboard::GimbalAngleCallback, this);
            team_info_sub_ = nh_.subscribe("team_info", 1, &Blackboard::TeamInfoCallback, this);
            
            nh_.param("loop_rate", loop_rate_, 5);
            nh_.param("go_buff_time", go_buff_time_, 8);
            nh_.param("force_add_blood", force_add_blood_, 0.25);
            nh_.param("force_add_bullet", force_add_bullet_, 10);
            nh_.param("search_points", search_points_, 16);
            nh_.param("fight_distance", fight_distance_, 1.8);
            nh_.param("back_distance", back_distance_, 1.25);
            nh_.param("attack_distance", attack_distance_, 2.0);
            nh_.param("force_attack_distance", force_attack_distance_, 2.5);
            
            last_patrol_point_ = 6;
            self_blood_buff_status_ = false;
            self_bullet_buff_status_ = false;
            enemy_blood_buff_status_ = false;
        }
        ~Blackboard() {}
        robot_msgs::GameStatus game_status_;
        robot_msgs::GameResult game_result_;
        robot_msgs::GameRobotHP game_robot_HP_;
        robot_msgs::GameBuff game_buff_;
        robot_msgs::RobotStatus robot_status_;
        robot_msgs::RobotDamage robot_damage_;
        geometry_msgs::PoseStamped robot_pose_;
        robot_msgs::ArmorDetection armor_detection_;
        robot_msgs::ZMQdata sentry_pose_;
        robot_msgs::GimbalAngle gimbal_angle_;
        robot_msgs::TeamInfo team_info_;
        bool game_status_received_;
        bool game_result_received_;
        bool game_robot_HP_received_;
        bool game_buff_received_;
        bool robot_status_received_;
        bool robot_damage_received_;
        bool robot_pose_received_;
        bool armor_detection_received_;
        bool sentry_pose_received_;
        bool gimble_angle_received_;
        bool team_info_received_;

        
        int loop_rate_;
        int go_buff_time_;
        double force_add_blood_;
        int force_add_bullet_;
        int search_points_;
        double fight_distance_;
        double back_distance_;
        double attack_distance_;
        double force_attack_distance_;


        cv::Point3d self_pose_;
        cv::Point3d teammate_pose_;
        std::vector<cv::Point2f> buff_pos_ = {
            {0.5,2.79},
            {1.9,1.65},
            {4.04,4.035},
            {4.04,0.445},
            {6.18,2.83},
            {7.58,1.69}
        };
        double barries_pos[9][4] = {{0.0, 1.0, 3.48, 3.28}, 
                                    {1.50, 1.70, 1.0, 0.0},
                                    {3.54, 4.540, 3.545, 3.345},
                                    {3.540, 4.540, 1.135, 0.935},
                                    {6.380, 6.580, 4.48, 3.48},
                                    {7.080, 8.080, 1.2, 1.0},
                                    {1.50, 2.30, 2.34, 2.14},
                                    {3.863, 4.217, 2.417, 2.063},
                                    {5.780, 6.580, 2.34, 2.14}};
        double patrol_pos[6][3] = {{0.90, 1.70, 0},
                                    {2.50, 0.60, 0},
                                    {6.48, 0.88, 3.14},
                                    {7.18, 2.78, 3.14},
                                    {5.58, 3.88, 3.14},
                                    {1.60, 3.60, 0}};
        int last_patrol_point_;
        bool self_blood_buff_status_;
        bool self_bullet_buff_status_;
        bool enemy_blood_buff_status_;
        bool enemy_bullet_buff_status_;
        bool no_shoot_status_;
        int self_blood_buff_id_;
        int self_bullet_buff_id_;
        int enemy_blood_buff_id_;
        int enemy_bullet_buff_id_;
        int no_shoot_id_;
        std::vector<cv::Point3d> enemy_pose_;
        double gimbal_global_angle_;
        double gimbal_global_angle_bias_;
        int bias_count_;
        int armor_hit_cnt_ = 0;
        cv::Point2f detected_enemy_pose_;
        cv::Point2f sentry_enemy_pose_[2];
        cv::Point2f sentry_die_pose[2];
        int self_hp_;
        int teammate_hp_;
        int enemy_hp1_;
        int enemy_hp2_;
        int remain_bullets_num_;
        int teammate_remain_bullets_num_;
        int enemy_remain_bullets_;

        bool line_barriers_check(double *l1, double*l2, int &id)
        {
            double inflate_radius = 0.25;
            for(int i = 0; i < sizeof(barries_pos)/sizeof(barries_pos[0]); i++)
            {
                double sq[4] = {barries_pos[i][0]-inflate_radius, barries_pos[i][2]+inflate_radius, 
                                barries_pos[i][1]+inflate_radius, barries_pos[i][3]-inflate_radius};
                if(line_rect_check(l1,l2,sq))
                {
                    id = i;
                    return true;
                }
            }
            return false;
        }
        void reset_flag()
        {
            game_status_received_ = false;
            game_result_received_ = false;
            game_robot_HP_received_ = false;
            game_buff_received_ = false;
            robot_status_received_ = false;
            robot_damage_received_ = false;
            robot_pose_received_ = false;
            armor_detection_received_ = false;
            sentry_pose_received_ = false;
            gimble_angle_received_ = false;
            team_info_received_ = false;
            team_info_.to_buff = 6;
        }

        std::mutex game_status_cbk_mutex_;
        std::mutex game_result_cbk_mutex_;
        std::mutex game_robot_hp_cbk_mutex_;
        std::mutex game_buff_cbk_mutex_;
        std::mutex robot_status_cbk_mutex_;
        std::mutex robot_damage_cbk_mutex_;
        std::mutex eskf_pose_cbk_mutex_;
        std::mutex armor_detect_cbk_mutex_;
        std::mutex sentry_pose_cbk_mutex_;
        std::mutex gimbal_angle_cbk_mutex_;
        std::mutex team_info_cbk_mutex;

        private:
        void GameStatusCallback(const robot_msgs::GameStatus::ConstPtr msg)
        {
            game_status_cbk_mutex_.lock();
            game_status_ = *msg;
            game_status_received_ = true;
            game_status_cbk_mutex_.unlock();
        }
        void GameResultCallback(const robot_msgs::GameResult::ConstPtr msg)
        {
            game_result_cbk_mutex_.lock();
            game_result_ = *msg;
            game_result_received_ = true;
            game_result_cbk_mutex_.unlock();
        }
        void GameRobotHPCallback(const robot_msgs::GameRobotHP::ConstPtr msg)
        {
            game_robot_hp_cbk_mutex_.lock();
            game_robot_HP_ = *msg;
            if(robot_status_.id == 1)
            {
                self_hp_ = game_robot_HP_.red1;
                teammate_hp_ = game_robot_HP_.red2;
                enemy_hp1_ = game_robot_HP_.blue1;
                enemy_hp2_ = game_robot_HP_.blue2;
            }
            else if(robot_status_.id == 2)
            {
                self_hp_ = game_robot_HP_.red2;
                teammate_hp_ = game_robot_HP_.red1;
                enemy_hp1_ = game_robot_HP_.blue1;
                enemy_hp2_ = game_robot_HP_.blue2;
            }
            else if(robot_status_.id == 101)
            {
                self_hp_ = game_robot_HP_.blue1;
                teammate_hp_ = game_robot_HP_.blue2;
                enemy_hp1_ = game_robot_HP_.red1;
                enemy_hp2_ = game_robot_HP_.red2;
            }
            else if(robot_status_.id == 102)
            {
                self_hp_ = game_robot_HP_.blue2;
                teammate_hp_ = game_robot_HP_.blue1;
                enemy_hp1_ = game_robot_HP_.red1;
                enemy_hp2_ = game_robot_HP_.red2;
            }
            game_robot_HP_received_ = true;
            game_robot_hp_cbk_mutex_.unlock();
        }
        void GameBuffCallback(const robot_msgs::GameBuff::ConstPtr msg)
        {
            game_buff_cbk_mutex_.lock();
            game_buff_ = *msg;
            if(robot_status_.id < 100)
            {
                for(int i = 0; i < 6; i++)
                {
                    if(game_buff_.zone[i].type == 3)
                    {
                        enemy_blood_buff_status_ = game_buff_.zone[i].active;
                        enemy_blood_buff_id_ = i;
                    }
                    else if(game_buff_.zone[i].type == 4)
                    {
                        enemy_bullet_buff_status_ = game_buff_.zone[i].active;
                        enemy_bullet_buff_id_ = i;
                    }
                    else if(game_buff_.zone[i].type == 1)
                    {
                        self_blood_buff_status_ = game_buff_.zone[i].active;
                        self_blood_buff_id_ = i;
                    }
                    else if(game_buff_.zone[i].type == 2)
                    {
                        self_bullet_buff_status_ = game_buff_.zone[i].active;
                        self_bullet_buff_id_ = i;
                    }
                    else if(game_buff_.zone[i].type == 5)
                    {
                        no_shoot_status_ = game_buff_.zone[i].active;
                        no_shoot_id_ = i;
                    }
                }
            }
            else
            {
                for(int i = 0; i < 6; i++)
                {
                    if(game_buff_.zone[i].type == 1)
                    {
                        enemy_blood_buff_status_ = game_buff_.zone[i].active;
                        enemy_blood_buff_id_ = i;
                    }
                    else if(game_buff_.zone[i].type == 2)
                    {
                        enemy_bullet_buff_status_ = game_buff_.zone[i].active;
                        enemy_bullet_buff_id_ = i;
                    }
                    else if(game_buff_.zone[i].type == 3)
                    {
                        self_blood_buff_status_ = game_buff_.zone[i].active;
                        self_blood_buff_id_ = i;
                    }
                    else if(game_buff_.zone[i].type == 4)
                    {
                        self_bullet_buff_status_ = game_buff_.zone[i].active;
                        self_bullet_buff_id_ = i;
                    }
                    else if(game_buff_.zone[i].type == 5)
                    {
                        no_shoot_status_ = game_buff_.zone[i].active;
                        no_shoot_id_ = i;
                    }
                }
            }
            if(robot_status_.id == 1)
            {
                remain_bullets_num_ = game_buff_.red1;
                teammate_remain_bullets_num_ = game_buff_.red2;
                enemy_remain_bullets_ = game_buff_.blue1 + game_buff_.blue2;
            }
            else if(robot_status_.id == 2)
            {
                remain_bullets_num_ = game_buff_.red2;
                teammate_remain_bullets_num_ = game_buff_.red1;
                enemy_remain_bullets_ = game_buff_.blue1 + game_buff_.blue2;
            }
            else if(robot_status_.id == 101)
            {
                remain_bullets_num_ = game_buff_.blue1;
                teammate_remain_bullets_num_ = game_buff_.blue2;
                enemy_remain_bullets_ = game_buff_.red1 + game_buff_.red2;
            }
            else if(robot_status_.id == 102)
            {
                remain_bullets_num_ = game_buff_.blue2;
                teammate_remain_bullets_num_ = game_buff_.blue1;
                enemy_remain_bullets_ = game_buff_.red1 + game_buff_.red2;
            }
            game_buff_received_ = true;
            game_buff_cbk_mutex_.unlock();
        }
        void RobotStatusCallback(const robot_msgs::RobotStatus::ConstPtr msg)
        {
            robot_status_cbk_mutex_.lock();
            robot_status_ = *msg;
            robot_status_received_ = true;
            robot_status_cbk_mutex_.unlock();
        }
        void RobotDamageCallback(const robot_msgs::RobotDamage::ConstPtr msg)
        {
            robot_damage_cbk_mutex_.lock();
            robot_damage_ = *msg;
            if(robot_damage_.damage_type == 0)
            {
                armor_hit_cnt_++;
            }
            robot_damage_received_ = true;
            robot_damage_cbk_mutex_.unlock();
        }
        void EskfPoseCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
        {
            eskf_pose_cbk_mutex_.lock();
            robot_pose_ = *msg;
            self_pose_.x = robot_pose_.pose.position.x;
            self_pose_.y = robot_pose_.pose.position.y;
            self_pose_.z = tf::getYaw(robot_pose_.pose.orientation);
            robot_pose_received_ = true;
            eskf_pose_cbk_mutex_.unlock();
        }
        void ArmorDetectionCallback(const robot_msgs::ArmorDetection::ConstPtr msg)
        {
            armor_detect_cbk_mutex_.lock();
            armor_detection_ = *msg;
            if(armor_detection_.detected_enemy == true){
                double enemy_angle = armor_detection_.yaw_angle * 3.14159265357 / 180.0;
                double enemy_x = armor_detection_.distance/1000.0 * cos(enemy_angle) + 0.153;
                double enemy_y = armor_detection_.distance/1000.0 * sin(enemy_angle);
                double angle = gimbal_global_angle_;
                detected_enemy_pose_.x = cos(angle) * enemy_x - sin(angle) * enemy_y + self_pose_.x;
                detected_enemy_pose_.y = sin(angle) * enemy_x + cos(angle) * enemy_y + self_pose_.y;
            }
            armor_detection_received_ = true;
            armor_detect_cbk_mutex_.unlock();
        }
        void SentryPoseCallback(const robot_msgs::ZMQdata::ConstPtr msg)
        {
            sentry_pose_cbk_mutex_.lock();
            sentry_pose_ = *msg;
            if(robot_status_.id < 100)
            {
                // 红方
                sentry_enemy_pose_[0].x = sentry_pose_.robot_pose_x[0] / 100.0;
                sentry_enemy_pose_[0].y = sentry_pose_.robot_pose_y[0] / 100.0;
                sentry_enemy_pose_[1].x = sentry_pose_.robot_pose_x[1] / 100.0;
                sentry_enemy_pose_[1].y = sentry_pose_.robot_pose_y[1] / 100.0;
            }
            else
            {
                // 蓝方
                sentry_enemy_pose_[0].x = sentry_pose_.robot_pose_x[2] / 100.0;
                sentry_enemy_pose_[0].y = sentry_pose_.robot_pose_y[2] / 100.0;
                sentry_enemy_pose_[1].x = sentry_pose_.robot_pose_x[3] / 100.0;
                sentry_enemy_pose_[1].y = sentry_pose_.robot_pose_y[3] / 100.0;
            }
            sentry_die_pose[0].x = sentry_pose_.robot_pose_x[4] / 100.0;
            sentry_die_pose[0].y = sentry_pose_.robot_pose_y[4] / 100.0;
            sentry_die_pose[1].x = sentry_pose_.robot_pose_x[5] / 100.0;
            sentry_die_pose[1].y = sentry_pose_.robot_pose_y[5] / 100.0;
            sentry_pose_received_ = true;
            sentry_pose_cbk_mutex_.unlock();
        }
        void GimbalAngleCallback(const robot_msgs::GimbalAngle::ConstPtr msg)
        {
            gimbal_angle_cbk_mutex_.lock();
            gimbal_angle_ = *msg;
            if(std::fabs(gimbal_angle_.yaw_ecd_angle) < 2.0){
                bias_count_++;
                if(bias_count_ > 250){
                    bias_count_ = 0;
                    gimbal_global_angle_bias_ = gimbal_angle_.yaw_angle - self_pose_.z * 180.0 / 3.14159265357 - gimbal_angle_.yaw_ecd_angle;
                }
            }
            else bias_count_ = 0;
            double delta = gimbal_angle_.yaw_angle - gimbal_global_angle_bias_;
            int round = delta / 360.0;
            delta = delta - round * 360.0;
            if(delta >= 180.0) gimbal_global_angle_ = -(360.0 - delta) * 3.14159265357 / 180.0;
            else if(delta <= -180.0) gimbal_global_angle_ = (360.0 + delta) * 3.14159265357 / 180.0;
            else gimbal_global_angle_ = delta * 3.14159265357 / 180.0;
            gimble_angle_received_ = true;
            gimbal_angle_cbk_mutex_.unlock();
        }
        void TeamInfoCallback(const robot_msgs::TeamInfo::ConstPtr msg)
        {
            team_info_cbk_mutex.lock();
            team_info_ = *msg;
            teammate_pose_.x = team_info_.pose_x_teammate / 100.0;
            teammate_pose_.y = team_info_.pose_y_teammate / 100.0;
            team_info_received_ = true;
            team_info_cbk_mutex.unlock();
        }
        ros::NodeHandle nh_;
        ros::Subscriber game_status_sub_;
        ros::Subscriber game_result_sub_;
        ros::Subscriber game_robot_HP_sub_;
        ros::Subscriber game_event_sub_;
        ros::Subscriber robot_status_sub_;
        ros::Subscriber robot_damage_sub_;
        ros::Subscriber robot_pose_sub_;
        ros::Subscriber armor_detection_sub_;
        ros::Subscriber sentry_pose_sub_;
        ros::Subscriber gimbal_angle_sub_;
        ros::Subscriber team_info_sub_;

        double cross(double* p1, double* p2, double *p3)
        {
            double x1 = p2[0] - p1[0];
            double y1 = p2[1] - p1[1];
            double x2 = p3[0] - p1[0];
            double y2 = p3[1] - p1[1];
            return x1 * y2 - x2 * y1;
        }
        bool segment(double* p1, double* p2, double *p3, double *p4)
        {
            if(std::max(p1[0], p2[0])>=std::min(p3[0], p4[0]) && std::max(p3[0], p4[0])>=std::min(p1[0], p2[0])
            && std::max(p1[1], p2[1])>=std::min(p3[1], p4[1]) && std::max(p3[1], p4[1])>=std::min(p1[1], p2[1]))
            {
                if(cross(p1,p2,p3)*cross(p1,p2,p4)<=0 && cross(p3,p4,p1)*cross(p3,p4,p2)<=0)
                    return true;
                else
                    return false;
            }
            else return false;
        }
        bool line_rect_check(double* l1, double* l2, double *sq)
        {
            double p1[2] = {sq[0], sq[1]};
            double p2[2] = {sq[2], sq[3]};
            double p3[2] = {sq[2], sq[1]};
            double p4[2] = {sq[0], sq[3]};
            if(segment(l1,l2,p1,p2) || segment(l1,l2,p3,p4))
                return true;
            else
                return false;
        }
    };
}

#endif