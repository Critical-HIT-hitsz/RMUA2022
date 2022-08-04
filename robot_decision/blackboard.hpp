#ifndef BLACKBOARD_H
#define BLACKBOARD_H

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
// #include <numeric>
#include <deque>
#include <opencv2/opencv.hpp>
#include <callback.hpp>
#include <mutex>

namespace robot_decision
{
    class Blackboard
    {
        public:
        typedef std::shared_ptr<Blackboard> Ptr;
        Blackboard()
        {
            nh_.param("/robot_decision/loop_rate", loop_rate_, 10);
            nh_.param("/robot_decision/go_buff_time", go_buff_time_, 10);
            nh_.param("/robot_decision/force_add_blood", force_add_blood_, 0.25);
            nh_.param("/robot_decision/force_add_bullet", force_add_bullet_, 10);
            nh_.param("/robot_decision/search_points", search_points_, 16);
            nh_.param("/robot_decision/fight_distance", fight_distance_, 1.8);
            nh_.param("/robot_decision/back_distance", back_distance_, 1.25);
            nh_.param("/robot_decision/attack_distance", attack_distance_, 2.0);
            nh_.param("/robot_decision/force_attack_distance", force_attack_distance_, 2.5);
            nh_.param("/robot_decision/avoid_encircle",avoid_encircle_,1.5);

            game_status_.Init();
            game_result_.Init();
            game_robot_HP_.Init(&self_id_);
            game_buff_.Init(&self_id_);
            robot_status_.Init(&self_id_);
            robot_damage_.Init();
            robot_pose_.Init(&self_pose_);
            armor_detection_.Init(&gimbal_global_angle_, &self_pose_);
            global_map_.Init(&self_id_, &teammate_pose_);
            gimbal_angle_.Init(&gimbal_global_angle_,&self_pose_);
            team_info_.Init();
            planning_info_.Init(&time_cnt_);
            team_path_.Init(&teammate_pose_);

            game_status_sub_ = nh_.subscribe<robot_msgs::GameStatus>("game_status", 1, boost::bind(&Game_status::GameStatusCallback, &game_status_, _1));
            game_result_sub_ = nh_.subscribe<robot_msgs::GameResult>("game_result", 1, boost::bind(&Game_result::GameResultCallback, &game_result_, _1));
            game_robot_HP_sub_ = nh_.subscribe<robot_msgs::GameRobotHP>("game_robot_HP", 1, boost::bind(&Game_robot_HP::GameRobotHPCallback, &game_robot_HP_, _1));
            game_event_sub_ = nh_.subscribe<robot_msgs::GameBuff>("game_buff", 1, boost::bind(&Game_buff::GameBuffCallback, &game_buff_, _1));
            robot_status_sub_ = nh_.subscribe<robot_msgs::RobotStatus>("robot_status", 1, boost::bind(&Robot_status::RobotStatusCallback, &robot_status_, _1));
            robot_damage_sub_ = nh_.subscribe<robot_msgs::RobotDamage>("robot_damage", 1, boost::bind(&Robot_damage::RobotDamageCallback, &robot_damage_, _1));
            robot_pose_sub_ = nh_.subscribe<nav_msgs::Odometry>("odometer", 1, boost::bind(&Robot_pose::EskfPoseCallback, &robot_pose_, _1));
            armor_detection_sub_ = nh_.subscribe<robot_msgs::ArmorDetection>("armor_detection_info", 1, boost::bind(&Armor_detection::ArmorDetectionCallback, &armor_detection_, _1));
            gimbal_angle_sub_ = nh_.subscribe<robot_msgs::GimbalAngle>("gimbal_angle_info", 1, boost::bind(&Gimbal_angle::GimbalAngleCallback, &gimbal_angle_, _1));
            team_info_sub_ = nh_.subscribe<robot_msgs::TeamInfo>("team_info", 1, boost::bind(&Team_info::TeamInfoCallback, &team_info_, _1));
            planning_info_sub_ = nh_.subscribe<robot_msgs::OdomGoalMsg>("planning_info", 1, boost::bind(&Planning_info::PlanningInfoCallback, &planning_info_, _1));
            global_map_sub_ = nh_.subscribe<robot_msgs::GlobalMap>("global_map",1,boost::bind(&Global_map::GlobalMapCallback,&global_map_,_1));
            team_path_sub_ = nh_.subscribe<robot_msgs::SharePath>("share_path", 1, boost::bind(&TeamPose::Callback, &team_path_, _1));

            std::string mappath_ = ros::package::getPath("robot_decision") + "/assets/background.png";
            map_ = cv::imread(mappath_);
            dst_map_ = map_.clone();
            costmap_ = map_.clone();
            time_cnt_ = 0;
            gimbal_global_angle_ = 0;
            self_pose_ = {0,0,0};
            teammate_pose_={0,0,0};
            perception_enemy_flag_ = false;
            sentry_enemy_flag_ = false;
            global_map_trust_ = true;
        }
        ~Blackboard() {}

        Game_status game_status_;
        Game_result game_result_;
        Game_robot_HP game_robot_HP_;
        Game_buff game_buff_;
        Robot_status robot_status_;
        Robot_damage robot_damage_;
        Robot_pose robot_pose_;
        Armor_detection armor_detection_;
        Gimbal_angle gimbal_angle_;
        Team_info team_info_;
        Planning_info planning_info_;
        Global_map global_map_;
        TeamPose team_path_;

        ros::Subscriber game_status_sub_, game_result_sub_, game_robot_HP_sub_,game_event_sub_, 
                        robot_status_sub_, robot_damage_sub_, robot_pose_sub_,armor_detection_sub_, 
                        global_map_sub_, gimbal_angle_sub_, team_info_sub_, planning_info_sub_, team_path_sub_;

        // 决策参数
        int loop_rate_;//Hz
        int go_buff_time_;//s
        double force_add_blood_;//强制加血血量（百分比）
        int force_add_bullet_;//强制加弹弹丸剩余量
        int search_points_;//攻击搜寻点个数
        double fight_distance_;//攻击搜寻点距离,单位m
        double back_distance_;//机器人后退攻击阈值距离,单位m
        double attack_distance_;//机器人前进攻击阈值距离,单位m
        double force_attack_distance_;//机器人强制前进攻击阈值距离,单位m
        double avoid_encircle_;

        cv::Mat map_;
        cv::Mat dst_map_;
        cv::Mat costmap_;
        cv::Point3d self_pose_;
        cv::Point3d teammate_pose_;
        cv::Point3d enemy_pose1_;
        cv::Point3d enemy_pose2_;
        int self_id_;
        double gimbal_global_angle_;
        int time_cnt_;
        bool perception_enemy_flag_;
        bool sentry_enemy_flag_;
        bool global_map_trust_;
        // std::deque<int> perception_enemy1_inRecent10(10);//初始化默认为0
        // std::deque<int> perception_enemy2_inRecent10(10);//初始化默认为0
        // std::deque<int> sentry_enemy1_inRecent10(10);//初始化默认为0
        // std::deque<int> sentry_enemy2_inRecent10(10);//初始化默认为0

        std::vector<cv::Point2f> buff_pos_ = {
            {7.58,1.69},
            {6.18,2.83},
            {4.04,0.45},
            {4.04,4.00},
            {1.90,1.65},
            {0.50,2.79}  
        };
        bool buff_active_[6] = {true,true,true,true,true,true};
        double barries_pos[9][4] = {{0.0, 1.0, 3.48, 3.28},  // x1, x2, y2, y1
                                    {1.50, 1.70, 1.0, 0.0},
                                    {3.54, 4.540, 3.545, 3.345},
                                    {3.540, 4.540, 1.135, 0.935},
                                    {6.380, 6.580, 4.48, 3.48},
                                    {7.080, 8.080, 1.2, 1.0},
                                    {1.50, 2.30, 2.34, 2.14},
                                    {3.863, 4.217, 2.417, 2.063},//中间处理成正方形
                                    {5.780, 6.580, 2.34, 2.14}};
        double barries_expand_pos[9][4] = {{0.0, 1.25, 3.73, 3.03},  // x1, x2, y2, y1
                                    {1.25, 1.95, 1.25, 0.0},
                                    {3.29, 4.790, 3.795, 3.095},
                                    {3.29, 4.790, 1.385, 0.685},
                                    {6.130, 6.830, 4.73, 3.23},
                                    {6.830, 8.330, 1.45, 0.75},
                                    {1.25, 2.55, 2.59, 1.89},
                                    {3.613, 4.467, 2.667, 1.813},//中间处理成正方形
                                    {5.530, 6.830, 2.59, 1.89}};
        double patrol_pos[6][3] = {{0.90, 1.70, 0}, //逆时钟排序
                                    {2.50, 0.60, 0},
                                    {6.48, 0.88, 3.14},
                                    {7.18, 2.78, 3.14},
                                    {5.58, 3.88, 3.14},
                                    {1.60, 3.60, 0}};
        

        bool check_block(double goal_x, double goal_y)
        {
            double dis = 0;
            for(int i=0; i<6; i++)
            {
                if(goal_x >= barries_expand_pos[i][0] && goal_x <= barries_expand_pos[i][1] && goal_y >= barries_expand_pos[i][3] && goal_y <= barries_expand_pos[i][2])//目标坐标在障碍块上则返回true
                {
                    return true;
                }
                    
            }
            for(int i=0; i<6; i++)
            {
                if(buff_active_[i] == true && std::sqrt(std::pow(goal_x-buff_pos_[i].x,2)+std::pow(goal_y-buff_pos_[i].y,2)) < 0.5)
                    return true; 
            }
            for(int i=0; i<5; i++)
            {
                if(planning_info_.no_go_points[i][0] == 0 && planning_info_.no_go_points[i][1] == 0)  
                {
                    continue;
                }
                dis = std::sqrt(std::pow(goal_x-planning_info_.no_go_points[i][0],2)+std::pow(goal_y-planning_info_.no_go_points[i][1],2));
                if(dis < 0.3)
                {
                    return true;
                }   
                else    
                {
                    continue;
                }
            }
            for(int i=0; i<2; i++)
            {
                if(global_map_.received_ == false && global_map_.trust_)  
                {
                    break;
                }
                if(global_map_.sentry_enemy_pose_[i].x != 0 && global_map_.sentry_enemy_pose_[i].y != 0)
                {
                    dis = std::sqrt(std::pow(goal_x-global_map_.sentry_enemy_pose_[i].x,2)+std::pow(goal_y-global_map_.sentry_enemy_pose_[i].y,2));
                    if(dis < 0.5) 
                    {
                        return true;
                    }
                }
                if(global_map_.sentry_die_pose_[i].x != 0 && global_map_.sentry_die_pose_[i].y != 0)
                {
                    dis = std::sqrt(std::pow(goal_x-global_map_.sentry_die_pose_[i].x,2)+std::pow(goal_y-global_map_.sentry_die_pose_[i].y,2));
                    if(dis < 0.5) 
                    {
                        return true;
                    }
                }
            }
            if(teammate_pose_.x != 0 && teammate_pose_.y != 0 && global_map_.received_ && global_map_.trust_)
            {
                dis = std::sqrt(std::pow(goal_x-teammate_pose_.x,2)+std::pow(goal_y-teammate_pose_.y,2));
                if(dis < 0.5) 
                {
                    return true;
                }
            }
            return false;
        }

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
                if(cross(p1,p2,p3)*cross(p1,p2,p4)<=0 && cross(p3,p4,p1)*cross(p3,p4,p2)<=0)//自己与目标的连线是否与矩形对角线相交；向量叉乘法判断两线段是否相交
                    return true;
                else
                    return false;
            }
            else return false;
        }
        // 判断线段是否与矩形相交
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
        // 判断线段是否与障碍块相交
        bool line_barriers_check(double *l1, double*l2, int &id)//l1为自己坐标,l2为目标坐标，并将影响到攻击的障碍块id传回；true为相交,false为不相交
        {
            double inflate_radius = 0.25;
            for(int i = 0; i < 6; i++)
            {
                double sq[4] = {barries_pos[i][0]-inflate_radius, barries_pos[i][2]+inflate_radius, 
                                barries_pos[i][1]+inflate_radius, barries_pos[i][3]-inflate_radius};
                // if(l2[0] >= barries_pos[i][0] && l2[0] <= barries_pos[i][1] && l2[1] >= barries_pos[i][3] && l2[1] <= barries_pos[i][2])//目标坐标在障碍块上则返回true
                //     return true;
                if(line_rect_check(l1,l2,sq))
                {
                    id = i;
                    return true;
                }
            }
            return false;
        }

        bool back_car_check(double *goal_point)//检测目标点后方是否有障碍块保护，或者是否有敌方存在
        {
            if(transfer_to_car_coordinate_obstacle(goal_point) == false)    return false;
            if(global_map_.received_ == false)  return false;
            if(global_map_.sentry_enemy_pose_[0].x > 0 && global_map_.sentry_enemy_pose_[0].x < 8.08 && global_map_.sentry_enemy_pose_[0].y > 0 && global_map_.sentry_enemy_pose_[0].y < 4.48)
            {
                if(transfer_to_car_coordinate_enemy(goal_point, global_map_.sentry_enemy_pose_[0]))
                    return true;
            }
            if(global_map_.sentry_enemy_pose_[1].x > 0 && global_map_.sentry_enemy_pose_[1].x < 8.08 && global_map_.sentry_enemy_pose_[1].y > 0 && global_map_.sentry_enemy_pose_[1].y < 4.48)
            {
                if(transfer_to_car_coordinate_enemy(goal_point, global_map_.sentry_enemy_pose_[1]))
                    return true;
            }
            return false;
        }

        bool transfer_to_car_coordinate_obstacle(double *goal_point)
        {
            for(int i = 0; i < 6; i++)
            {
                double pos_1[2] = {barries_pos[i][0] - goal_point[0], barries_pos[i][3] - goal_point[1]};
                double pos_2[2] = {barries_pos[i][1] - goal_point[0], barries_pos[i][2] - goal_point[1]};
                double new_point_1[2], new_point_2[2];
                new_point_1[0] = pos_1[0] * sin(goal_point[2]) - pos_1[1] * cos(goal_point[2]);
                new_point_1[1] = pos_1[0] * cos(goal_point[2]) + pos_1[1] * sin(goal_point[2]);
                new_point_2[0] = pos_2[0] * sin(goal_point[2]) - pos_2[1] * cos(goal_point[2]);
                new_point_2[1] = pos_2[0] * cos(goal_point[2]) + pos_2[1] * sin(goal_point[2]);
                if(std::fabs(new_point_1[0] > 0.5 || std::fabs(new_point_2[0]) > 0.5))
                    continue;
                if(new_point_1[1] > -0.3 || new_point_1[1] < -0.7 || new_point_2[1] > -0.3 || new_point_2[1] < -0.7)
                    continue;
                if(std::fabs(new_point_1[0] - new_point_2[0] > 0.7))
                    return false;
            }
            return true;
        }

        bool transfer_to_car_coordinate_enemy(double *goal_point, cv::Point3d enemy_point)
        {
            double pos[2] = {enemy_point.x - goal_point[0], enemy_point.y - goal_point[1]};
            double new_enemy_point[2];
            new_enemy_point[0] = pos[0] * sin(goal_point[2]) - pos[1] * cos(goal_point[2]);
            new_enemy_point[1] = pos[0] * cos(goal_point[2]) + pos[1] * sin(goal_point[2]);
            if(std::fabs(new_enemy_point[0]) < 0.5 && new_enemy_point[1] > -2.5 && new_enemy_point[1] < -0.5)
                return true;
            else
                return false;
        }

        bool transfer_to_enemy_coordinate_(double *goal_point, cv::Point3d enemy_point_1, cv::Point3d enemy_point_2)
        {
            if(enemy_point_1.x==0 && enemy_point_1.y==0) 
                return false;
            if(enemy_point_2.x==0 && enemy_point_2.y==0) 
                return false;
            double pos_1[2] = {goal_point[0] - enemy_point_1.x, goal_point[1] - enemy_point_1.y};
            double new_goal_point_1[2];
            new_goal_point_1[0] = pos_1[0] * sin(enemy_point_1.z) - pos_1[1] * cos(enemy_point_1.z);
            new_goal_point_1[1] = pos_1[0] * cos(enemy_point_1.z) + pos_1[1] * sin(enemy_point_1.z);
            double dis_1 = std::sqrt(std::pow(new_goal_point_1[0],2)+std::pow(new_goal_point_1[1],2));
            double dis_weight_1;
            if(dis_1 < 2.5) dis_weight_1 = 1;
            else if(dis_1 >= 2.5 && dis_1 <= 3.5) dis_weight_1 = 0.5*(cos(CV_PI*(dis_1-2)-CV_PI*0.5)+1);
            else return false;

            double pos_2[2] = {goal_point[0] - enemy_point_2.x, goal_point[1] - enemy_point_2.y};
            double new_goal_point_2[2];
            new_goal_point_2[0] = pos_2[0] * sin(enemy_point_2.z) - pos_2[1] * cos(enemy_point_2.z);
            new_goal_point_2[1] = pos_2[0] * cos(enemy_point_2.z) + pos_2[1] * sin(enemy_point_2.z);
            double dis_2 = std::sqrt(std::pow(new_goal_point_2[0],2)+std::pow(new_goal_point_2[1],2));
            double dis_weight_2;
            if(dis_2 > 0.5 && dis_2 < 1.5)  dis_weight_2 = 0.5*(cos(CV_PI*(dis_2-2)+CV_PI*0.5) + 1);
            else if(dis_2 >= 1.5 && dis_2 <= 2.5) dis_weight_2 = 1;
            else if(dis_2 >= 2.5 && dis_2 <= 3.5) dis_weight_2 = 0.5*(cos(CV_PI*(dis_2-2)-CV_PI*0.5)+1);
            else return false;
            
            int id; 
            double enemy_pose_check_1[2]={enemy_point_1.x,enemy_point_1.y};
            double enmey_pose_check_2[2]={enemy_point_2.x,enemy_point_2.x};
            double goal_point_check_[2]={goal_point[0],goal_point[1]};
            if((dis_weight_1+dis_weight_2) > avoid_encircle_ && new_goal_point_1[1] >0 && new_goal_point_2[1] > 0 && !line_barriers_check(goal_point_check_,enemy_pose_check_1,id) && !line_barriers_check(goal_point_check_,enmey_pose_check_2,id))
                return true;
            return false;
        }


        void visualize()
        {
            cv::Mat show_img = map_.clone();
            char str[20];
            int time_ = (int)game_status_.remaining_time;
            sprintf(str, "time: %d", time_);
            int baseLine = 0;
            cv::Size label_size = cv::getTextSize(str, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            cv::putText(show_img, str, cv::Point(0, 0 + label_size.height),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
            draw_chassis(show_img,self_pose_, self_id_);
            draw_gimbal(show_img,cv::Point3d(self_pose_.x,self_pose_.y,gimbal_global_angle_));
            sprintf(str, "%d %d", game_robot_HP_.self_hp_, game_buff_.remain_bullets_num_);
            label_size = cv::getTextSize(str, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            cv::putText(show_img, str, cv::Point(self_pose_.x*100, show_img.rows-1-self_pose_.y*100 + label_size.height),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
            sprintf(str, "%d %d", game_robot_HP_.teammate_hp_, game_buff_.teammate_remain_bullets_num_);
            label_size = cv::getTextSize(str, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            cv::putText(show_img, str, cv::Point(teammate_pose_.x*100, show_img.rows-1-teammate_pose_.y*100 + label_size.height),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
            sprintf(str, "%d %d", game_robot_HP_.enemy_hp1_, game_buff_.enemy1_remaining_bullets_);
            label_size = cv::getTextSize(str, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            cv::putText(show_img, str, cv::Point(global_map_.sentry_enemy_pose_[0].x*100, show_img.rows-1-global_map_.sentry_enemy_pose_[0].y*100 + label_size.height),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
            sprintf(str, "%d %d", game_robot_HP_.enemy_hp2_, game_buff_.enemy2_remaining_bullets_);
            label_size = cv::getTextSize(str, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            cv::putText(show_img, str, cv::Point(global_map_.sentry_enemy_pose_[1].x*100, show_img.rows-1-global_map_.sentry_enemy_pose_[1].y*100 + label_size.height),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
            for(int i = 0; i < 6; i++)
            {
                if(!game_buff_.active_[i])
                    continue;
                switch(game_buff_.type_[i])
                {
                    case 1://red_hp
                        cv::putText(show_img, "HP", cv::Point(buff_pos_[i].x*100, show_img.rows-1- buff_pos_[i].y*100), 
                                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255));
                        break;
                    case 2://red_bullet
                        cv::putText(show_img, "BULLET", cv::Point( buff_pos_[i].x*100, show_img.rows-1- buff_pos_[i].y*100), 
                                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255));
                        break;
                    case 3://blue_hp
                        cv::putText(show_img, "HP", cv::Point( buff_pos_[i].x*100, show_img.rows-1- buff_pos_[i].y*100), 
                                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,0,0));
                        break;
                    case 4://blue_bullet
                        cv::putText(show_img, "BULLET", cv::Point( buff_pos_[i].x*100, show_img.rows-1- buff_pos_[i].y*100), 
                                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,0,0));
                        break;
                    case 5://no_shoot
                        cv::putText(show_img, "S", cv::Point( buff_pos_[i].x*100, show_img.rows-1- buff_pos_[i].y*100), 
                                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0));
                        break;
                    case 6://no_move
                        cv::putText(show_img, "M", cv::Point( buff_pos_[i].x*100, show_img.rows-1- buff_pos_[i].y*100), 
                                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0));
                        break;
                    default:;
                }
            }

            if( self_id_ < 100)
            {
                for(int i = 0; i < 2; i++)
                {
                    if(global_map_.received_ && global_map_.sentry_enemy_pose_[i].x != 0 &&  global_map_.sentry_enemy_pose_[i].y != 0)
                    {
                        cv::circle(show_img, cv::Point( global_map_.sentry_enemy_pose_[i].x*100, show_img.rows-1- global_map_.sentry_enemy_pose_[i].y*100), 30, cv::Scalar(255,0,0), 3);
                        cv::arrowedLine(show_img, cv::Point(global_map_.sentry_enemy_pose_[i].x*100, show_img.rows-1- global_map_.sentry_enemy_pose_[i].y*100),
                                        cv::Point(global_map_.sentry_enemy_pose_[i].x*100+25*cos(global_map_.sentry_enemy_pose_[i].z), show_img.rows-1- global_map_.sentry_enemy_pose_[i].y*100-25*sin(global_map_.sentry_enemy_pose_[i].z)),cv::Scalar(0,0,0),2);
                    }        
                    if(global_map_.received_ && global_map_.sentry_die_pose_[i].x != 0 &&  global_map_.sentry_die_pose_[i].y != 0)
                        cv::circle(show_img, cv::Point( global_map_.sentry_die_pose_[i].x*100, show_img.rows-1- global_map_.sentry_die_pose_[i].y*100), 30, cv::Scalar(0,0,0), 3);
                }
                if(teammate_pose_.x != 0 &&  teammate_pose_.y != 0)
                {
                    cv::arrowedLine(show_img, cv::Point(teammate_pose_.x*100, show_img.rows-1- teammate_pose_.y*100),
                                        cv::Point(teammate_pose_.x*100+25*cos(teammate_pose_.z), show_img.rows-1- teammate_pose_.y*100-25*sin(teammate_pose_.z)),cv::Scalar(0,0,0),2);
                    cv::circle(show_img, cv::Point( teammate_pose_.x*100, show_img.rows-1- teammate_pose_.y*100), 30, cv::Scalar(0,0,255), 3);

                }
                if(team_info_.received_ && team_info_.attack_enemy != 0)
                    cv::circle(show_img, cv::Point(team_info_.teammate_enemy_pose_.x*100 && show_img.rows-1-team_info_.teammate_enemy_pose_.y*100), 5, cv::Scalar(255,0,0), 5);
                if(armor_detection_.received_ && armor_detection_.detected_enemy)
                    cv::circle(show_img, cv::Point( armor_detection_.detected_enemy_pose_.x*100, show_img.rows-1- armor_detection_.detected_enemy_pose_.y*100), 5, cv::Scalar(0,0,0), 5);
            }
            else
            {
                for(int i = 0; i < 2; i++)
                {
                    if(global_map_.received_ && global_map_.sentry_enemy_pose_[i].x != 0 &&  global_map_.sentry_enemy_pose_[i].y != 0)
                    {
                        cv::circle(show_img, cv::Point( global_map_.sentry_enemy_pose_[i].x*100, show_img.rows-1- global_map_.sentry_enemy_pose_[i].y*100), 30, cv::Scalar(0,0,255), 3);
                        cv::arrowedLine(show_img, cv::Point(global_map_.sentry_enemy_pose_[i].x*100, show_img.rows-1- global_map_.sentry_enemy_pose_[i].y*100),
                                        cv::Point(global_map_.sentry_enemy_pose_[i].x*100+25*cos(global_map_.sentry_enemy_pose_[i].z), show_img.rows-1- global_map_.sentry_enemy_pose_[i].y*100-25*sin(global_map_.sentry_enemy_pose_[i].z)),cv::Scalar(0,0,0),2);
                    }
                    if(global_map_.received_ && global_map_.sentry_die_pose_[i].x != 0 &&  global_map_.sentry_die_pose_[i].y != 0)
                        cv::circle(show_img, cv::Point( global_map_.sentry_die_pose_[i].x*100, show_img.rows-1- global_map_.sentry_die_pose_[i].y*100), 30, cv::Scalar(0,0,0), 3);
                }
                if(global_map_.received_ && teammate_pose_.x != 0 &&  teammate_pose_.y != 0)
                    cv::circle(show_img, cv::Point( teammate_pose_.x*100, show_img.rows-1- teammate_pose_.y*100), 30, cv::Scalar(255,0,0), 3);
                if(team_info_.received_ && team_info_.attack_enemy != 0)
                cv::circle(show_img, cv::Point( team_info_.teammate_enemy_pose_.y*100 && show_img.rows-1-team_info_.teammate_enemy_pose_.y*100), 5, cv::Scalar(0,0,255), 5);
                if(armor_detection_.received_ && armor_detection_.detected_enemy)
                cv::circle(show_img, cv::Point( armor_detection_.detected_enemy_pose_.x*100, show_img.rows-1- armor_detection_.detected_enemy_pose_.y*100), 5, cv::Scalar(0,0,0), 5);
            }
            dst_map_ = show_img.clone();

        }

        void draw_chassis(cv::Mat& dst_img, cv::Point3d post, int id)
        {
            if(post.z > CV_PI)  post.z -= CV_2PI;
            if(post.z <-CV_PI)  post.z += CV_2PI;
            double width_2 = 0.6/2*100, height_2 = 0.5/2*100;
            cv::Point p1, p2, p3, p4;
            p1.x =   width_2*cos(post.z) - height_2*sin(post.z) + post.x*100;
            p1.y = -(width_2*sin(post.z) + height_2*cos(post.z) + post.y*100) + dst_img.rows;
            p2.x =  -width_2*cos(post.z) - height_2*sin(post.z) + post.x*100;
            p2.y =-(-width_2*sin(post.z) + height_2*cos(post.z) + post.y*100) + dst_img.rows;
            p3.x =  -width_2*cos(post.z) + height_2*sin(post.z) + post.x*100;
            p3.y =-(-width_2*sin(post.z) - height_2*cos(post.z) + post.y*100) + dst_img.rows;
            p4.x =   width_2*cos(post.z) + height_2*sin(post.z) + post.x*100;
            p4.y = -(width_2*sin(post.z) - height_2*cos(post.z) + post.y*100) + dst_img.rows;
            cv::line(dst_img, p1, p4, cv::Scalar(0,0,0), 2);
            if( id > 100)
            {
                cv::line(dst_img, p1, p2, cv::Scalar(255,0,0), 2);
                cv::line(dst_img, p2, p3, cv::Scalar(255,0,0), 2);
                cv::line(dst_img, p3, p4, cv::Scalar(255,0,0), 2);
            }
            else
            {
                cv::line(dst_img, p1, p2, cv::Scalar(0,0,255), 2);
                cv::line(dst_img, p2, p3, cv::Scalar(0,0,255), 2);
                cv::line(dst_img, p3, p4, cv::Scalar(0,0,255), 2);
            }
        }

        void draw_gimbal(cv::Mat& dst_img, cv::Point3d post)
        {
            double width = 0.4*100, height_2 = 0.1/2*100;
            cv::Point p1, p2, p3, p4;
            p1.x =   width*cos(post.z) - height_2*sin(post.z) + post.x*100;
            p1.y = -(width*sin(post.z) + height_2*cos(post.z) + post.y*100) + dst_img.rows;
            p2.x = - height_2*sin(post.z) + post.x*100;
            p2.y = -(height_2*cos(post.z) + post.y*100) + dst_img.rows;
            p3.x =   height_2*sin(post.z) + post.x*100;
            p3.y =-(-height_2*cos(post.z) + post.y*100) + dst_img.rows;
            p4.x =   width*cos(post.z) + height_2*sin(post.z) + post.x*100;
            p4.y = -(width*sin(post.z) - height_2*cos(post.z) + post.y*100) + dst_img.rows;

            cv::line(dst_img, p1, p2, cv::Scalar(0,0,0), 2);
            cv::line(dst_img, p2, p3, cv::Scalar(0,0,0), 2);
            cv::line(dst_img, p3, p4, cv::Scalar(0,0,0), 2);
            cv::line(dst_img, p1, p4, cv::Scalar(0,0,0), 2);
        }

        //监控函数，重置所有数据接收标志位
        void reset_flag()
        {
            game_status_.received_ = false;
            game_result_.received_ = false;
            game_robot_HP_.received_ = false;
            game_buff_.received_ = false;
            robot_status_.received_ = false;
            robot_damage_.received_ = false;
            robot_pose_.received_ = false;
            armor_detection_.received_ = false;
            armor_detection_.detected_id_ = 0;
            global_map_.received_ = false;
            gimbal_angle_.received_ = false;
            team_info_.received_ = false;
            planning_info_.received_ = false;
            planning_info_.flag = false;
            team_path_.received_ = false;
            // 重新初始化一些数据量
            team_info_.to_buff = 6;
            time_cnt_++;
            if(time_cnt_ >= 5) time_cnt_ = 0;
            planning_info_.no_go_points[time_cnt_][0] = 0;
            planning_info_.no_go_points[time_cnt_][1] = 0;
        }

        private:
        ros::NodeHandle nh_;
    };
}

#endif