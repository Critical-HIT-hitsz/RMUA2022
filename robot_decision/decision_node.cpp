#include "decision_node.hpp"
#include <ros/package.h>
#include <iostream>
#include <fstream>

namespace robot_decision
{
    decision_node::decision_node()
    {
        ros::NodeHandle nh;
        nh.param("decision_visualize_flag_", visualize_flag_, true);
        nh.param("loop_rate", loop_rate_, 5);
        int fight_mode;
        nh.param("fight_mode", fight_mode, 1);
        if(visualize_flag_){
            map_ = cv::imread(ros::package::getPath("robot_decision") + "/map.pgm");
            cv::resize(map_, map_, cv::Size(808, 448));
        }
        blackboard_ = std::make_shared<Blackboard>();
        buff_exe_ = std::make_shared<Buff_executor>();
        log_exe_ = std::make_shared<Log_executor>();
        chassis_exe_ = std::make_shared<Chassis_executor>(log_exe_);
        self_info_exe_ = std::make_shared<Self_Info_executor>();
        root_node_ = new SequenceNode("robot_decision", 0, blackboard_, chassis_exe_, log_exe_, buff_exe_, self_info_exe_);
        GameStartBehavior* game_start_node = new GameStartBehavior("wait_for_start", 1, blackboard_, chassis_exe_, log_exe_, buff_exe_, self_info_exe_);
        LeaveBuffBehavior* leave_buff_node = new LeaveBuffBehavior("leave_buff", 1, blackboard_, chassis_exe_, log_exe_, buff_exe_, self_info_exe_);
        AddBloodBehavior* add_blood_node = new AddBloodBehavior("add_blood", 2, blackboard_, chassis_exe_, log_exe_, buff_exe_, self_info_exe_);
        AddBulletBehavior* add_bullet_node = new AddBulletBehavior("add_bullet", 2, blackboard_, chassis_exe_, log_exe_, buff_exe_, self_info_exe_);
        RetreatBehavior* retreat_node = new RetreatBehavior("retreat", 1, blackboard_, chassis_exe_, log_exe_, buff_exe_, self_info_exe_);
        SupportBehavior* support_node = new SupportBehavior("support_teammate", 2, blackboard_, chassis_exe_, log_exe_, buff_exe_, self_info_exe_);
        AttackEnemyBehavior* attack_node = new AttackEnemyBehavior("attack by detection", 2, blackboard_, chassis_exe_, log_exe_, buff_exe_, self_info_exe_);
        SentryBehavior* sentry_node = new SentryBehavior("attack by sentry", 2, blackboard_, chassis_exe_, log_exe_, buff_exe_, self_info_exe_);
        PatrolBehavior* patrol_node = new PatrolBehavior("patrol", 1, blackboard_, chassis_exe_, log_exe_, buff_exe_, self_info_exe_);
        root_node_->addChild(game_start_node);
        root_node_->addChild(leave_buff_node);
        SequenceNode* go_buff_node = new SequenceNode("go_to_buff", 1, blackboard_, chassis_exe_, log_exe_, buff_exe_, self_info_exe_);
        go_buff_node->addChild(add_blood_node);
        go_buff_node->addChild(add_bullet_node);
        root_node_->addChild(go_buff_node);
        root_node_->addChild(retreat_node);
        SequenceNode* fight_node = new SequenceNode("fight", 1, blackboard_, chassis_exe_, log_exe_, buff_exe_, self_info_exe_);
        if(fight_mode == 1)
        {
            fight_node->addChild(support_node);
            fight_node->addChild(attack_node);
        }
        else
        {
            fight_node->addChild(attack_node);
            fight_node->addChild(support_node);
        }
        fight_node->addChild(sentry_node);
        root_node_->addChild(fight_node);
        root_node_->addChild(patrol_node);
        std::stringstream str;
        str << "***********************" << std::endl;
        str << "*Critical HIT decision*" << std::endl;
        str << "***********************";
        log_exe_->print(str);
        root_node_->print_tree();
        str.str("");    // clear the stringstream buffer
        str << "*********START*********";
        log_exe_->print(str);

        decision_thread_ = std::thread(&decision_node::ExecuteLoop, this);
        decision_thread_running_ = true;
    }

    void decision_node::ExecuteLoop()
    {
        ros::Rate loop_rate(loop_rate_);
        int cnt = 0;
        while (decision_thread_running_)
        {
            self_info_exe_->reset();
            buff_exe_->reset();
            root_node_->Run();

            buff_exe_->set_buff(blackboard_->self_bullet_buff_id_, false);
            for(int i = 0; i < 6; i++){
                if(!blackboard_->game_buff_.zone[i].active)
                    buff_exe_->set_buff(i, false);
            }
            self_info_exe_->update();
            buff_exe_->update();
            if(visualize_flag_) visualize(); 
            cnt++;
            if(cnt >= loop_rate_){
                cnt = 0;
                blackboard_->reset_flag();
            }
            loop_rate.sleep(); 
        }
    }

    void decision_node::visualize()
    {
        cv::Mat show_img = map_.clone();
        char str[20];
        sprintf(str, "time: %d", blackboard_->game_status_.remaining_time);
        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(str, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        cv::putText(show_img, str, cv::Point(0, 0 + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
        if(blackboard_->robot_status_.id < 100)
            cv::circle(show_img, cv::Point(blackboard_->self_pose_.x*100, show_img.rows-1-blackboard_->self_pose_.y*100), 2, cv::Scalar(0,0,255),2);
        else
            cv::circle(show_img, cv::Point(blackboard_->self_pose_.x*100, show_img.rows-1-blackboard_->self_pose_.y*100), 2, cv::Scalar(255,0,0),2);
        cv::arrowedLine(show_img, cv::Point(blackboard_->self_pose_.x*100, show_img.rows-1-blackboard_->self_pose_.y*100),
                        cv::Point(blackboard_->self_pose_.x*100+20*cos(blackboard_->gimbal_global_angle_), 
                        show_img.rows-blackboard_->self_pose_.y*100-20*sin(blackboard_->gimbal_global_angle_)), cv::Scalar(0,0,255), 1);
        cv::arrowedLine(show_img, cv::Point(blackboard_->self_pose_.x*100, show_img.rows-1-blackboard_->self_pose_.y*100),
                        cv::Point(blackboard_->self_pose_.x*100+20*cos(blackboard_->self_pose_.z), 
                        show_img.rows-blackboard_->self_pose_.y*100-20*sin(blackboard_->self_pose_.z)), cv::Scalar(0,255,0), 1);
        sprintf(str, "%d %d", blackboard_->self_hp_, blackboard_->remain_bullets_num_);
        label_size = cv::getTextSize(str, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        cv::putText(show_img, str, cv::Point(blackboard_->self_pose_.x*100, show_img.rows-1-blackboard_->self_pose_.y*100 + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
        
        for(int i = 0; i < 6; i++)
        {
            if(!blackboard_->game_buff_.zone[i].active)
                continue;
            switch(blackboard_->game_buff_.zone[i].type)
            {
                case 1://red_hp
                    cv::putText(show_img, "HP", cv::Point(blackboard_->buff_pos_[i].x*100, show_img.rows-1-blackboard_->buff_pos_[i].y*100), 
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255));
                    break;
                case 2://red_bullet
                    cv::putText(show_img, "BULLET", cv::Point(blackboard_->buff_pos_[i].x*100, show_img.rows-1-blackboard_->buff_pos_[i].y*100), 
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255));
                    break;
                case 3://blue_hp
                    cv::putText(show_img, "HP", cv::Point(blackboard_->buff_pos_[i].x*100, show_img.rows-1-blackboard_->buff_pos_[i].y*100), 
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0));
                    break;
                case 4://blue_bullet
                    cv::putText(show_img, "BULLET", cv::Point(blackboard_->buff_pos_[i].x*100, show_img.rows-1-blackboard_->buff_pos_[i].y*100), 
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0));
                    break;
                case 5://no_shoot
                    cv::putText(show_img, "S", cv::Point(blackboard_->buff_pos_[i].x*100, show_img.rows-1-blackboard_->buff_pos_[i].y*100), 
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0));
                    break;
                case 6://no_move
                    cv::putText(show_img, "M", cv::Point(blackboard_->buff_pos_[i].x*100, show_img.rows-1-blackboard_->buff_pos_[i].y*100), 
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0));
                    break;
                default:;
            }
        }

        for(int i = 0; i < blackboard_->enemy_pose_.size(); i++)
        {
            if(blackboard_->robot_status_.id < 100)
                cv::circle(show_img, cv::Point(blackboard_->enemy_pose_[i].x*100, show_img.rows-1-blackboard_->enemy_pose_[i].y*100), 5, cv::Scalar(255,0,0), 5);
            else
                cv::circle(show_img, cv::Point(blackboard_->enemy_pose_[i].x*100, show_img.rows-1-blackboard_->enemy_pose_[i].y*100), 5, cv::Scalar(0,0,255), 5);
        }

        if(blackboard_->robot_status_.id < 100)
            cv::circle(show_img, cv::Point(blackboard_->detected_enemy_pose_.x*100, show_img.rows-1-blackboard_->detected_enemy_pose_.y*100), 2, cv::Scalar(255,0,0), 2);
        else
            cv::circle(show_img, cv::Point(blackboard_->detected_enemy_pose_.x*100, show_img.rows-1-blackboard_->detected_enemy_pose_.y*100), 2, cv::Scalar(0,0,255), 2);

        cv::imshow("decision_debug", show_img);
        cv::waitKey(1);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_decision_node");
    ros::NodeHandle ros_nh_;
    robot_decision::decision_node decision;
    ros::spin();
    return 0;
}