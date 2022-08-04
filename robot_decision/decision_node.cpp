#include "decision_node.hpp"
#include <ros/package.h>
#include <iostream>
#include <fstream>

namespace robot_decision
{
    decision_node::decision_node()
    {
        ros::NodeHandle nh;
        nh.param("/robot_decision/decision_visualize_flag_", visualize_flag_, true);
        nh.param("/robot_decision/loop_rate", loop_rate_, 10);
        int fight_mode;
        nh.param("/robot_decision/fight_mode", fight_mode, 1);
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
        GlobalmapBehavior* global_map_node = new GlobalmapBehavior("global_map", 2, blackboard_, chassis_exe_, log_exe_, buff_exe_, self_info_exe_);
        RetreatBehavior* retreat_node = new RetreatBehavior("retreat", 1, blackboard_, chassis_exe_, log_exe_, buff_exe_, self_info_exe_);
        SupportBehavior* support_node = new SupportBehavior("support_teammate", 2, blackboard_, chassis_exe_, log_exe_, buff_exe_, self_info_exe_);
        AttackEnemyBehavior* attack_node = new AttackEnemyBehavior("attack by detection", 2, blackboard_, chassis_exe_, log_exe_, buff_exe_, self_info_exe_);
        PatrolBehavior* patrol_node = new PatrolBehavior("patrol", 1, blackboard_, chassis_exe_, log_exe_, buff_exe_, self_info_exe_);
        root_node_->addChild(game_start_node);
        root_node_->addChild(leave_buff_node);
        SequenceNode *go_buff_node = new SequenceNode("go_to_buff", 1, blackboard_, chassis_exe_, log_exe_, buff_exe_, self_info_exe_);
        go_buff_node->addChild(add_blood_node);
        go_buff_node->addChild(add_bullet_node);
        root_node_->addChild(go_buff_node);
        root_node_->addChild(retreat_node);
        SequenceNode* fight_node = new SequenceNode("fight", 1, blackboard_, chassis_exe_, log_exe_, buff_exe_, self_info_exe_);
        fight_node->addChild(global_map_node);
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
        std::cout << "*********START*********"<<std::endl;
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
            // 将已激活的buff设置为可活动区
            buff_exe_->set_buff(blackboard_->game_buff_.self_bullet_buff_id_, false);//加弹永远可以去
            for(int i = 0; i < 6; i++){
                if(!blackboard_->game_buff_.active_[i])
                    buff_exe_->set_buff(i, false);
            }
            self_info_exe_->update();
            buff_exe_->update();
            if(visualize_flag_)
            {
                blackboard_->visualize();
                cv::line(blackboard_->dst_map_, cv::Point(blackboard_->self_pose_.x*100,blackboard_->dst_map_.rows-1-blackboard_->self_pose_.y*100),
                         cv::Point(chassis_exe_->to_visual_goal.x*100, blackboard_->dst_map_.rows-1-chassis_exe_->to_visual_goal.y*100), cv::Scalar(0,0,0), 2);
                cv::imshow("dst_map",blackboard_->dst_map_);
                cv::waitKey(1);
            } 
            cnt++;
            if(cnt >= loop_rate_){
                cnt = 0;
                blackboard_->reset_flag();
            }
            loop_rate.sleep(); 
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_decision");
    ros::NodeHandle ros_nh_;
    robot_decision::decision_node decision;
    ros::spin();
    return 0;
}
