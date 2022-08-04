#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "blackboard.hpp"
#include "chassis_executor.hpp"
#include "log_executor.hpp"
#include "buff_executor.hpp"
#include "self_info_executor.hpp"

namespace robot_decision
{
    enum class BehaviorState {
        SUCCESS,
        FAILURE
    };
    class TreeNode
    {
        public:
        TreeNode(){}
        TreeNode(std::string name, 
                 int level, 
                 const Blackboard::Ptr &blackboard_ptr, 
                 Chassis_executor::Ptr &chassis_exe_ptr, 
                 Log_executor::Ptr &log_exe_ptr,
                 Buff_executor::Ptr &buff_exe_ptr,
                 Self_Info_executor::Ptr &self_info_exe_ptr) :
            name_(name),
            level_(level),
            blackboard_ptr_(blackboard_ptr),
            chassis_exe_ptr_(chassis_exe_ptr),
            log_exe_ptr_(log_exe_ptr),
            buff_exe_ptr_(buff_exe_ptr),
            self_info_exe_ptr_(self_info_exe_ptr)
        {}
        virtual ~TreeNode() = default;
        virtual BehaviorState Update() = 0;
        virtual void print() = 0;
        virtual void print_tree() = 0;
        BehaviorState Run()
        {
            behavior_state_ = Update();
            return behavior_state_;
        }
        Blackboard::Ptr blackboard_ptr_;
        Chassis_executor::Ptr chassis_exe_ptr_;
        Log_executor::Ptr log_exe_ptr_;
        Buff_executor::Ptr buff_exe_ptr_;
        Self_Info_executor::Ptr self_info_exe_ptr_;
        int level_;
        std::string name_;
        BehaviorState behavior_state_;
    };

    class ActionNode : public TreeNode
    {
        public:
        ActionNode():TreeNode::TreeNode(){}
        ActionNode(std::string name, 
                   int level, 
                   const Blackboard::Ptr &blackboard_ptr, 
                   Chassis_executor::Ptr &chassis_exe_ptr, 
                   Log_executor::Ptr &log_exe_ptr,
                   Buff_executor::Ptr &buff_exe_ptr,
                   Self_Info_executor::Ptr &self_info_exe_ptr) :
            TreeNode::TreeNode(name, level, blackboard_ptr, chassis_exe_ptr, log_exe_ptr, buff_exe_ptr, self_info_exe_ptr)
        {}
        virtual ~ActionNode() = default;
        void print()
        {
            std::stringstream str;
            for(int i = 0; i < level_; i++)
            {
                str << "  ";
            }
            str << "- " << name_.data();
            log_exe_ptr_->print(str);
        }
        void print_tree()
        {
            print();
        }
    };

    class SequenceNode : public TreeNode
    {
        public:
        SequenceNode():TreeNode::TreeNode(){}
        SequenceNode(std::string name, 
                     int level, 
                     const Blackboard::Ptr &blackboard_ptr, 
                     Chassis_executor::Ptr &chassis_exe_ptr, 
                     Log_executor::Ptr &log_exe_ptr,
                     Buff_executor::Ptr &buff_exe_ptr,
                     Self_Info_executor::Ptr &self_info_exe_ptr) :
            TreeNode::TreeNode(name, level, blackboard_ptr, chassis_exe_ptr, log_exe_ptr, buff_exe_ptr, self_info_exe_ptr)
        {}
        virtual ~SequenceNode() = default;
        void addChild(TreeNode* child_node_ptr)
        {
            child_node_ptr_list_.push_back(child_node_ptr);
        }
        virtual BehaviorState Update()
        {
            if(child_node_ptr_list_.size() == 0)
            {
                return BehaviorState::FAILURE;
            }
            else
            {
                for(int i = 0; i < child_node_ptr_list_.size(); i++)
                {
                    if(child_node_ptr_list_[i]->Run() == BehaviorState::SUCCESS)
                    {
                        return BehaviorState::SUCCESS;
                    }
                }
                return BehaviorState::FAILURE;
            }
        }
        void print()
        {
            std::stringstream str;
            for(int i = 0; i < level_; i++)
            {
                str << "  ";
            }
            str << "+ " << name_.data();
            log_exe_ptr_->print(str);
        }
        void print_tree()
        {
            print();
            for(int i = 0; i < child_node_ptr_list_.size(); i++)
            {
                child_node_ptr_list_[i]->print_tree();
            }
        }
        private:
        std::vector<TreeNode*> child_node_ptr_list_;
    };
}

#endif