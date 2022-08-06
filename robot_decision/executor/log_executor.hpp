#ifndef LOG_EXECUTOR_H
#define LOG_EXECUTOR_H

#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <time.h>

class Log_executor
{
    public:
    typedef std::shared_ptr<Log_executor> Ptr;
    Log_executor():
      create_(false)
    {
        time_t tt;
        time( &tt );
        tt = tt + 8*3600;  // transform the time zone
        tm* t= gmtime( &tt );
        char str[30];
        sprintf(str, "/log/%d-%02d-%02d %02d:%02d:%02d.txt",
                t->tm_year + 1900,
                t->tm_mon + 1,
                t->tm_mday,
                t->tm_hour,
                t->tm_min,
                t->tm_sec);
        // outFile = std::ofstream(ros::package::getPath("robot_decision") + str, std::ios_base::out);
        // if(outFile.is_open())
        // {
        //     create_ = true;
        // }
        // else
        // {
        //     std::cout << "\033[0;1;31m" << "Log Init Failed"  << "\033[0m" << std::endl;
        // }
    }
    ~Log_executor(){}
    void print(std::stringstream& str_stream)
    {
        if(!create_) return;
        // std::cout << str_stream.str() << std::endl;
        // outFile << str_stream.str() << std::endl;
    }
    private:
    std::ofstream outFile;
    bool create_;
};

#endif