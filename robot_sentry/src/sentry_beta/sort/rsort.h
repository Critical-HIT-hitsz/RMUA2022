#ifndef R_SORT_H
#define R_SORT_H
#include <iostream>
#include <vector>
// #include "tracker.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "pv_kalman_filter.h"
#include "Hungarian.h"

typedef struct detection_box{
    int car_type;
    double conf;
    double x_pos;
    double y_pos;
    double car_angle;
    std::vector<double> det_vector;
    int tracker_id;
    bool fix_flag;
    bool filter_flag;
    
    //来个构造函数angle_kf
    detection_box()
    {
       x_pos = -1;
       y_pos = -1;
       car_angle = -1;
       car_type = -1;
       tracker_id = -1;
       fix_flag = false;
       filter_flag = false;
    }
}detbox;

class tracker{
public:
   detbox box;
   double delta_x;
   double delta_y;
   PVKalmanFilter pos_kf;
   PVKalmanFilter angle_kf;
   bool restart_track;
   std::vector<detbox> detboxs;
   detbox master_box;
   detbox slave_boxs;
   //来个构造函数
   tracker()
   {
      box.det_vector.resize(4);
      box.det_vector[0] = box.x_pos;
      box.det_vector[1] = box.y_pos;
      box.det_vector[2] = 0;
      box.det_vector[3] = 1;
      restart_track = true;
   }
   void reset()
   {
      restart_track = true;
   }
   void Predict(double delta_t)
   {
      pos_kf.Predict(delta_t);
      angle_kf.Predict(delta_t);
      Eigen::Vector2d angle_data = angle_kf.GetBox();
      Eigen::Vector2d pos_data = pos_kf.GetBox();
      box.x_pos = pos_data(0);
      box.y_pos = pos_data(1);
      box.car_angle = std::atan2(angle_data(0), angle_data(1));
      box.det_vector[0] = box.x_pos;
      box.det_vector[1] = box.y_pos;
      box.det_vector[2] = angle_data(0);
      box.det_vector[3] = angle_data(1);
   }
   std::vector<double> GetPredictResult()
   {
      Eigen::Vector2d angle_data = angle_kf.GetBox();
      Eigen::Vector2d pos_data = pos_kf.GetBox();
      std::vector<double> predict_result;
      predict_result.push_back(pos_data(0));
      predict_result.push_back(pos_data(1));
      predict_result.push_back(angle_data(0));
      predict_result.push_back(angle_data(1));
      return predict_result;
   }
};

class R_SORTER
{
   public:
   R_SORTER(int color_flag);
   ~R_SORTER();

   void divide_detection(int trkNum, std::vector<tracker*> trackers, std::vector<detbox> detections, std::vector<detbox>& unmatched_detections);
   
   std::vector<detbox> UpdateWithTwoCam(std::vector<std::vector<detbox>>detections,double delta_t,bool first_flag, int team_pos[4],bool team_locate_flags[2]);

   void track_iou_filter();

   void switch_trackid(cv::Point ID_point);

   double get_tracker_distance(int id);

   //void fix_track_id(int track_id,double x_pos,double y_pos,int switched_id);

   bool CheckStartTracker(std::vector<detbox> trackresult);

   bool CheckSentryAvailable(std::vector<tracker*> tracker_normal_list);

   void FixTracker(int team_pose[4],bool team_locate_flags[2]);

   //void CheckAndFixSwitch(int team_pose[4]);

   std::vector<tracker> trackers_list;
   
   int frame_count_;
   
   double delta_t_;

   bool fix_flag;

   HungarianAlgorithm HungAlgo_;

   double angle_error = 0.01;
   double pos_error = 0.01;
   bool ID_switch_flag;
   int color_flag_;
   double  max_iou_error;
   double  max_dis_error;

};

class  OutPutFilter
{
   public:
   PVKalmanFilter tracker_filters[4];

   bool filter_flag[4];

   OutPutFilter();

   ~OutPutFilter();

   bool InitOutputFilter(std::vector<detbox> &tracked_result);

   void TrackerPredict(double dT,std::vector<detbox> &tracked_result);

   void TrackerFixUpdate(std::vector<detbox> &tracked_result);


};

#endif
