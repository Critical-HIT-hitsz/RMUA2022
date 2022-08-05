#ifndef ROBOT_DETECTOR_H
#define ROBOT_DETECTOR_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include "simple_yolo.hpp"
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdarg.h>
#include <cstring>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/core/eigen.hpp>

namespace robot_detector
{
    using namespace std;

    class RobotDetector
    {
    public:
        typedef struct
        {
            float confidence;
            int class_id;
            cv::Rect box; //left_top(x,y), width, height
        } Object;

        struct Robot
        {
            cv::Rect m_rect;
            int m_number;
            int m_color;
            int m_armornums;
            float m_color_prb;
            float m_number_prb;
            int m_id; //1-red1 2-red2 3-blue1 4-blue2 5-grey 0-unknown
            cv::Point3f XYZ_camera;
            cv::Point3f XYZ_body;
            Robot()
            {
                m_number = 0;
                m_color = 0;
                m_armornums = 0;
                m_color_prb = 0.0;
                m_number_prb = 0.0;
                m_id = -1;
                XYZ_camera = {0.0, 0.0, 0.0};
                XYZ_body = {0.0, 0.0, 0.0};
            }
        };

        RobotDetector();
        ~RobotDetector();

        bool init(const string &model_name, SimpleYolo::Mode precision, const double cof_threshold, const double nms_threshold,
                  const std::vector<float> heights, const std::vector<float> distances);
        void load_camera_params(const std::vector<std::vector<double>> intrinsics,
                                const std::vector<std::vector<double>> extrinsics,
                                const std::vector<std::vector<double>> discoeffs);
        bool process_frame(cv::Mat inframe, vector<Object> &detected_objects, const int camera_id);
        void RobotMatch(const int camera_id, const std::vector<Object> &results, std::vector<Robot> &Robots);

    private:
        cv::Mat letterBox(cv::Mat src);
        cv::Rect detect2origin(const cv::Rect &dete_rect, float rate_to, int top, int left);
        void location_estimation(const int camera_id, std::vector<Robot> &Robots, bool is_robot_from_armors = false);

        //params
        double _cof_threshold;
        double _nms_area_threshold;
        float _ratio;
        int _topPad;
        int _btmPad;
        int _leftPad;
        int _rightPad;

        std::shared_ptr<SimpleYolo::Infer> _yolo;

        std::vector<cv::Mat> _intrinsic_cvs;
        std::vector<cv::Mat> _extrinsic_cvs;
        std::vector<cv::Mat> _discoff_cvs;

        std::vector<float> _distances;
        std::vector<float> _heights;
    };

} //namespace
#endif