#include <builder/trt_builder.hpp>
#include <common/ilogger.hpp>
#include <common/cuda_tools.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <infer/trt_infer.hpp>
#include <common/ilogger.hpp>
#include <algorithm>
#include <thread>
#include <opencv2/opencv.hpp>
#include <common/preprocess_kernel.cuh>
#include <common/trt_tensor.hpp>
#include <rsort.h>

class YoloBox
{
    private:
        float getWidth() { return (x2 - x1); };
        float getHeight() { return (y2 - y1); };

    public:
        int x1;
        int y1;
        int x2;
        int y2;
        float cos_;
        float sin_;
        float angle_;
        
        int cate;
        float score;
        float origin_score;

        float area() { return getWidth() * getHeight(); };
};
float calc_iou(const YoloBox &a, const YoloBox &b);
class trrt_detector
{
    public:
        trrt_detector(std::string model_file, std::string onnx_file,int input_size);
        ~trrt_detector(){};
        void infer(cv::Mat &img,int sentry_id,std::vector<YoloBox>&infer_result,std::vector<detbox> &detection_list);
        void prehandle(std::vector<std::shared_ptr<TRT::Tensor>> extracter,std::vector<YoloBox>& result,cv::Mat &img);
        void yolo2xya(std::vector<YoloBox>& result,std::vector<detbox> &detection_list,int sentry_id);
        bool CheckSentryAvailable(std::vector<detbox> &detection_list, int teampose[4],int color_flag);
    
    private:
        std::shared_ptr<TRT::Infer> engine;
        int area_width_;
        int area_height_;
        int src_width_;
        int src_height_;
        int max_batch_size;
        int input_size_;
        int high_sample_size_;
        int low_sample_size_;
        std::vector<float> anchor;
        
        
};