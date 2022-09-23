#include "trrt_detector.h"

//Get inter area of 2 yolo boxes
float intersection_area(const YoloBox &a, const YoloBox &b)
{
    if (a.x1 > b.x2 || a.x2 < b.x1 || a.y1 > b.y2 || a.y2 < b.y1)
    {
        // no intersection
        return 0.f;
    }

    float inter_width = std::min(a.x2, b.x2) - std::max(a.x1, b.x1);
    float inter_height = std::min(a.y2, b.y2) - std::max(a.y1, b.y1);

    return inter_width * inter_height;
}

bool scoreSort(YoloBox a, YoloBox b) 
{ 
    return (a.score > b.score); 
}

//NMS method
int nmsHandle(std::vector<YoloBox> &tmpBoxes, 
              std::vector<YoloBox> &dstBoxes,float nmsThresh)
{
    std::vector<int> picked;
    
    sort(tmpBoxes.begin(), tmpBoxes.end(), scoreSort);

    for (int i = 0; i < tmpBoxes.size(); i++) {
        int keep = 1;
        for (int j = 0; j < picked.size(); j++) {

            float IoU = calc_iou(tmpBoxes[i], tmpBoxes[picked[j]]);


            if(IoU > nmsThresh) {
                keep = 0;
                break;
            }
        }

        if (keep) {
            picked.push_back(i);
        }
    }
    
    for (int i = 0; i < picked.size(); i++) {
        dstBoxes.push_back(tmpBoxes[picked[i]]);
    }

    return 0;
}

//====================================================TRT Function ===============================================================================
//Get the category with the highest socre, mainly for NMS
int getCategory(const float *values, int index, int &category, float &score)
{
    float tmp = 0;
    float objScore  = values[4 * 3 + index];

    for (int i = 0; i < 6; i++) {
        float clsScore = values[4 * 3 + 3 + i];
        clsScore *= objScore;

        if(clsScore > tmp) {
            score = clsScore;
            category = i;

            tmp = clsScore;
        }
    }
    
    return 0;
}
//==========================================================Detector =========================================================
//Init a TensorRT detector
trrt_detector::trrt_detector(std::string model_file, std::string onnx_file,int input_size)
{
    //Set GPU num and complie the model
    TRT::set_device(0);
    if(!iLogger::exists(model_file)){
        printf("model doesn't exist! Build the engine!\n");
        TRT::compile(TRT::Mode::FP16, 1, onnx_file, model_file);
    }

    std::cout << "create engine" << std::endl;
    //Load model
    engine = TRT::load_infer(model_file);
    engine->print();
    max_batch_size = 1;

    //Yolo anchor params
    std::vector<float> bias {18.39,45.10, 19.97,47.21, 23.96,34.24, 
                        24.91,36.72, 27.27,43.50, 28.25,52.91};
    anchor.assign(bias.begin(), bias.end());
    //src_img，feature map, dst_img params
    input_size_ = input_size;
    high_sample_size_ = 40;
    low_sample_size_ = 20;
    src_width_ = 900;
    src_height_ = 480;
    area_width_ = 808;
    area_height_ = 448;

}
//Get rotate IOU of  2 yoloboxes with angle
float calc_iou(const YoloBox &a, const YoloBox &b)
{
    int height = 60;
    int width =  60;
    double area1 = height*width;
    double angle1 = a.angle_;
    double angle2 = b.angle_;
    cv::RotatedRect rbox1(cv::Point((a.x1+a.x2)/2, (a.y1+a.y2)/2), cv::Size(height,width), angle1);
    cv::RotatedRect rbox2(cv::Point((b.x1+b.x2)/2, (b.y1+b.y2)/2), cv::Size(height,width), angle2);
    std::vector<cv::Point2f> vertices;
    try
    {
        int intersectionType = cv::rotatedRectangleIntersection(rbox1, rbox2, vertices);
        if (vertices.size()==0)
            return 0.0;
        else{
            std::vector<cv::Point2f> order_pts;
            // 找到交集（交集的区域），对轮廓的各个点进行排序
            cv::convexHull(cv::Mat(vertices), order_pts, true);
            double area = cv::contourArea(order_pts);
            float inner = (float) (area / (area1 + area1 - area + 0.0001));
            return inner;
        }
    }
    catch(cv::Exception& e)
    {
        std::cout<<e.what()<<endl;
        return 0.999;
    }
}
//Prehandle for the result of the model
void trrt_detector::prehandle(std::vector<std::shared_ptr<TRT::Tensor>> extracter,std::vector<YoloBox>& result,cv::Mat &img)
{

    std::vector<YoloBox> yolo_boxes;
    float scale_w,scale_h;
    scale_w = float(src_width_)/input_size_;
    scale_h = float(src_height_)/input_size_;
    for(int output_i=0;output_i<extracter.size();output_i++)
    {
        float* image_based_output = extracter[output_i]->cpu<float>();
        int out_width = extracter[output_i]->size(2);
        int out_height = extracter[output_i]->size(1);
        int stride;
        stride =  input_size_/ out_height;
        for(int i = 0; i < out_height; i++)
        {   
            for(int j = 0; j < out_width; j++)
            {
                float* values = image_based_output+23*(j+i*out_width);
                for(int k = 0; k < 3; k++)
                {
                    int category = -1;
                    float score = -1;
                    getCategory(values, k, category, score);
                    //std::cout<<"Category: "<<category<<std::endl;
                    if(score > 0.7)
                    {
                        YoloBox tmpBox;
                        float bcx, bcy, bw, bh;
                        float bcos,bsin;
                        bcx = ((values[k * 4 + 0] * 2. - 0.5) + j) * stride;
                        bcy = ((values[k * 4 + 1] * 2. - 0.5) + i) * stride;
                        bw = std::pow((values[k * 4 + 2] * 2.), 2) * anchor[(1 * 3 * 2) + k * 2 + 0];
                        bh = std::pow((values[k * 4 + 3] * 2.), 2) * anchor[(1 * 3 * 2) + k * 2 + 1];
                        bcos = values[21];
                        bsin = values[22];
                        float x1 = (bcx - 0.5 * bw)*scale_w;
                        float y1 = (bcy - 0.5 * bh)*scale_h;
                        float x2 = (bcx + 0.5 * bw)*scale_w;
                        float y2 = (bcy + 0.5 * bh)*scale_h;
                        tmpBox.x1 = x1;
                        tmpBox.y1 = y1;
                        tmpBox.x2 = x2;
                        tmpBox.y2 = y2;
                        tmpBox.score = score;
                        tmpBox.cate = category;
                        tmpBox.angle_ = atan2(bsin,bcos);
                        tmpBox.cos_ = cos(tmpBox.angle_);
                        tmpBox.sin_ = sin(tmpBox.angle_);
                        //std::cout<<"car_x: " << center_x<<"car_y: "<<center_y<<std::endl;
                        yolo_boxes.push_back(tmpBox);
                    }
                }     
            }
        }
    }
    nmsHandle(yolo_boxes,result,0.5);
    //We get yolo boxes with angle 

}

//Get box with x,y,angle for tracking
void trrt_detector::yolo2xya(std::vector<YoloBox>& result,std::vector<detbox> &detection_list,int sentry_id)
{
    for(int i = 0;i<result.size();i++)
    {
        detbox tmp_detbox;
        tmp_detbox.car_type = result[i].cate;
        // tmp_detbox.conf = result[i].score;
        float center_x = (result[i].x1+result[i].x2)*0.5;
        float center_y = (result[i].y1+result[i].y2)*0.5;
        if(center_x>=780||center_y<=32) continue;
        tmp_detbox.conf = std::sqrt(center_x*center_x + (480-center_y)*(480-center_y));
        float box_angle = result[i].angle_;
        switch(sentry_id){
            case 1:
                tmp_detbox.x_pos = 808-center_x;
                tmp_detbox.y_pos = 480-center_y;
                if(box_angle>0)
                {
                    box_angle = M_PI-box_angle;
                }
                else
                {
                    box_angle = -M_PI-box_angle;
                }
                tmp_detbox.car_angle = box_angle;
                break;
            case 2:
                if(box_angle> 0)
                {
                    box_angle -= M_PI;
                }
                else
                {
                    box_angle +=M_PI;
                }
                tmp_detbox.x_pos = 808-center_x;
                tmp_detbox.y_pos = 448-(480-center_y);
                tmp_detbox.car_angle = box_angle;
                break;
            case 3:
                tmp_detbox.x_pos = center_x;
                tmp_detbox.y_pos = 448-(480-center_y);
                box_angle = -box_angle;
                tmp_detbox.car_angle = box_angle;
                break;
            case 4:
                tmp_detbox.x_pos = center_x;
                tmp_detbox.y_pos = 480-center_y;
                tmp_detbox.car_angle = box_angle;
        }
        tmp_detbox.det_vector.push_back(tmp_detbox.x_pos);
        tmp_detbox.det_vector.push_back(tmp_detbox.y_pos);
        tmp_detbox.det_vector.push_back(sin(tmp_detbox.car_angle));
        tmp_detbox.det_vector.push_back(cos(tmp_detbox.car_angle));
        detection_list.push_back(tmp_detbox);
    }
}
//THIS FUNCUTION IS UNUSED
bool trrt_detector::CheckSentryAvailable(std::vector<detbox>&detection_list, int teampose[4],int color_flag)
{
    double car_1_error = 0;
    double car_2_error = 0;
    int car_1_detnum = 0;
    int car_2_detnum = 0;
    switch(color_flag)
    {
        case 0:
            for(int i=0;i<detection_list.size();i++)
            {
                if(detection_list[i].car_type == 0)
                {  
                    car_1_detnum++;
                    double dis = sqrt(pow((detection_list[i].x_pos-teampose[0]),2)+pow((detection_list[i].y_pos-teampose[1]),2));
                    car_1_error+= dis;
                }
                if(detection_list[i].car_type == 1)
                {  
                    car_2_detnum++;
                    double dis = sqrt(pow((detection_list[i].x_pos-teampose[2]),2)+pow((detection_list[i].y_pos-teampose[3]),2));
                    car_2_error+= dis;
                }
            }
            break;
        case 1:
            for(int i=0;i<detection_list.size();i++)
            {
                if(detection_list[i].car_type == 2)
                {  
                    car_1_detnum++;
                    double dis = sqrt(pow((detection_list[i].x_pos-teampose[0]),2)+pow((detection_list[i].y_pos-teampose[1]),2));
                    car_1_error+= dis;
                }
                if(detection_list[i].car_type == 3)
                {  
                    car_2_detnum++;
                    double dis = sqrt(pow((detection_list[i].x_pos-teampose[2]),2)+pow((detection_list[i].y_pos-teampose[3]),2));
                    car_2_error+= dis;
                }
            }
            break;
    }
    double avg_car1_error,avg_car2_error,avg_car_error;
    avg_car1_error = avg_car2_error= avg_car_error =0;
    if(car_1_detnum>0)
    {
        avg_car1_error = car_1_error/car_1_detnum;
        avg_car_error+=avg_car1_error;
    }
    if(car_2_detnum>0)
    {
        avg_car2_error = car_2_error/car_2_detnum;
        avg_car_error+=avg_car2_error;
    }
    if(car_1_detnum>0&&car_2_detnum>0)
    {
        avg_car_error*=0.5;
    }
    std::cout<<"Error: "<<avg_car_error<<std::endl;
    if(avg_car_error>30)
    {
        std::cout<<"Sentry Failed! "<<std::endl;
        return false;
    }
    else{
        return true;
    }
}
//Infer 2 image and get 2 detection lists, master and slave
void trrt_detector::infer(cv::Mat &img,int sentry_id,std::vector<YoloBox>&infer_result,std::vector<detbox> &detection_list)
{
    cv::Mat src_img = img.clone();
    //std::cout<<"SENTRY ID"<< sentry_id<<std::endl;
    cv::resize(src_img, src_img, cv::Size(input_size_, input_size_));
    std::shared_ptr<TRT::Tensor> input = engine->tensor("image");
    input->resize_single_dim(0, max_batch_size).to_gpu();  
    int ibatch = 0;
    float mean[] = {0,0,0};
    float std[] = {1.0,1.0,1.0};
    input->set_norm_mat(ibatch, src_img, mean, std);
    engine->forward(true);

    //Get low sample result
    std::vector<std::shared_ptr<TRT::Tensor>> output_extracter;
    std::shared_ptr<TRT::Tensor> out_high = engine->output(0);
    std::shared_ptr<TRT::Tensor> out_low = engine->output(1);
    output_extracter.push_back(out_high);
    output_extracter.push_back(out_low);
    prehandle(output_extracter,infer_result,img);
    yolo2xya(infer_result,detection_list,sentry_id);
}

