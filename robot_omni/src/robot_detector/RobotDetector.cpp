#include "RobotDetector.h"

namespace robot_detector
{
    using namespace std;

    static bool exists(const string &path)
    {
        return access(path.c_str(), R_OK) == 0;
    }

    RobotDetector::RobotDetector() {}

    RobotDetector::~RobotDetector() {}

    bool RobotDetector::init(const string &model_name, SimpleYolo::Mode precision, const double cof_threshold, const double nms_threshold,
                             const std::vector<float> heights, const std::vector<float> distances)
    {
        int device_id = 0;
        SimpleYolo::set_device(device_id);
        auto type = SimpleYolo::Type::V5;
        const char *name = model_name.c_str();
        string onnx_file = cv::format("%s.onnx", name);
        string model_file = cv::format("%s.%s.trtmodel", name, SimpleYolo::mode_string(precision));
        int test_batch_size = 6;
        if (!exists(model_file) && !SimpleYolo::compile(
                                       precision,
                                       type,
                                       test_batch_size,
                                       onnx_file,
                                       model_file,
                                       1 << 30,
                                       ""))
        {
            printf("Can't find trtmodel and compile trtmodel failed\n");
            return false;
        }

        _cof_threshold = cof_threshold;
        _nms_area_threshold = nms_threshold;

        _yolo = SimpleYolo::create_infer(model_file, type, device_id, cof_threshold, nms_threshold);
        if (_yolo == nullptr)
        {
            printf("Yolo infer is nullptr\n");
            return false;
        }

        _distances = distances;
        _heights = heights;

        return true;
    }

    void RobotDetector::load_camera_params(const std::vector<std::vector<double>> intrinsics,
                                           const std::vector<std::vector<double>> extrinsics,
                                           const std::vector<std::vector<double>> discoeffs)
    {
        for (int i = 0; i < intrinsics.size(); i++)
        {
            cv::Mat intrinsic_temp = cv::Mat(3, 3, CV_64FC1);
            memcpy(intrinsic_temp.data, intrinsics[i].data(), intrinsics[i].size() * sizeof(double));
            _intrinsic_cvs.push_back(intrinsic_temp);
        }
        for (int i = 0; i < extrinsics.size(); i++)
        {
            cv::Mat extrinsic_temp = cv::Mat(4, 4, CV_64FC1);
            memcpy(extrinsic_temp.data, extrinsics[i].data(), extrinsics[i].size() * sizeof(double));
            _extrinsic_cvs.push_back(extrinsic_temp);
        }
        for (int i = 0; i < discoeffs.size(); i++)
        {
            _discoff_cvs.push_back(cv::Mat(discoeffs[i]).t());
        }
    }

    bool RobotDetector::process_frame(cv::Mat inframe,
                                      std::vector<Object> &detected_objects,
                                      const int camera_id)
    {
        if (inframe.empty())
        {
            std::cout << "Invalid Picture Input" << std::endl;
            return false;
        }

        cv::Mat resize_img = letterBox(inframe);

        auto objs = _yolo->commit(resize_img).get();

        std::vector<cv::Rect> origin_rect;
        std::vector<float> origin_rect_cof;
        std::vector<int> origin_classId;
        for (auto &obj : objs)
        {
            cv::Rect resize_rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top);
            origin_rect.push_back(resize_rect);
            origin_rect_cof.push_back(obj.confidence);
            origin_classId.push_back(obj.class_label);
        }

        std::vector<int> final_id;
        cv::dnn::NMSBoxes(origin_rect, origin_rect_cof, _cof_threshold, _nms_area_threshold, final_id);
        for (int i = 0; i < final_id.size(); ++i)
        {
            cv::Rect resize_rect = origin_rect[final_id[i]];
            cv::Rect rawrect = detect2origin(resize_rect, _ratio, _topPad, _leftPad);
            detected_objects.push_back(Object{
                origin_rect_cof[final_id[i]],
                origin_classId[final_id[i]],
                rawrect});
        }

        return true;
    }

    cv::Mat RobotDetector::letterBox(cv::Mat src)
    {
        if (src.empty())
        {
            std::cout << "input image invalid" << std::endl;
            return cv::Mat();
        }
        int in_w = src.cols;
        int in_h = src.rows;
        int tar_w = 640;
        int tar_h = 640;
        _ratio = std::min(float(tar_h) / in_h, float(tar_w) / in_w);
        int inside_w = std::round(in_w * _ratio);
        int inside_h = std::round(in_h * _ratio);
        int pad_w = tar_w - inside_w;
        int pad_h = tar_h - inside_h;

        cv::Mat resize_img;
        cv::resize(src, resize_img, cv::Size(inside_w, inside_h));
        pad_w = pad_w % 32;
        pad_h = pad_h % 32;
        pad_w = pad_w / 2;
        pad_h = pad_h / 2;

        _topPad = int(std::round(pad_h - 0.1));
        _btmPad = int(std::round(pad_h + 0.1));
        _leftPad = int(std::round(pad_w - 0.1));
        _rightPad = int(std::round(pad_w + 0.1));

        cv::copyMakeBorder(resize_img, resize_img, _topPad, _btmPad, _leftPad, _rightPad, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
        return resize_img;
    }

    cv::Rect RobotDetector::detect2origin(const cv::Rect &det_rect, float rate_to, int top, int left)
    {

        int inside_x = det_rect.x - left;
        int inside_y = det_rect.y - top;
        int ox = std::round(float(inside_x) / rate_to);
        int oy = std::round(float(inside_y) / rate_to);
        int ow = std::round(float(det_rect.width) / rate_to);
        int oh = std::round(float(det_rect.height) / rate_to);
        cv::Rect origin_rect(ox, oy, ow, oh);
        return origin_rect;
    }

    float linear_inter(const std::vector<float> X_Arr, const std::vector<float> Y_Arr, const int length, const float x)
    {
        int j;
        float y = 0;

        if (x <= X_Arr[0])
        {
            y = Y_Arr[0];
            return y;
        }
        else if (x >= X_Arr[length - 1])
        {
            y = Y_Arr[length - 1];
            return y;
        }
        else
        {
            for (j = 1; j < length; j++)
            {
                if (x <= X_Arr[j])
                {
                    j--;
                    break;
                }
            }
        }

        y = Y_Arr[j] * ((x - X_Arr[j + 1]) / (X_Arr[j] - X_Arr[j + 1])) + Y_Arr[j + 1] * ((x - X_Arr[j]) / (X_Arr[j + 1] - X_Arr[j]));

        return y;
    }

    void RobotDetector::location_estimation(const int camera_id, std::vector<Robot> &Robots, bool is_robot_from_armors)
    {

        for (int i = 0; i < Robots.size(); ++i)
        {
            float fx = _intrinsic_cvs[camera_id].at<double>(0, 0);
            float fy = _intrinsic_cvs[camera_id].at<double>(1, 1);
            float cx = _intrinsic_cvs[camera_id].at<double>(0, 2);
            float cy = _intrinsic_cvs[camera_id].at<double>(1, 2);
            float center_u = Robots[i].m_rect.x + Robots[i].m_rect.width / 2;
            float center_v = Robots[i].m_rect.y + Robots[i].m_rect.height / 2;

            if (!is_robot_from_armors)
            {
                Robots[i].XYZ_camera.z = linear_inter(_heights, _distances, _heights.size(), Robots[i].m_rect.height);
            }
            else
            {
                Robots[i].XYZ_camera.z = 0.35;
            }
            Robots[i].XYZ_camera.x = (center_u - cx) * Robots[i].XYZ_camera.z / fx;
            Robots[i].XYZ_camera.y = (center_v - cy) * Robots[i].XYZ_camera.z / fy;

            Eigen::Matrix4f extrinsic_temp;
            cv::cv2eigen(_extrinsic_cvs[camera_id], extrinsic_temp);
            Eigen::Vector3f P_camera;
            P_camera << Robots[i].XYZ_camera.x, Robots[i].XYZ_camera.y, Robots[i].XYZ_camera.z;
            Eigen::Matrix3f Rbc = extrinsic_temp.topLeftCorner(3, 3).inverse();
            Eigen::Vector3f tbc = -Rbc * extrinsic_temp.topRightCorner(3, 1);
            Eigen::Vector3f P_body = Rbc * P_camera + tbc;
            Robots[i].XYZ_body.x = P_body[0];
            Robots[i].XYZ_body.y = P_body[1];
            Robots[i].XYZ_body.z = 0;
            //std::cout<<"In Camera "<< camera_id << " Find "<< "Robot "<< Robots[i].m_id <<" x: "<< Robots[i].XYZ_body.x << " y: "<< Robots[i].XYZ_body.y << " z: "<< Robots[i].XYZ_body.z <<std::endl;
        }
    }

    void RobotDetector::RobotMatch(const int camera_id, const std::vector<Object> &results, std::vector<Robot> &Robots)
    {
        //split Robot and Armor
        std::vector<Object> armors;
        for (int i = 0; i < results.size(); ++i)
        {
            if (results[i].class_id == 0)
            {
                Robot Robot_item;
                Robot_item.m_rect = results[i].box;
                Robots.push_back(Robot_item);
            }
            else
            {
                armors.push_back(results[i]);
            }
        }

        std::vector<int> matched_armor_indexes;

        for (int i = 0; i < Robots.size(); ++i)
        {
            for (int j = 0; j < armors.size(); ++j)
            {
                if (armors[j].box.x >= Robots[i].m_rect.x - 3 &&
                    armors[j].box.y > Robots[i].m_rect.y &&
                    (armors[j].box.width + armors[j].box.x) <= (Robots[i].m_rect.width + Robots[i].m_rect.x + 5) && 
                    (armors[j].box.height + armors[j].box.y) < (Robots[i].m_rect.height + Robots[i].m_rect.y) &&
                    (armors[j].box.y + armors[j].box.height / 2) > (Robots[i].m_rect.height / 2 + Robots[i].m_rect.y))
                {
                    Robots[i].m_armornums += 1;
                    switch (armors[j].class_id)
                    {
                    case 1: //red_1
                        Robots[i].m_color_prb += armors[j].confidence;
                        Robots[i].m_number_prb += armors[j].confidence;
                        break;
                    case 2: //red_2
                        Robots[i].m_color_prb += armors[j].confidence;
                        Robots[i].m_number_prb -= armors[j].confidence;
                        break;
                    case 3: //blue_1
                        Robots[i].m_color_prb -= armors[j].confidence;
                        Robots[i].m_number_prb += armors[j].confidence;
                        break;
                    case 4: //blue_2
                        Robots[i].m_color_prb -= armors[j].confidence;
                        Robots[i].m_number_prb -= armors[j].confidence;
                        break;
                    case 5: //grey
                        // Robots[i].m_color = 3; 
                        break;
                    default:
                        break;
                    }
                    matched_armor_indexes.push_back(j);
                }
            }
            if (Robots[i].m_armornums > 0)
            {
                if (Robots[i].m_color_prb > 0.0)
                {                          
                    Robots[i].m_color = 1; //red
                }
                else if (Robots[i].m_color_prb < 0.0)
                {
                    Robots[i].m_color = 2; //blue
                }
                else
                {
                    Robots[i].m_color = 3; //grey
                    Robots[i].m_id = 5;
                }
                if (Robots[i].m_number_prb > 0.0)
                {
                    Robots[i].m_number = 1;
                }
                else if (Robots[i].m_number_prb < 0.0)
                {
                    Robots[i].m_number = 2;
                }
                if (Robots[i].m_number == 1 && Robots[i].m_color == 1)
                {
                    Robots[i].m_id = 1;
                }
                else if (Robots[i].m_number == 2 && Robots[i].m_color == 1)
                {
                    Robots[i].m_id = 2;
                }
                else if (Robots[i].m_number == 1 && Robots[i].m_color == 2)
                {
                    Robots[i].m_id = 3;
                }
                else if (Robots[i].m_number == 2 && Robots[i].m_color == 2)
                {
                    Robots[i].m_id = 4;
                }
            }
            else
            {
                Robots[i].m_id = 0; //unknown
            }
        }

        location_estimation(camera_id, Robots);

        std::vector<Object> unmatched_armors;
        for (int i = 0; i < armors.size(); i++)
        {
            std::vector<int>::iterator it = std::find(matched_armor_indexes.begin(), matched_armor_indexes.end(), i);
            if (it == matched_armor_indexes.end())
            {
                unmatched_armors.push_back(armors[i]);
            }
        }
        std::vector<Robot> robots_by_armors;
        for (int i = 0; i < unmatched_armors.size(); i++)
        {
            //The distance is too close, and only the armor is detected
            if (unmatched_armors[i].box.height > 150 || unmatched_armors[i].box.height * unmatched_armors[i].box.width > 30000)
            {
                Robot Robot_item;
                Robot_item.m_rect = unmatched_armors[i].box;
                Robot_item.m_id = unmatched_armors[i].class_id;
                robots_by_armors.push_back(Robot_item);
            }
        }
        location_estimation(camera_id, robots_by_armors, true);
        Robots.insert(Robots.end(), robots_by_armors.begin(), robots_by_armors.end());
    }

} //namespace
