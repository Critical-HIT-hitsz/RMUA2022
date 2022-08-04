#include "armor_detection.h"

namespace robot_detection {
    ArmorDetection::ArmorDetection() {
        last_armor_center_ = cv::Point2f(0, 0);
        armor_height = 60;
        armor_width = 127.5;
        SolveArmorCoordinate(armor_width, armor_height);
    }

    void ArmorDetection::LoadParam(bool color, 
                                   bool enable_debug,
                                   double light_min_angle, 
                                   double light_max_angle_diff,
                                   double light_aspect_ratio, 
                                   double armor_max_angle, 
                                   double armor_max_aspect_ratio, 
                                   double armor_min_aspect_ratio,
                                   int blue_threshold, 
                                   int red_threshold,
                                   int red_brightness_threshold,
                                   int blue_brightness_threshold,
                                   cv::Mat intrinsic_matrix, cv::Mat distortion_coeffs) {
        enemy_color_ = color;
        enable_debug_ = enable_debug;
        light_min_angle_ = light_min_angle;
        light_max_angle_ = 180 - light_min_angle_;
        light_max_angle_diff_ = light_max_angle_diff;
        light_aspect_ratio_ = light_aspect_ratio;
        armor_max_angle_ = armor_max_angle;
        armor_max_aspect_ratio_ = armor_max_aspect_ratio;
        armor_min_aspect_ratio_ = armor_min_aspect_ratio;
        blue_threshold_ = blue_threshold;
        red_threshold_ = red_threshold;
        red_brightness_threshold_ = red_brightness_threshold;
        blue_brightness_threshold_ = blue_brightness_threshold;
        intrinsic_matrix_ = intrinsic_matrix;
        distortion_coeffs_ = distortion_coeffs;
    }

    void ArmorDetection::DetectArmor(cv::Mat &src_img_, bool &detected, cv::Point3f &target_3d, cv::Point3f &target_r) {
        std::vector<cv::RotatedRect> lights;
        std::vector<double> lights_angle;
        std::vector<cv::RotatedRect> armors;

        DetectLights(src_img_, lights, lights_angle);
        PossibleArmors(lights, armors, lights_angle);
        if (!armors.empty()) {
            detected = true;
            final_armor_ = SelectFinalArmor(armors, target_3d);

            last_armor_center_ = cv::Point2f(final_armor_.center.x, final_armor_.center.y);

            if (enable_debug_) {
                cv::Point2f vertex[4];
                final_armor_.points(vertex);
                for (int i = 0; i < 4; i++)
                    cv::line(src_img_, vertex[i], vertex[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
            }

            // CalcControlInfo(src_img_, final_armor_, target_3d, target_r);
            std::stringstream ss;
            ss << (int)target_3d.x << "," << (int)target_3d.y << "," << (int)target_3d.z;
            std::string res;
            ss >> res;
            cv::putText(src_img_, res, cv::Point(20,20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255), 1);
        } else {
            detected = false;
        }
        if(enable_debug_) {
            cv::imshow("armors_after_filter", src_img_);
            if(cv::waitKey(1) == 'q')
            {
                enable_debug_ = false;
                cv::destroyAllWindows();
            }
        }
    }

    void ArmorDetection::DetectLights(const cv::Mat &src_img_, std::vector<cv::RotatedRect> &lights, std::vector<double> &lights_angle) {
        std::vector<cv::Mat> bgr;
        cv::Mat light, gray_img_, binary_brightness_img_;
        cv::split(src_img_, bgr);
        if (enemy_color_ == true) {
            // red
            cv::subtract(bgr[2], bgr[1], light);
        } else if (enemy_color_ == false) {
            // blue
            cv::subtract(bgr[0], bgr[2], light);
        }
        cv::cvtColor(src_img_, gray_img_, CV_BGR2GRAY);
        double thresh;
        if (enemy_color_ == false)
        {
            thresh = blue_threshold_;
            cv::threshold(gray_img_, binary_brightness_img_, blue_brightness_threshold_, 255, CV_THRESH_BINARY);
        }
        else
        {
            thresh = red_threshold_;
            cv::threshold(gray_img_, binary_brightness_img_, red_brightness_threshold_, 255, CV_THRESH_BINARY);
        }
        cv::threshold(light, binary_color_img_, thresh, 255, CV_THRESH_BINARY);
        cv::Mat close_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(6, 6));
        cv::dilate(binary_color_img_, binary_color_img_, close_element);
        cv::bitwise_and(binary_color_img_, binary_brightness_img_, binary_color_img_);

        if(enable_debug_) {
            // cv::imshow("binary_color", binary_color_img_);
        }

        std::vector<std::vector<cv::Point>> contours_brightness;
        cv::findContours(binary_color_img_, contours_brightness, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        
        lights.clear();

        for (int i = 0; i < contours_brightness.size(); ++i) {
            double armor_angle;
            cv::RotatedRect single_light = cv::minAreaRect(contours_brightness[i]);
            if(single_light.size.width < single_light.size.height) {
                armor_angle = single_light.angle - 90;
            } else {
                armor_angle = single_light.angle;
            }
            armor_angle *= -1;

            auto light_aspect_ratio = std::max(single_light.size.width, single_light.size.height) /
                                      std::min(single_light.size.width, single_light.size.height);

            if (armor_angle < light_max_angle_ && armor_angle > light_min_angle_ && light_aspect_ratio > light_aspect_ratio_) {
                lights.push_back(single_light);
                lights_angle.push_back(armor_angle);
            }
        }

        cv::imshow("gray_image", binary_color_img_);
    }

    void ArmorDetection::PossibleArmors(const std::vector<cv::RotatedRect> &lights,
                                        std::vector<cv::RotatedRect> &armors_possible, 
                                        std::vector<double> &lights_angle) {
        if(lights.size() < 2) return;
        for(unsigned int i = 0; i < lights.size(); i++) {
            for (unsigned int j = i + 1; j < lights.size(); j++) {
                cv::RotatedRect light1 = lights[i];
                cv::RotatedRect light2 = lights[j];
                auto edge1 = std::minmax(light1.size.width, light1.size.height);
                auto edge2 = std::minmax(light2.size.width, light2.size.height);

                if(std::fabs(light1.center.x - light2.center.x) < 1e-3)
                    continue;

                double armor_angle = std::atan((light1.center.y - light2.center.y) /
                                              (light1.center.x - light2.center.x)) / CV_PI * 180.0;
                double center_angle_for_rect;
                if (armor_angle > 0)
                    center_angle_for_rect = armor_angle;
                else
                    center_angle_for_rect = 180 + armor_angle;
                if(std::abs(armor_angle) > armor_max_angle_)
                {
                    continue;
                }
                auto angle_diff = std::abs(lights_angle[i] - lights_angle[j]);
                if(angle_diff > light_max_angle_diff_)
                {
                    continue;
                }
                auto lights_dis = std::sqrt((light1.center.x - light2.center.x) * (light1.center.x - light2.center.x) +
                                            (light1.center.y - light2.center.y) * (light1.center.y - light2.center.y));
                double armor_height_max = std::max<double>(edge1.second, edge2.second);
                double armor_height_min = std::min<double>(edge1.second, edge2.second);
                if(std::fabs(armor_height_min) < 1e-3)
                    continue;
                
                if(lights_dis / armor_height_max > armor_max_aspect_ratio_)
                {
                    continue;
                }
                if(lights_dis / armor_height_max < armor_min_aspect_ratio_)
                {
                    continue;
                }
                if(armor_height_max / armor_height_min > 1.8)
                {
                    continue;
                }
                cv::RotatedRect rect;
                rect.angle = center_angle_for_rect;
                if(rect.angle > 90)
                {
                    rect.angle -= 180;
                }
                rect.center.x = (light1.center.x + light2.center.x) / 2;
                rect.center.y = (light1.center.y + light2.center.y) / 2;
                rect.size.width = lights_dis;
                rect.size.height = armor_height_max;
                armors_possible.emplace_back(rect);
            }
        }
        if(armors_possible.size() < 2) return;
        std::vector<bool> is_armor(armors_possible.size(), true);
        for (int i = 0; i < armors_possible.size() - 1 && is_armor[i]; i++) {
            for (int j = i + 1; j < armors_possible.size() && is_armor[j]; j++) {
                double dx = armors_possible[i].center.x - armors_possible[j].center.x;
                double dy = armors_possible[i].center.y - armors_possible[j].center.y;
                double dis = std::sqrt(dx * dx + dy * dy);
                // filter the armor reflected by the ground
                if (std::abs(dy / dx) > 20 || std::fabs(dx) < 1e-3) {
                    if (dy > 0) //delete the lower armor
                        is_armor[i] = false;
                    else
                        is_armor[j] = false;
                }
                //if two armors have public area
                auto edge1 = std::minmax(armors_possible[i].size.width, armors_possible[i].size.height);
                auto edge2 = std::minmax(armors_possible[j].size.width, armors_possible[j].size.height);
                if (dis <= (edge1.second + edge2.second)/2) {
                    double angle1 = fabs(armors_possible[i].angle);
                    double angle2 = fabs(armors_possible[j].angle);
                    if(fabs(angle1-angle2) > 5)
                    {
                        if (angle1 > angle2)
                            is_armor[i] = false;
                        else
                            is_armor[j] = false;
                    }
                }
            }
        }
        for (unsigned int i = 0; i < armors_possible.size(); i++) {
            if(!is_armor[i]) {
                armors_possible.erase(armors_possible.begin() + i);
                is_armor.erase(is_armor.begin() + i);
                //after erasing, i points to next element
                i--;
            }
        }
    }

    cv::RotatedRect ArmorDetection::SelectFinalArmor(std::vector<cv::RotatedRect> &armors, cv::Point3f &target_3d_) {
        cv::Point2f current_armor_center;
        double center_dis, min_center_dis = 10000;
        int last_index = 0, last_distance;
        int return_index = 0;
        int min_distance = 10000;
        int nearest_index = 0;
        cv::Point3f last_target, nearest_target;

        for (int i = 0; i < armors.size(); i++) {
            cv::Mat rvec, tvec;
            std::vector<cv::Point2f> armor_points;
            cv::Point2f armor_point[4];
            armors[i].points(armor_point);
            double armor_angle;
            if(armors[i].angle < 90) {
                armor_angle = 90 - armors[i].angle;
            } else {
                armor_angle = 270 - armors[i].angle;
            }
            if(armor_angle >= 90) {
                armor_points.push_back(armor_point[2]);
                armor_points.push_back(armor_point[1]);
                armor_points.push_back(armor_point[0]);
                armor_points.push_back(armor_point[3]);
            } else {
                armor_points.push_back(armor_point[0]);
                armor_points.push_back(armor_point[3]);
                armor_points.push_back(armor_point[2]);
                armor_points.push_back(armor_point[1]);
            }
            cv::solvePnP(armor_points_,
                        armor_points,
                        intrinsic_matrix_,
                        distortion_coeffs_,
                        rvec,
                        tvec);
            cv::Point3f target_3d = cv::Point3f(tvec);
            if(target_3d.z < min_distance)
            {
                min_distance = target_3d.z;
                nearest_index = i;
                nearest_target = target_3d;
            }

            current_armor_center = cv::Point2f(armors[i].center.x, armors[i].center.y);

            center_dis = std::sqrt(std::pow(current_armor_center.x - last_armor_center_.x, 2) +
                                   std::pow(current_armor_center.y - last_armor_center_.y, 2));

            if (center_dis < min_center_dis) {
                min_center_dis = center_dis;
                last_index = i;
                last_distance = target_3d.z;
                last_target = target_3d;
            }
        }
        if(std::abs(last_distance - min_distance) < 500)
        {
            return_index = last_index;
            target_3d_ = last_target;
        }
        else
        {
            return_index = nearest_index;
            target_3d_ = nearest_target;
        }
        return armors[return_index];
    }

    void ArmorDetection::CalcControlInfo(cv::Mat &src_img_, cv::RotatedRect &armor, cv::Point3f &target_3d, cv::Point3f &target_r) {
        cv::Mat rvec;
        cv::Mat tvec;

        std::vector<cv::Point2f> armor_points;
        cv::Point2f armor_point[4];
        armor.points(armor_point);
        double armor_angle;
        if(armor.angle < 90) {
            armor_angle = 90 - armor.angle;
        } else {
            armor_angle = 270 - armor.angle;
        }
        if(armor_angle >= 90) {
            armor_points.push_back(armor_point[2]);
            armor_points.push_back(armor_point[1]);
            armor_points.push_back(armor_point[0]);
            armor_points.push_back(armor_point[3]);
        } else {
            armor_points.push_back(armor_point[0]);
            armor_points.push_back(armor_point[3]);
            armor_points.push_back(armor_point[2]);
            armor_points.push_back(armor_point[1]);
        }

        cv::solvePnP(armor_points_,
                     armor_points,
                     intrinsic_matrix_,
                     distortion_coeffs_,
                     rvec,
                     tvec);
        target_3d = cv::Point3f(tvec);
        target_r = cv::Point3f(rvec);
    }

    void ArmorDetection::SolveArmorCoordinate(const double width, const double height) {
        armor_points_.emplace_back(cv::Point3f(-width / 2, height / 2, 0.0));
        armor_points_.emplace_back(cv::Point3f(width / 2, height / 2, 0.0));
        armor_points_.emplace_back(cv::Point3f(width / 2, -height / 2, 0.0));
        armor_points_.emplace_back(cv::Point3f(-width / 2, -height / 2, 0.0));
    }

    ArmorDetection::~ArmorDetection() {}
} 