#ifndef ARMOR_DETECTION_H
#define ARMOR_DETECTION_H

#include <opencv2/opencv.hpp>

namespace robot_detection {
    class ArmorDetection {
    public:
        ArmorDetection();

        void LoadParam(bool color, 
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
                       cv::Mat intrinsic_matrix, cv::Mat distortion_coeffs);

        void DetectArmor(cv::Mat &src_img_, bool &detected, cv::Point3f &target_3d, cv::Point3f &target_r);

        void DetectLights(const cv::Mat &src_img_, std::vector<cv::RotatedRect> &lights, std::vector<double> &lights_angle);

        void PossibleArmors(const std::vector<cv::RotatedRect> &lights,
                            std::vector<cv::RotatedRect> &armors_possible, 
                            std::vector<double> &lights_angle);

        cv::RotatedRect SelectFinalArmor(std::vector<cv::RotatedRect> &armors, cv::Point3f &target_3d_);

        void CalcControlInfo(cv::Mat &src_img_, cv::RotatedRect &armor, cv::Point3f &target_3d, cv::Point3f &target_r);

        void SolveArmorCoordinate(const double width, const double height);

        ~ArmorDetection();

    private:
        cv::Mat binary_color_img_;

        cv::Mat intrinsic_matrix_;
        cv::Mat distortion_coeffs_;

        // Parameters come form .prototxt file
        bool enable_debug_;
        bool enemy_color_;

        //! armor size
        double armor_width;
        double armor_height;

        //! armor info
        std::vector<cv::Point3f> armor_points_;
        cv::RotatedRect final_armor_;

        //! Filter lights
        double light_min_angle_;
        double light_max_angle_;
        double light_max_angle_diff_;
        double light_aspect_ratio_;

        //! Filter armor
        double armor_max_angle_;
        double armor_max_aspect_ratio_;
        double armor_min_aspect_ratio_;

        cv::Point2f last_armor_center_;

        int red_threshold_;
        int blue_threshold_;
        int red_brightness_threshold_;
        int blue_brightness_threshold_;
    };
}

#endif