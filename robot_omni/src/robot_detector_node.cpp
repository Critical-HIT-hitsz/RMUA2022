#include <ros/ros.h>
#include <csignal>
#include <ros/package.h>
#include <cmath>
#include <image_transport/image_transport.h>
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/String.h"
#include "RobotDetector.h"
#include "robot_msgs/GlobalMap.h"

namespace robot_detector
{
    typedef RobotDetector::Object Object;
    typedef RobotDetector::Robot Robot;
    static const char *labels[] = {"robot", "red1", "red2", "blue1", "blue2", "grey"};
    const std::vector<cv::Scalar> colors = {cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(0, 0, 255),
                                            cv::Scalar(255, 0, 0), cv::Scalar(255, 0, 0), cv::Scalar(128, 128, 128)};

    RobotDetector *detector = new RobotDetector;
    bool showResult;
    bool flg_exit = false;
    std::mutex mtx_buffer;

    struct MeasureGroup
    {
        double img1_time;
        double img2_time;
        double img3_time;
        double img4_time;

        std::vector<Object> cam1_objects;
        std::vector<Object> cam2_objects;
        std::vector<Object> cam3_objects;
        std::vector<Object> cam4_objects;
    };
    MeasureGroup Measures;

    void SigHandle(int sig)
    {
        flg_exit = true;
        ROS_WARN("catch sig %d", sig);
    }

    bool sync_packages(MeasureGroup meas, std::vector<std::vector<Object>> &detected_objects)
    {
        std::vector<std::vector<Object>> detected_objects_temp;
        detected_objects_temp.push_back(meas.cam1_objects);
        detected_objects_temp.push_back(meas.cam2_objects);
        detected_objects_temp.push_back(meas.cam3_objects);
        detected_objects_temp.push_back(meas.cam4_objects);

        std::vector<double> src_times;
        src_times.push_back(meas.img1_time);
        src_times.push_back(meas.img2_time);
        src_times.push_back(meas.img3_time);
        src_times.push_back(meas.img4_time);

        bool syncFlag = false;

        for (int i = 0; i < src_times.size(); i++)
        {
            if (src_times[i] != 0 && (ros::Time::now().toSec() - src_times[i]) < 0.5)
            {
                detected_objects.push_back(detected_objects_temp[i]);
                syncFlag = true;
            }
            else
            {
                std::vector<Object> emptyObject;
                detected_objects.push_back(emptyObject);
                ROS_WARN("Camera [%d] get image failed", i + 1);
            }
        }
        if (!syncFlag)
        {
            ROS_WARN("ALL Camera Failed, please check camera connection");
        }
        return syncFlag;
    }

    void PublishDetectRobots(ros::Publisher robot_detector_pub, const std::vector<Robot> robot_results)
    {
        robot_msgs::RobotLocation robots_temp;
        robot_msgs::GlobalMap submap_msg;
        submap_msg.header.stamp = ros::Time::now();

        for (int i = 0; i < robot_results.size(); ++i)
        {
            robots_temp.id = robot_results[i].m_id;
            robots_temp.x = robot_results[i].XYZ_body.x;
            robots_temp.y = robot_results[i].XYZ_body.y;
            robots_temp.yaw = 0;
            submap_msg.robots.push_back(robots_temp);
        }

        robot_detector_pub.publish(submap_msg);
    }

    void im_show(cv::Mat &src, std::string window_name, const std::vector<Object> detected_objects)
    {
        //Display at half the resolution of the original image
        cv::Mat srcHalf;
        cv::resize(src, srcHalf, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
        for (int i = 0; i < detected_objects.size(); ++i)
        {
            int xmin = detected_objects[i].box.x / 2;
            int ymin = detected_objects[i].box.y / 2;
            int width = detected_objects[i].box.width / 2;
            int height = detected_objects[i].box.height / 2;
            cv::Rect rect(xmin, ymin, width, height);
            int classId = detected_objects[i].class_id;
            cv::rectangle(srcHalf, rect, colors[classId], 2);
            auto name = labels[detected_objects[i].class_id];
            auto caption = cv::format("%s %.2f", name, detected_objects[i].confidence);
            int name_width = cv::getTextSize(caption, 0, 1, 2, nullptr).width - 50;
            cv::rectangle(srcHalf, cv::Point(xmin - 3, ymin - 20), cv::Point(xmin + name_width, ymin), colors[classId], -1);
            cv::putText(srcHalf, caption, cv::Point(xmin, ymin - 5), 0, 0.6, cv::Scalar::all(0), 2, 16);
        }
        cv::imshow(window_name, srcHalf);
        //cv::waitKey(1);
    }

    void im_show_around(cv::Mat &src, const std::vector<Robot> robot_results)
    {
        for (int i = 0; i < robot_results.size(); ++i)
        {
            int x_around = -robot_results[i].XYZ_body.y * 100 + 300;
            int y_around = -robot_results[i].XYZ_body.x * 100 + 400;
            if (robot_results[i].m_id == 1)
            {
                cv::putText(src, "1", cv::Point(x_around, y_around), 0, 1.5, cv::Scalar(0, 0, 255), 3, 8);
            }
            else if (robot_results[i].m_id == 2)
            {
                cv::putText(src, "2", cv::Point(x_around, y_around), 0, 1.5, cv::Scalar(0, 0, 255), 3, 8);
            }
            else if (robot_results[i].m_id == 3)
            {
                cv::putText(src, "1", cv::Point(x_around, y_around), 0, 1.5, cv::Scalar(255, 0, 0), 3, 8);
            }
            else if (robot_results[i].m_id == 4)
            {
                cv::putText(src, "2", cv::Point(x_around, y_around), 0, 1.5, cv::Scalar(255, 0, 0), 3, 8);
            }
            else if (robot_results[i].m_id == 5)
            {
                cv::putText(src, "0", cv::Point(x_around, y_around), 0, 1.5, cv::Scalar(100, 100, 100), 3, 8);
            }
        }
        cv::imshow("around", src);
        cv::waitKey(1);
    }

    void Image1Callback(const sensor_msgs::ImageConstPtr &image1)
    {
        cv::Mat src_image1 = cv_bridge::toCvShare(image1, "bgr8")->image;
        double img1_time = image1->header.stamp.toSec();
        ;

        std::vector<Object> cam1_objects;
        detector->process_frame(src_image1, cam1_objects, 1);

        mtx_buffer.lock();
        Measures.cam1_objects = cam1_objects;
        Measures.img1_time = img1_time;
        mtx_buffer.unlock();
        if (showResult)
        {
            im_show(src_image1, "cam1", cam1_objects);
        }
    }

    void Image2Callback(const sensor_msgs::ImageConstPtr &image2)
    {
        cv::Mat src_image2 = cv_bridge::toCvShare(image2, "bgr8")->image;
        double img2_time = image2->header.stamp.toSec();
        ;

        std::vector<Object> cam2_objects;
        detector->process_frame(src_image2, cam2_objects, 2);

        mtx_buffer.lock();
        Measures.cam2_objects = cam2_objects;
        Measures.img2_time = img2_time;
        mtx_buffer.unlock();
        if (showResult)
        {
            im_show(src_image2, "cam2", cam2_objects);
        }
    }

    void Image3Callback(const sensor_msgs::ImageConstPtr &image3)
    {
        cv::Mat src_image3 = cv_bridge::toCvShare(image3, "bgr8")->image;
        double img3_time = image3->header.stamp.toSec();
        ;

        std::vector<Object> cam3_objects;
        detector->process_frame(src_image3, cam3_objects, 3);

        mtx_buffer.lock();
        Measures.cam3_objects = cam3_objects;
        Measures.img3_time = img3_time;
        mtx_buffer.unlock();
        if (showResult)
        {
            im_show(src_image3, "cam3", cam3_objects);
        }
    }

    void Image4Callback(const sensor_msgs::ImageConstPtr &image4)
    {
        cv::Mat src_image4 = cv_bridge::toCvShare(image4, "bgr8")->image;
        double img4_time = image4->header.stamp.toSec();
        ;

        std::vector<Object> cam4_objects;
        detector->process_frame(src_image4, cam4_objects, 4);

        mtx_buffer.lock();
        Measures.cam4_objects = cam4_objects;
        Measures.img4_time = img4_time;
        mtx_buffer.unlock();
        if (showResult)
        {
            im_show(src_image4, "cam4", cam4_objects);
        }
    }

} //namespace robot_detector

int main(int argc, char **argv)
{
    using namespace robot_detector;
    ros::init(argc, argv, "robot_detector_node");
    ros::NodeHandle ros_nh;
    std::string img1_topic_name, img2_topic_name, img3_topic_name, img4_topic_name;
    double cof_threshold, nms_area_threshold;
    std::string model_path, around_png_path;
    float distance_thresh1, distance_thresh2;
    int camera_num;
    std::vector<std::vector<double>> intrinsics(4), extrinsics(4), discoeffs(4);
    std::vector<float> distances, heights;

    ros_nh.param<std::string>("/robot_detector_node/yolo_model_path", model_path, "/YoloModel/0430");
    ros_nh.param<std::string>("/robot_detector_node/img1_topic_name", img1_topic_name, "/mvsua_cam/image_raw2");
    ros_nh.param<std::string>("/robot_detector_node/img2_topic_name", img2_topic_name, "/mvsua_cam/image_raw3");
    ros_nh.param<std::string>("/robot_detector_node/img3_topic_name", img3_topic_name, "/mvsua_cam/image_raw4");
    ros_nh.param<std::string>("/robot_detector_node/img4_topic_name", img4_topic_name, "/mvsua_cam/image_raw5");
    ros_nh.param<double>("/robot_detector_node/cof_threshold", cof_threshold, 0.5);
    ros_nh.param<double>("/robot_detector_node/nms_area_threshold", nms_area_threshold, 0.5);
    ros_nh.param<bool>("/robot_detector_node/imshow", showResult, false);
    ros_nh.param("/robot_detector_node/CamNums", camera_num, 4);
    for (int i = 1; i <= camera_num; i++)
    {
        ros_nh.getParam("/robot_detector_node/In" + std::to_string(i), intrinsics[i - 1]);
        ros_nh.getParam("/robot_detector_node/Ex" + std::to_string(i), extrinsics[i - 1]);
        ros_nh.getParam("/robot_detector_node/Dis" + std::to_string(i), discoeffs[i - 1]);
    }
    ros_nh.getParam("/robot_detector_node/Heights", heights);
    ros_nh.getParam("/robot_detector_node/Distances", distances);

    cv::Mat around_png = cv::imread(ros::package::getPath("omni_detect") + "/maps/around.png");
    model_path = ros::package::getPath("omni_detect") + model_path;

    image_transport::ImageTransport it(ros_nh);
    image_transport::Subscriber img1_sub, img2_sub, img3_sub, img4_sub;
    img1_sub = it.subscribe(img1_topic_name, 1, Image1Callback);
    img2_sub = it.subscribe(img2_topic_name, 1, Image2Callback);
    img3_sub = it.subscribe(img3_topic_name, 1, Image3Callback);
    img4_sub = it.subscribe(img4_topic_name, 1, Image4Callback);

    ros::Publisher robot_detector_pub = ros_nh.advertise<robot_msgs::GlobalMap>("/omni_detector", 1, true);

    detector->init(model_path, SimpleYolo::Mode::FP16, cof_threshold, nms_area_threshold, heights, distances);
    detector->load_camera_params(intrinsics, extrinsics, discoeffs);
    ROS_INFO("\033[1;32m----> Omni Robot Detector Init Successfully.\033[0m");
    //------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(30);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit)
            break;
        ros::Time strat_time = ros::Time::now();

        ros::spinOnce();

        std::vector<std::vector<Object>> detected_objects;
        std::vector<Robot> robot_results;       // All detected robots
        std::vector<Robot> final_robot_results; // Final results after filtering out duplicate robots
        int robot_nums = 0;

        if (sync_packages(Measures, detected_objects))
        {
            for (int camera_id = 0; camera_id < detected_objects.size(); camera_id++)
            {
                std::vector<Object> obj = detected_objects[camera_id];
                std::vector<Robot> robots_temp;
                detector->RobotMatch(camera_id, obj, robots_temp);
                for (int j = 0; j < robots_temp.size(); j++)
                {
                    if (robots_temp[j].m_id != 0)
                    {
                        robot_results.push_back(robots_temp[j]);
                    }
                }

                // The following code is for measuring the data used for interpolation
                // for(int j = 0; j < robots_temp.size(); j++){
                //         //if(robots_temp[j].m_id == 4){
                //         int area = robots_temp[j].m_rect.height * robots_temp[j].m_rect.width;
                //         std::cout<< "Robot area: "<< area << " height: "<<robots_temp[j].m_rect.height<< " width: "<< robots_temp[j].m_rect.width<<std::endl;
                //     }
                // }
                // for(int k = 0; k < obj.size(); k++){
                //     if(obj[k].class_id == 3){
                //         int area = obj[k].box.height * obj[k].box.width;
                //         std::cout<< "Armor area: "<< area << " height: "<<obj[k].box.height << " width: "<< obj[k].box.width<<std::endl;
                //     }
                // }
            }

            // Filter out duplicate results.
            std::vector<int> repeat_indexes;
            for (int j = 0; j < robot_results.size(); ++j)
            {
                std::vector<int>::iterator it = std::find(repeat_indexes.begin(), repeat_indexes.end(), j);
                if (it != repeat_indexes.end())
                    continue;

                bool repeat_flag = false;
                for (int k = j + 1; k < robot_results.size(); ++k)
                {
                    if (robot_results[j].m_id == robot_results[k].m_id)
                    {
                        float dis = sqrt((robot_results[j].XYZ_body.x - robot_results[k].XYZ_body.x) * (robot_results[j].XYZ_body.x - robot_results[k].XYZ_body.x) +
                                         (robot_results[j].XYZ_body.y - robot_results[k].XYZ_body.y) * (robot_results[j].XYZ_body.y - robot_results[k].XYZ_body.y));
                        if (dis < 0.5)
                        {
                            repeat_flag = true;
                            repeat_indexes.push_back(k);
                            Robot robot_temp = robot_results[j];
                            robot_temp.XYZ_body.x = (robot_results[j].XYZ_body.x + robot_results[k].XYZ_body.x) / 2;
                            robot_temp.XYZ_body.y = (robot_results[j].XYZ_body.y + robot_results[k].XYZ_body.y) / 2;
                            final_robot_results.push_back(robot_temp);
                            continue;
                        }
                    }
                }
                if (!repeat_flag)
                    final_robot_results.push_back(robot_results[j]);
            }

            if (showResult)
            {
                cv::Mat around = around_png.clone();
                im_show_around(around, final_robot_results);
            }
            PublishDetectRobots(robot_detector_pub, final_robot_results);
        }

        std::cout << "total time: " << (ros::Time::now() - strat_time).toSec() * 1000 << "ms"
                  << " Find " << final_robot_results.size() << " robots" << std::endl;
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}
