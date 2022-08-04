#include <iostream>
#include <vector>
#include <set>
#include "rsort.h"


//If a tracker matched 2 detections ,mix the detections
detbox GetMixDetection(const detbox& box1,const detbox& box2)
{
    //std::cout<<"GET MIX"<<std::endl;
    detbox new_box;
    double score2 = box1.conf/(box1.conf+box2.conf);
    double score1 = box2.conf/(box1.conf+box2.conf);
    new_box.x_pos = score1*box1.x_pos+score2*box2.x_pos;
    new_box.y_pos = score1*box1.y_pos+score2*box2.y_pos;
    
    new_box.det_vector.push_back(new_box.x_pos);
    new_box.det_vector.push_back(new_box.y_pos);

    if(box1.car_type ==box2.car_type)
    {
        new_box.car_type = box1.car_type;
    }
    if(box1.conf<box2.conf)
    {
        new_box.car_type = box1.car_type;
        new_box.car_angle = box1.car_angle;
    }
    else
    {
        new_box.car_type = box2.car_type;
        new_box.car_angle = box2.car_angle;
    }
    new_box.det_vector.push_back(sin(new_box.car_angle));
    new_box.det_vector.push_back(cos(new_box.car_angle));
    return new_box;

}

double GetRiou_vec(std::vector<double> box1,std::vector<double> box2)
{
    int height = 60;
    int width =  60;
    double area1 = height*width;
    double area2 = area1;
    double angle1 = atan2(box1[2],box1[3]);
    double angle2 = atan2(box2[2],box2[3]);
    cv::RotatedRect  rbox1(cv::Point(box1[0], box1[1]), cv::Size(height,width) ,angle1);
    cv::RotatedRect  rbox2(cv::Point(box2[0], box2[1]), cv::Size(height,width),angle2);
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
};

R_SORTER::R_SORTER(int color_flag)
{
    ID_switch_flag = false;
    trackers_list.resize(4);
    //Only work on 4 trackers
    frame_count_ = 0;
    color_flag_ = color_flag;
    max_dis_error = 0;
    max_iou_error = 100;
    fix_flag = false;
}
R_SORTER::~R_SORTER()
{

}
//Set the Initial track id
void R_SORTER::switch_trackid(cv::Point ID_point)
{
    int track_num = 0;
    double min_dis = 10000;
    for(int i= 0 ;i<trackers_list.size();i++)
    {
        if(trackers_list[i].box.tracker_id == -1) continue;
        double temp_dis = std::sqrt(std::pow((ID_point.x-trackers_list[i].box.x_pos),2)+std::pow((ID_point.y-trackers_list[i].box.y_pos),2));
        if(temp_dis<min_dis)
        {
            track_num = i;
            min_dis = temp_dis; 
        }
    }
    trackers_list[track_num].box.tracker_id +=1;
    if(trackers_list[track_num].box.tracker_id>5)
    {
        trackers_list[track_num].box.tracker_id=0;
    } 
    
}
void R_SORTER::track_iou_filter()
{
    for(int i=0;i<trackers_list.size();i++){
        if(trackers_list[i].restart_track==true) continue;
        else{
            for(int j =i+1;j<trackers_list.size();j++){
                if(trackers_list[j].restart_track==true) continue;
                else{
                    double iou = GetRiou_vec(trackers_list[i].box.det_vector,trackers_list[j].box.det_vector);
                    if(iou>1.0/7){
                        double distance1 = std::sqrt(std::pow(trackers_list[i].delta_x, 2)+
                                        std::pow(trackers_list[i].delta_y, 2));
                        double distance2 = std::sqrt(std::pow(trackers_list[j].delta_x, 2)+
                                        std::pow(trackers_list[j].delta_y, 2));
                        if(distance1>distance2) trackers_list[i].restart_track = true;
                        else
                            trackers_list[j].restart_track = true;
                    }
                }
            }
        }

    }
}

void R_SORTER::divide_detection(int trkNum, std::vector<tracker*> tracker_normal_list, std::vector<detbox> detections, std::vector<detbox>& unmatched_detections)
{
    if(trkNum == 0)
    {
        for(int i = 0; i < detections.size(); i++)
        {
            unmatched_detections.push_back(detections[i]);
        }
        return;
    }
    int detNum_first = detections.size();
    std::vector<std::vector<double>> iou_matrix_first;
    iou_matrix_first.clear();
    iou_matrix_first.resize(trkNum,std::vector<double>(detNum_first,0));
    for (unsigned int i = 0; i < trkNum; i++) // compute iou matrix as a distance matrix
    {
        for (unsigned int j = 0; j < detNum_first; j++)
        {
            iou_matrix_first[i][j] = std::max(1 - GetRiou_vec(tracker_normal_list[i]->GetPredictResult(), detections[j].det_vector), 0.0);
        }
    }
    std::vector<int> assignments;
    HungAlgo_.Solve(iou_matrix_first,assignments);
    for(int i = 0; i < trkNum; i++)
    {
        if(assignments[i] != -1)
        {
            double det_track_dis = sqrt(pow(tracker_normal_list[i]->box.x_pos-detections[assignments[i]].x_pos,2)+pow(tracker_normal_list[i]->box.y_pos-detections[assignments[i]].y_pos,2));
            double riou = 1-iou_matrix_first[i][assignments[i]];
            if(riou<0.6&&det_track_dis>20)
            {
                unmatched_detections.push_back(detections[assignments[i]]);
                continue;
            }
            tracker_normal_list[i]->detboxs.push_back(detections[assignments[i]]);
        }
    }
    set<int> allItems_first;
    set<int> matchedItems_first;
    set<int> unmatchedDetections_first;
    if (detNum_first > trkNum) //	there are unmatched detections
    {
        for (unsigned int n = 0; n < detNum_first; n++)
            allItems_first.insert(n);

        for (unsigned int i = 0; i < trkNum; ++i)
            matchedItems_first.insert(assignments[i]);

        set_difference(allItems_first.begin(), allItems_first.end(),
            matchedItems_first.begin(), matchedItems_first.end(),
            insert_iterator<set<int>>(unmatchedDetections_first, unmatchedDetections_first.begin()));
    }
    for (auto umd : unmatchedDetections_first)
    {
        unmatched_detections.push_back(detections[umd]);
    }
}

bool R_SORTER::CheckSentryAvailable(std::vector<tracker*> tracker_normal_list)
{
    if(tracker_normal_list.size()==0) return true;
    double master_sum_iou_error;
    double master_sum_dis_error;
    double slave_sum_iou_error;
    double slave_sum_dis_error;
    int matched_trk_num = 0;
    for(int i  = 0;i<tracker_normal_list.size();i++)
    {
        if(!tracker_normal_list[i]->detboxs.size()>0) continue;
        std::vector<double> predict_result = tracker_normal_list[i]->GetPredictResult();
        matched_trk_num++;
        double error = 0;
        double master_distance_error = 0;
        double slave_distance_error = 0;
        double master_iou_error = 0;
        double slave_iou_error = 0;
        for(int j=0;j<tracker_normal_list[i]->detboxs.size();j++)
        {
            if(j == 0)
            {
                master_iou_error += GetRiou_vec(predict_result, tracker_normal_list[i]->detboxs[j].det_vector);
                master_distance_error += sqrt(pow(predict_result[0]-tracker_normal_list[i]->detboxs[j].det_vector[0],2)+ 
                                          pow(predict_result[1]-tracker_normal_list[i]->detboxs[j].det_vector[1],2));
            }
            else if(j == 1)
            {
                slave_iou_error += GetRiou_vec(predict_result, tracker_normal_list[i]->detboxs[j].det_vector);
                slave_distance_error += sqrt(pow(predict_result[0]-tracker_normal_list[i]->detboxs[j].det_vector[0],2)+ 
                                          pow(predict_result[1]-tracker_normal_list[i]->detboxs[j].det_vector[1],2));
            }
        }
        master_sum_iou_error += master_iou_error;
        master_sum_dis_error += master_distance_error;
        slave_sum_iou_error += slave_iou_error;
        slave_sum_dis_error += slave_distance_error;
    }
    if(matched_trk_num<1) return false;
    double ave_master_iou = master_sum_iou_error/matched_trk_num;
    double ave_master_dis = master_sum_dis_error/matched_trk_num;
    double ave_slave_iou = slave_sum_iou_error/matched_trk_num;
    double ave_slave_dis = slave_sum_dis_error/matched_trk_num;
    std::cout<<" Master AVERAGE IOU ERROR" << ave_master_iou << std::endl;
    std::cout<< "Master AVERAGE DIS ERROR " << ave_master_dis << std::endl;
    std::cout<<" Slave AVERAGE IOU ERROR" << ave_slave_iou << std::endl;
    std::cout<< "Slave AVERAGE DIS ERROR " << ave_slave_dis << std::endl;
    if(ave_master_iou<max_iou_error||ave_slave_iou<max_iou_error)
    {
        if(ave_master_iou<ave_slave_iou) max_iou_error = ave_master_iou;
        else max_iou_error = ave_slave_iou;
    }
    if(ave_master_dis>max_dis_error||ave_slave_dis>max_dis_error)
    {
        if(ave_master_dis>ave_slave_dis) max_dis_error = ave_master_dis;
        else max_dis_error = ave_slave_dis;
    }
    std::cout<< "Max DIS ERROR " <<max_dis_error <<std::endl;
    std::cout<< "Max IOU ERROR " <<max_iou_error<<std::endl;
    /*
    此处可加入哨岗完全失效的判定条件
    if(sum_iou_error/tracker_normal_list.size() < 0.1 ||sum_dis_error/tracker_normal_list.size()>100)
    {
        std::cout<<"SENTRY BROKEN！！！！！！！！"
    }*/
    return true;
}
//利用检测结果，我方定位数据更新跟踪结果
std::vector<detbox> R_SORTER::UpdateWithTwoCam(std::vector<std::vector<detbox>>detections,double delta_t,bool first_flag,int team_pos[4],bool team_locate_flags[2])
{
    // std::cout<<"FRAME_COUNT"<<frame_count_<<std::endl;
    frame_count_ += 1;
    // 根据相机对应的检测结果 对已有轨迹进行预测
    std::vector<tracker*> tracker_normal_list;
    std::vector<tracker*> tracker_restart_list;
    for(int i = 0; i < trackers_list.size(); i++)
    {
        if(trackers_list[i].restart_track == false)
        {
            trackers_list[i].Predict(delta_t);
            tracker_normal_list.push_back(&trackers_list[i]);
        }
        else
        {
            tracker_restart_list.push_back(&trackers_list[i]);
        }
    }
    // 如果预测的结果数目为0 那么就所有的det均为 new tracker
    // 匹配检测结果和轨迹
    // 计算得到IOU矩阵 调用riou_cal
    int trkNum = tracker_normal_list.size();
    // 一号相机
    std::vector<detbox> unmatched_detections;
    bool master_available;
    bool slave_available;
    //为有预测结果的跟踪器分配检测结果
    divide_detection(trkNum, tracker_normal_list, detections[0], unmatched_detections);
    divide_detection(trkNum, tracker_normal_list, detections[1], unmatched_detections);
    //std::cout<<"CHECK SENTRY!"<<std::endl;
    //master_available = CheckSentryAvailable(tracker_normal_list);

    //为跟踪失败的跟踪器在未分配的检测结果中寻找匹配
    for(int i = 0; i < tracker_restart_list.size(); i++)
    {
        double min_distance = 100000;
        double min_id = -1;
        for(int j = 0; j < unmatched_detections.size(); j++)
        {
            bool abandon_flag = false;
            for(int k = 0; k < 4; k++)
            {
                if(trackers_list[k].restart_track == true)
                    continue;
                double iou = GetRiou_vec(trackers_list[k].box.det_vector, unmatched_detections[j].det_vector);
                //计算旋转IOU
                if(iou > 0.3)
                    abandon_flag = true;//与跟踪成功的跟踪器重叠部分过大
                for(int f =0;f<trackers_list[k].detboxs.size();f++)
                {
                    double iou2 = GetRiou_vec(trackers_list[k].detboxs[f].det_vector, unmatched_detections[j].det_vector);
                    //与成功跟踪的跟踪器匹配的检测结果重叠部分过大
                    if(iou2>0.2)
                    {
                        abandon_flag = true;
                    }
                }
            }
            if(abandon_flag)
                continue;
            double distance = std::sqrt(std::pow(tracker_restart_list[i]->box.x_pos-unmatched_detections[j].x_pos, 2)+
                                        std::pow(tracker_restart_list[i]->box.y_pos-unmatched_detections[j].y_pos, 2));//计算与最后一次跟踪结果距离，选取最近的检测结果
            if(distance < min_distance)
            {
                min_distance = distance;
                min_id = j;
            }
        }
        if(min_id == -1 || (min_distance>100 && tracker_restart_list[i]->box.x_pos!=-1))//检测结果与最后一次跟踪结果相差过大，或者未成功初始化，匹配失败
            continue;
        //重新激活跟踪器
        tracker_restart_list[i]->restart_track = false;
        unmatched_detections[min_id].tracker_id = tracker_restart_list[i]->box.tracker_id;
        tracker_restart_list[i]->box = unmatched_detections[min_id];
        if(tracker_restart_list[i]->box.tracker_id == -1) tracker_restart_list[i]->box.tracker_id = unmatched_detections[min_id].car_type;
        //重新初始化对应输出滤波器
        Eigen::Vector2d pos(tracker_restart_list[i]->box.x_pos, tracker_restart_list[i]->box.y_pos);
        tracker_restart_list[i]->pos_kf.Init(delta_t,pos,pos_error);
        Eigen::Vector2d angle(sin(tracker_restart_list[i]->box.car_angle), cos(tracker_restart_list[i]->box.car_angle));
        tracker_restart_list[i]->angle_kf.Init(delta_t,angle,angle_error);
        unmatched_detections.erase(unmatched_detections.begin()+min_id);
    }

    for(int i = 0; i < tracker_normal_list.size(); i++)
    {
        if(tracker_normal_list[i]->detboxs.size() == 0)
        //正常的跟踪器没有匹配到检测结果，说明跟踪丢失，等待下一帧重新分配结果
        {
            tracker_normal_list[i]->reset();
        }
        else
        {
            // mix
            detbox mix_box;
            if(tracker_normal_list[i]->detboxs.size() == 2)//匹配到两个结果，融合
                mix_box = GetMixDetection(tracker_normal_list[i]->detboxs[0], tracker_normal_list[i]->detboxs[1]);
            else
                mix_box = tracker_normal_list[i]->detboxs[0];
            //更新 跟踪器内的滤波器
            Eigen::Vector2d pos(mix_box.x_pos,mix_box.y_pos);
            Eigen::Vector2d angle(sin(mix_box.car_angle),cos(mix_box.car_angle));
            tracker_normal_list[i]->box.car_type = mix_box.car_type;
            tracker_normal_list[i]->angle_kf.MeasurementUpdate(angle);
            tracker_normal_list[i]->pos_kf.MeasurementUpdate(pos);
            tracker_normal_list[i]->detboxs.clear();
            tracker_normal_list[i]->delta_x = mix_box.x_pos - tracker_normal_list[i]->box.x_pos;
            tracker_normal_list[i]->delta_y = mix_box.y_pos - tracker_normal_list[i]->box.y_pos;
        }
    }
    //防止输出的轨迹重叠
    track_iou_filter();
    if(fix_flag)
    {
        //鼠标点击，更改跟踪ID
        FixTracker(team_pos,team_locate_flags);
    }
    //输出结果
    std::vector<detbox> tracked_result;
    for(int i = 0; i < trackers_list.size(); i++)
    {
        detbox box;
        
        if(trackers_list[i].restart_track == false)
        {
            box = trackers_list[i].box;
            box.filter_flag = true;
        }
        
        else
        {
            box = trackers_list[i].box;
            
        }
        
        //box = trackers_list[i].box;
        tracked_result.push_back(box);
    }
    
    // cout<< "Finish" << endl;
    return tracked_result;
}
//利用车端定位数据修正跟踪错误
void R_SORTER::FixTracker(int team_pose[4],bool team_locate_flags[2])
{
    std::vector<int> team_wrong_id;
    std::vector<int> enemy_wrong_id;
    switch(color_flag_)
    {
        case 0:
        {
            for(int i = 0;i<trackers_list.size();i++)
            {
                if(trackers_list[i].box.tracker_id == 0)
                {
                    if(team_locate_flags[0]==false) continue;//我方机器人定位数据丢失，不修复
                    double distance = sqrt(pow(trackers_list[i].box.x_pos-team_pose[0],2)+pow(trackers_list[i].box.y_pos-team_pose[1],2));
                    if(distance > 70)
                    {
                        //我方机器人跟踪结果与定位结果差距大，存在跟踪错误可能性
                        std::cout<<"Car 1 Wrong！"<<std::endl;
                        std::cout<<"Car 1 ERROR"<<distance<<std::endl;
                        team_wrong_id.push_back(i);
                        for(int j = 0;j<trackers_list.size();j++)
                        {
                            if(trackers_list[j].box.tracker_id ==0 ||trackers_list[j].box.tracker_id==1) continue;
                            double enemy_team_dis = sqrt(pow(trackers_list[j].box.x_pos-team_pose[0],2)+pow(trackers_list[j].box.y_pos-team_pose[1],2));
                            if(enemy_team_dis<30)
                            {
                                enemy_wrong_id.push_back(j);
                                //找到与我方定位结果接近的敌方跟踪结果
                                std::cout<<"Find Wrong enemy!"<< "tracker id "<<j<<std::endl;

                            }
                        }
                    }
                }
                else if(trackers_list[i].box.tracker_id == 1)
                {
                    if(team_locate_flags[1]==false) continue;
                    double distance = sqrt(pow(trackers_list[i].box.x_pos-team_pose[2],2)+pow(trackers_list[i].box.y_pos-team_pose[3],2));
                    if(distance > 70)
                    {
                        std::cout<<"Car 2 Wrong"<<std::endl;
                        std::cout<<"Car 2 ERROR"<<distance<<std::endl;
                        team_wrong_id.push_back(i);
                        for(int j = 0;j<trackers_list.size();j++)
                        {
                            if(trackers_list[j].box.tracker_id ==0 ||trackers_list[j].box.tracker_id==1) continue;
                            double enemy_team_dis = sqrt(pow(trackers_list[j].box.x_pos-team_pose[2],2)+pow(trackers_list[j].box.y_pos-team_pose[3],2));
                            if(enemy_team_dis<30)
                            {
                                std::cout<<"Find Wrong enemy!"<< "tracker id "<<j<<std::endl;
                                enemy_wrong_id.push_back(j);
                            }
                        }
                    }
                }
            }
            if(team_wrong_id.size()==2 && enemy_wrong_id.size() ==0)
            {
                //我方两个机器人均跟踪错误而敌方跟踪结果不存在我方机器人，说明我方跟踪交换
                bool switch_teammate_flag = false;
                int trackid_one = team_wrong_id[0];
                if(trackers_list[trackid_one].box.tracker_id  == 0)
                {
                    double dis2teamate = sqrt(pow(trackers_list[trackid_one].box.x_pos-team_pose[2],2)+pow(trackers_list[trackid_one].box.y_pos-team_pose[3],2));
                    double dis2myself =  sqrt(pow(trackers_list[trackid_one].box.x_pos-team_pose[0],2)+pow(trackers_list[trackid_one].box.y_pos-team_pose[1],2));
                    if(dis2teamate<dis2myself) switch_teammate_flag = true;
                }
                else if(trackers_list[trackid_one].box.tracker_id  == 1)
                {
                    double dis2teamate = sqrt(pow(trackers_list[trackid_one].box.x_pos-team_pose[0],2)+pow(trackers_list[trackid_one].box.y_pos-team_pose[1],2));
                    double dis2myself =  sqrt(pow(trackers_list[trackid_one].box.x_pos-team_pose[2],2)+pow(trackers_list[trackid_one].box.y_pos-team_pose[3],2));
                    if(dis2teamate<dis2myself) switch_teammate_flag = true;
                }

                if(switch_teammate_flag)
                {
                    std::cout<<"Team Mate Switch"<<std::endl;
                    int temp_track_id;
                    int track_id_1 = team_wrong_id[0];
                    int track_id_2 = team_wrong_id[1];
                    temp_track_id = trackers_list[track_id_1].box.tracker_id;
                    trackers_list[track_id_1].box.tracker_id = trackers_list[track_id_2].box.tracker_id;
                    trackers_list[track_id_2].box.tracker_id=  temp_track_id;

                }

            }
            else if(team_wrong_id.size() == enemy_wrong_id.size())
            //敌我跟踪交换
            {
                if(enemy_wrong_id.size()>0)
                {
                     std::cout<<"Team Enemy Switch"<<"Size: "<<enemy_wrong_id.size()<<std::endl;
                }
                for(int k =0;k<team_wrong_id.size();k++)
                {
                    int temp_track_id;
                    int track_id_team = team_wrong_id[k];
                    int track_id_enemy = enemy_wrong_id[k];
                    temp_track_id = trackers_list[track_id_team].box.tracker_id;
                    trackers_list[track_id_team].box.tracker_id = trackers_list[track_id_enemy].box.tracker_id;
                    trackers_list[track_id_enemy].box.tracker_id= temp_track_id;
                }
            }
        }
            break;
        //由于存在误检测，目前哨岗无法解决敌方之间交换的问题
        case 2:
        {
            for(int i = 0;i<trackers_list.size();i++)
            {
                if(trackers_list[i].box.tracker_id ==2)
                {
                    if(team_locate_flags[0]==false) continue;
                    double distance = sqrt(pow(trackers_list[i].box.x_pos-team_pose[0],2)+pow(trackers_list[i].box.y_pos-team_pose[1],2));
                    
                    if(distance > 70)
                    {
                        std::cout<<"Car 1 Wrong！"<<std::endl;
                        std::cout<<"Car 1 ERROR"<<distance<<std::endl;
                        team_wrong_id.push_back(i);
                        for(int j = 0;j<trackers_list.size();j++)
                        {
                            if(trackers_list[j].box.tracker_id ==2 ||trackers_list[j].box.tracker_id==3) continue;
                            double enemy_team_dis = sqrt(pow(trackers_list[j].box.x_pos-team_pose[0],2)+pow(trackers_list[j].box.y_pos-team_pose[1],2));
                            if(enemy_team_dis<30)
                            {
                                std::cout<<"Find Wrong enemy!"<< "tracker id "<<j<<std::endl;
                                enemy_wrong_id.push_back(j);
                            }
                        }
                    }
                }
                else if(trackers_list[i].box.tracker_id ==3)
                {
                    if(team_locate_flags[1]==false) continue;
                    double distance = sqrt(pow(trackers_list[i].box.x_pos-team_pose[2],2)+pow(trackers_list[i].box.y_pos-team_pose[3],2));
                    if(distance > 70)
                    {
                        std::cout<<"Car 2 Wrong"<<std::endl;
                        std::cout<<"Car 2 ERROR"<<distance<<std::endl;
                        team_wrong_id.push_back(i);
                        for(int j = 0;j<trackers_list.size();j++)
                        {
                            if(trackers_list[j].box.tracker_id ==2 ||trackers_list[j].box.tracker_id==3) continue;
                            double enemy_team_dis = sqrt(pow(trackers_list[j].box.x_pos-team_pose[2],2)+pow(trackers_list[j].box.y_pos-team_pose[3],2));
                            if(enemy_team_dis<30)
                            {
                                std::cout<<"Find Wrong enemy!"<< "tracker id "<<"j"<<std::endl;
                                enemy_wrong_id.push_back(j);
                            }
                        }
                    }
                }
            }
            if(team_wrong_id.size()==2 && enemy_wrong_id.size() ==0)
            {
                bool switch_teammate_flag = false;
                int trackid_one = team_wrong_id[0];
                std::cout<<"Track id one " << trackid_one << std::endl;
                if(trackers_list[trackid_one].box.tracker_id == 2)
                {
                    double dis2teamate = sqrt(pow(trackers_list[trackid_one].box.x_pos-team_pose[2],2)+pow(trackers_list[trackid_one].box.y_pos-team_pose[3],2));
                    double dis2myself =  sqrt(pow(trackers_list[trackid_one].box.x_pos-team_pose[0],2)+pow(trackers_list[trackid_one].box.y_pos-team_pose[1],2));
                    std::cout<< " dis2teamate "<< dis2teamate<<std::endl;
                    std::cout<< " dis2myself "<< dis2myself<<std::endl;
                    if(dis2teamate<dis2myself) switch_teammate_flag = true;
                }
                else if(trackers_list[trackid_one].box.tracker_id == 3)
                {
                    double dis2teamate = sqrt(pow(trackers_list[trackid_one].box.x_pos-team_pose[0],2)+pow(trackers_list[trackid_one].box.y_pos-team_pose[1],2));
                    double dis2myself =  sqrt(pow(trackers_list[trackid_one].box.x_pos-team_pose[2],2)+pow(trackers_list[trackid_one].box.y_pos-team_pose[3],2));
                    std::cout<< " dis2teamate "<< dis2teamate<<std::endl;
                    std::cout<< " dis2myself "<< dis2myself<<std::endl;
                    if(dis2teamate<dis2myself) switch_teammate_flag = true;
                }
                if(switch_teammate_flag)
                {
                    std::cout<<"Team Mate Switch"<<std::endl;
                    int temp_track_id;
                    int track_id_1 = team_wrong_id[0];
                    int track_id_2 = team_wrong_id[1];
                    temp_track_id = trackers_list[track_id_1].box.tracker_id;
                    trackers_list[track_id_1].box.tracker_id = trackers_list[track_id_2].box.tracker_id;
                    trackers_list[track_id_2].box.tracker_id=  temp_track_id;

                }
            }

            else if(team_wrong_id.size() == enemy_wrong_id.size())
            {
                if(team_wrong_id.size()>0)
                {
                     std::cout<<"Team Enemy Switch"<<"Size: "<<team_wrong_id.size()<<std::endl;
                }
                for(int k =0;k<team_wrong_id.size();k++)
                {
                    int temp_track_id;
                    int track_id_team = team_wrong_id[k];
                    int track_id_enemy = enemy_wrong_id[k];
                    temp_track_id = trackers_list[track_id_team].box.tracker_id;
                    trackers_list[track_id_team].box.tracker_id = trackers_list[track_id_enemy].box.tracker_id;
                    trackers_list[track_id_enemy].box.tracker_id= temp_track_id;
                }
            }

        }
            break;
        default:
            break;
    }
}
//检查初始4个跟踪器是否满足要求
bool R_SORTER::CheckStartTracker(std::vector<detbox> trackresult)
 {
    double min_distance = 10000;
    for(int i =0;i<trackresult.size();i++)
    {
        for(int j = i+1;j<trackresult.size();j++)
        {
            double temp_dis = std::sqrt(std::pow((trackresult[i].x_pos-trackresult[j].x_pos),2)+
                                        std::pow((trackresult[i].y_pos-trackresult[j].y_pos),2));
            if(temp_dis<min_distance)
            {
                min_distance = temp_dis;
            }
        }
    }
    std::cout<<"Min distance: "<< min_distance<<std::endl;
    //四个跟踪器距离太近说明出现重叠，不满足，重新初始化
    //实际测试：有时需要场上干扰较少时才能初始化
    if(min_distance < 80)
    {
        return false;
    }
    else
    {
        return true;
    }
   
 }


 OutPutFilter::OutPutFilter()
 {
     for(int i = 0;i< 4;i++)
     {
        filter_flag[i] = false;
     }
 }
OutPutFilter::~OutPutFilter()
 {

 }
 //初始化输出滤波器
 bool OutPutFilter::InitOutputFilter(std::vector<detbox> &tracked_result)
 {
     for(int i = 0;i<tracked_result.size();i++)
     {
         if(tracked_result[i].x_pos == -1)
         {
             int filter_id = tracked_result[i].tracker_id;
             filter_flag[filter_id] == false;
         }
         else
         {
            int filter_id = tracked_result[i].tracker_id;
            filter_flag[filter_id] == true;
            Eigen::Vector2d pos(tracked_result[i].x_pos,tracked_result[i].y_pos);
            tracker_filters[i].Init(1,pos,0.01);
         }
     }
     for(int j= 0;j<4; j++)
     {
         if(filter_flag[j]==false)
         {
             return false;
         } 
     }
     return true;
     //四个Tracker输出滤波器初始化成功返回
 }
//输出滤波器预测
void OutPutFilter::TrackerPredict(double dT,std::vector<detbox> &tracked_result)
{
    std::vector<Eigen::Vector2d> pos_data;
    for(int i =0;i<tracked_result.size();i++)
    {
        if(tracked_result[i].filter_flag == false) continue;
        else{
            int filter_id  = tracked_result[i].tracker_id;
            tracker_filters[i].Predict(dT);
        }
    }

}
//输出滤波器后验更新
void OutPutFilter::TrackerFixUpdate(std::vector<detbox> &tracked_result)
{
    for(int i =0;i<4;i++)
    {
        int filter_id = tracked_result[i].tracker_id;
        Eigen::Vector2d pos_data(tracked_result[i].x_pos,tracked_result[i].y_pos);
        if(filter_flag[filter_id]==true)
        {
            tracker_filters[filter_id].MeasurementUpdate(pos_data);
            Eigen::Vector2d filtered_pos = tracker_filters[filter_id].GetBox();
            tracked_result[i].x_pos = filtered_pos[0];
            tracked_result[i].y_pos = filtered_pos[1];
        }
    }
}
 