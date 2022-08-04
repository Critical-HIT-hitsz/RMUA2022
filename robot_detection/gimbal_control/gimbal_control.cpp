/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.

 ***************************************************************************/
#include <cmath>
#include <stdio.h>

#include "gimbal_control.h"

namespace robot_detection {

    void GimbalContrl::Init(double x, double y, double z,
                            double init_v,
                            double init_k) {
        offset_.x = x;
        offset_.y = y;
        offset_.z = z;
        init_v_ = init_v;
        init_k_ = init_k;
    }


//air friction is considered
    double GimbalContrl::BulletModel(double x, double v, double angle) { //x:m,v:m/s,angle:rad
        double t, y;
        if (init_k_ == 0) ROS_WARN("init_k_ = 0! ");
        if (v == 0) ROS_WARN("v = 0! ");
        if (cos(angle) == 0) ROS_WARN("cos(angle) = 0! ");
        t = (double) ((exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle)));
        y = (double) (v * sin(angle) * t - GRAVITY * t * t / 2);
        return y;
    }

//x:distance , y: height
    double GimbalContrl::GetPitch(double x, double y, double v) {
        double y_temp, y_actual, dy;
        double a;
        y_temp = y;
        // by iteration
        for (int i = 0; i < 20; i++) {
            a = (double) atan2(y_temp, x);
            y_actual = BulletModel(x, v, a);
            dy = y - y_actual;
            y_temp = y_temp + dy;
            if (fabsf(dy) < 0.001) {
                break;
            }
        }
        return a;

    }

    void GimbalContrl::Transform(cv::Point3f &postion, double &pitch, double &yaw) {
        pitch = -GetPitch((postion.z + offset_.z) / 1000, -(postion.y + offset_.y) / 1000, init_v_) * 180 / M_PI;
        //yaw positive direction :anticlockwise
        yaw = -(double) (atan2(postion.x + offset_.x, postion.z + offset_.z)) * 180 / M_PI;
    }

} // roborts_detection



