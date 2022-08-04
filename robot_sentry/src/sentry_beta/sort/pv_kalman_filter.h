#ifndef RYOLO_KALMAN_FILTER_H
#define RYOLO_KALMAN_FILTER_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>


class PVKalmanFilter
{
    public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            PVKalmanFilter();

            void Init(double dT, Eigen::Vector2d& pos,double R_error);

            void Predict(double dT);

            void MeasurementUpdate(Eigen::Vector2d z);

            Eigen::Vector4d GetState();

            Eigen::Vector2d GetBox();

            Eigen::Vector4d GetPredictState(double dT);

            Eigen::Vector2d GetMeasurement(double dT);

            Eigen::Vector4d x_;
            Eigen::Vector2d z_;

            Eigen::Matrix4d F_;
            Eigen::Matrix4d P_;
            Eigen::Matrix<double, 2, 4> H_;
            Eigen::Matrix4d Q_;
            Eigen::Matrix2d R_;
            Eigen::Matrix4d I_KH;
            Eigen::Matrix<double, 4, 2> K_;
            double P_set_error;
            double Q_set_error;
            double R_set_error;

            Eigen::Matrix4d I_;

            double nominal_dT_;

            // 0 for angle 1 for pos
};


#endif 
