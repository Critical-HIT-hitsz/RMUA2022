#include "pv_kalman_filter.h"
#include <iostream>
PVKalmanFilter:: PVKalmanFilter()
{

}

void PVKalmanFilter::Init(double dT, Eigen::Vector2d& pos,double R_error)
{
    nominal_dT_ = dT;
    F_  << 1.0,    0.0 ,     dT,        0.0,
                    0.0,   1, 0.0,   dT,
                    0.0,  0.0,   1.0 ,   0.0,
                    0.0,  0.0,   0.0 ,   1.0;
    H_ << 1.0,  0.0,   0.0 ,   0.0,
                    0.0,  1.0,   0.0 ,   0.0;
    P_set_error = 0.01;
    Q_set_error  = 5000.0;
    R_set_error = R_error;
    double p_pos =  1/pow(P_set_error,1);
    double p_vel =    1/pow(P_set_error,4);
    P_ << p_pos,   0,      0,     0,
                    0,  p_pos,      0,     0,
                    0,   0,   p_vel,     0,
                    0,   0,   0,    p_vel;
    double q_pos =double(1.0/3)*pow(dT,3)*Q_set_error;
    double q_vel =  (pow(dT,1)*Q_set_error);
    double q_pos_vel =  0.5*pow(dT,2)*Q_set_error;
    Q_ << q_pos,   0,     q_pos_vel,     0,
                    0,   q_pos,      0,     q_pos_vel,
                    q_pos_vel,   0,   q_vel,     0,
                    0,   q_pos_vel,   0,     q_vel;
    x_ << pos(0),pos(1),0,0;
    z_ << 0 , 0;
    R_ << R_set_error ,   0,
                0,      R_set_error;

    I_ = Eigen::MatrixXd::Identity(4,4);

}

    void PVKalmanFilter::Predict(double dT)
    {
        F_  << 1.0,    0.0 ,     dT,        0.0,
                    0.0,   1, 0.0,   dT,
                    0.0,  0.0,   1.0 ,   0.0,
                    0.0,  0.0,   0.0 ,   1.0;
        x_ = F_ * x_;
        Q_set_error  = 800.0;
        double q_pos = (double(1.0/3))*pow(dT,3)*Q_set_error;
        double q_vel =  (pow(dT,1)*Q_set_error);
        double q_pos_vel =  0.5*pow(dT,2)*Q_set_error;
        Q_ << q_pos,   0,     q_pos_vel,     0,
                    0,   q_pos,      0,     q_pos_vel,
                    q_pos_vel,   0,   q_vel,     0,
                    0,   q_pos_vel,   0,     q_vel;
        //std::cout<< "Q _ MATRIX" <<  Q_<< std::endl;
        P_ = F_*P_*F_.transpose()+Q_;
    }

    void PVKalmanFilter::MeasurementUpdate(Eigen::Vector2d z)
    {
        z_ = z;

        //std::cout<< "Get measurement: "<<z_<<std::endl;

        Eigen::Matrix2d New_P_ = (H_*P_*H_.transpose()+R_).inverse();

        K_ = P_ * H_.transpose() *New_P_;

        x_ = x_ + K_ * (z_ - H_*x_);


        //std::cout<< "Get update_x: "<<x_<<std::endl;

        I_KH = I_ - K_*H_;

        P_ = I_KH *  P_ * I_KH.transpose()  + K_ * R_ *K_.transpose();


    }

    Eigen::Vector4d PVKalmanFilter::GetState()
    {
        return x_;

    }

    Eigen::Vector4d PVKalmanFilter::GetPredictState(double dT)
    {
        return F_ * x_;

    }

    Eigen::Vector2d PVKalmanFilter::GetMeasurement(double dT)
    {
        return z_;

    }
    
    Eigen::Vector2d PVKalmanFilter::GetBox()
    {
        Eigen::Vector2d box(x_(0),x_(1));
        return box;

    }
