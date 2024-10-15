#pragma once
#include <Eigen/Dense>
#include "drone_model.h"

constexpr double STEP = 0.04;

class LQRCalculator
{
public:
    LQRCalculator();

    void DLQR_Control(ControlModelParam* param);
    void DARE(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R, Eigen::MatrixXd* K, const double eps);
    Eigen::Vector3d Calculate_horizon_angelCmd(Eigen::Vector2d out_acc, const double yaw_my);
    double computeDesiredThrust(double v_now_z, double v_des_z, double pos_now_z, double pos_des_z);


private:

    double dt = STEP;
    double gra = 9.81;
    Eigen::MatrixXd A, B, Q, R, K;
    double v_z_bias_old = 0;
    double hover_thrust = 0.61;
    double thrust = 0;
};

