#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <queue>
#include "drone_model.h"
struct Desired_State_t
{
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    double yaw;
    double yaw_rate;
    double head_rate;

    ~Desired_State_t() {};

    Desired_State_t()
        : p(Eigen::Vector3d::Zero()),
        v(Eigen::Vector3d::Zero()),
        a(Eigen::Vector3d::Zero()),
        yaw(0),
        yaw_rate(0),
        head_rate(0) {};
};

struct Controller_Output_t
{
    // Body rates in body frame
    Eigen::Vector3d bodyrates; // [rad/s]
    double roll_rate;
    double pitch_rate;
    double yaw_rate;
    double normalized_thrust;
    // Collective mass normalized thrust
    double thrust;

    Eigen::Vector3d angle_cmd;
};

class Now_State_t
{
public:
    Eigen::Vector3d p;   //位置
    Eigen::Vector3d v;   //速度
    Eigen::Vector3d w;   //角度度
    Eigen::Vector3d a;   //加速度
    double euler[3]{ 0 };

    Now_State_t();
};

class LQRCalculator
{
public:
    LQRCalculator();

    double dt = 0.2;
    Eigen::MatrixXd A, B, Q, R, K;

    void DLQR_Control(ControlModelParam* param);

    void DARE(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R, Eigen::MatrixXd* K, const double eps);
    Eigen::Vector3d Calculate_horizon_angelCmd(Eigen::Vector2d out_acc, const double yaw_my);
    double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d& des_acc);
private:


    // Thrust-accel mapping params
    const double rho2_ = 0.998; // do not change
    double thr2acc_ = gra / 0.72;
    double P_ = 1e6;
    double gra = 9.81;

};

