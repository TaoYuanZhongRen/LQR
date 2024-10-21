#include "LQRControl.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>


LQRCalculator::LQRCalculator()
{

}

/**
 * https://github.com/TakaHoribe/Riccati_Solver/blob/master/riccati_solver.cpp
 * @brief Computes the LQR gain matrix (usually denoted K) for a discrete time
 * infinite horizon problem.
 *
 * @param A State matrix of the underlying system
 * @param B Input matrix of the underlying system
 * @param Q Weight matrix penalizing the state
 * @param R Weight matrix penalizing the controls
 * @param K Pointer to the generated matrix (has to be a double/dynamic size
 * matrix!)
 * @param eps Delta between iterations that determines when convergence is
 * reached
 */

void LQRCalculator::DARE(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R, Eigen::MatrixXd* K, const double eps)
{
    // check if dimensions are compatible
    if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() ||
        Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols())
    {
        
    }
    Eigen::MatrixXd P = Q; // initialize

    Eigen::MatrixXd P_next;
    Eigen::MatrixXd A_T = A.transpose();
    Eigen::MatrixXd B_T = B.transpose();
    Eigen::MatrixXd Rinv = R.inverse();

    double diff;
    while (true)
    {
        // -- discrete solver --
        P_next = A_T * P * A - A_T * P * B * (R + B_T * P * B).inverse() * B_T * P * A + Q;
        diff = fabs((P_next - P).maxCoeff());
        P = P_next;
        if (diff < eps)
        {
            break;
        }
    }

    // compute K from P
    *K = (R + B_T * P * B).inverse() * (B_T * P * A);
}

void LQRCalculator::DLQR_Control(ControlModelParam* param)
{
    Eigen::Vector3d des_acc(0.0, 0.0, 0.0);      //期望加速度
    // A矩阵
    // [[1, 0, dt, 0],
    //  [0, 1, 0, dt],
    //  [0, 0, 1, 0],
    //  [0, 0, 0, 1]]
    A = Eigen::MatrixXd::Identity(4, 4);
    A(0, 2) = dt;
    A(1, 3) = dt;
    B = Eigen::MatrixXd::Zero(4, 2);
    B(2, 0) = dt;
    B(3, 1) = dt;
    // B矩阵
    // [[0, 0],
    // [0, 0],
    // [dt, 0],
    // [0, dt]]
    Q = Eigen::MatrixXd::Identity(4, 4);
    R = 0.4 * Eigen::MatrixXd::Identity(2, 2);
    DARE(A, B, Q, R, &K, 1e-15);

    const Eigen::Vector4d state_des = Eigen::Vector4d(param->desired[0], param->desired[1], param->desired_velocity[0], param->desired_velocity[1]);
    const Eigen::Vector4d state_now = Eigen::Vector4d(param->position[0], param->position[1], param->velocity[0], param->velocity[1]);
    const Eigen::Vector2d des_a = Eigen::Vector2d(param->des_a[0], param->des_a[1]);
    Eigen::Vector2d u_acc = -K * (state_now - state_des) + des_a;

    param->thrust = computeDesiredThrust(param->velocity[2],param->desired_velocity[2], param->position[2],param->desired[2]);
    for (size_t i = 0; i < 3; i++)
    {
        param->angle_cmd[i] = Calculate_horizon_angelCmd(u_acc, param->euler[2])[i];
    }
}
double LQRCalculator::computeDesiredThrust(double v_z_now, double v_z_des,double pos_z_now,double pos_z_des)
{
    double P = 0, I = 0, D = 0;
   
    double pos_z_bias = 2.0 * (pos_z_des - pos_z_now);
    double v_z_bias_new = pos_z_bias - v_z_now;

    //0.08  0.0089两个参数是师兄那里PID调参调出来的
    //这里z轴控制采用的是简单的单环PID控制
    P = 0.08 * v_z_bias_new;
    I = (v_z_bias_new + v_z_bias_old) * 0.5 * dt;
    D = 0.0089 * (v_z_bias_new - v_z_bias_old) / dt;
    v_z_bias_old = v_z_bias_new;
    thrust = - (P + I + D - hover_thrust);
    if (thrust <= 0.2)
    {
        thrust = 0.2;
    }
    return thrust;
}
Eigen::Vector3d LQRCalculator::Calculate_horizon_angelCmd(Eigen::Vector2d out_acc, const double yaw_my)
{
    // 读取自身角度
    const double g = 9.81;  // 重力加速度
    double sin_yaw = std::sin(yaw_my);
    double cos_yaw = std::cos(yaw_my);

    // 构建变换矩阵 A_psi 和其逆矩阵 A_psi_inverse
    Eigen::Matrix2d A_psi;
    A_psi << sin_yaw, cos_yaw,
        -cos_yaw, sin_yaw;
    Eigen::Matrix2d A_psi_inverse = A_psi.inverse();

    // 计算角度命令
    Eigen::Vector2d angle_h_cmd = (1.0 / g) * A_psi_inverse * Eigen::Vector2d(-out_acc[0], -out_acc[1]);
    double a_x_cmd = std::atan(angle_h_cmd(0));
    double a_y_cmd = std::atan(angle_h_cmd(1));

    Eigen::Vector3d angle_cmd = std::move(Eigen::Vector3d(a_x_cmd, a_y_cmd, 0));
    return angle_cmd;
}

