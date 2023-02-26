#pragma once

#include <Eigen/Core>
#include <kdl/frames.hpp>

#include <dh_ros_tools/stopwatch.hpp>
#include <dh_kdl/treekdlmodel.hpp>
#include <dh_linear_control/c2d/rk4.hpp>
#include <dh_linear_control/mpc/linear_dense.hpp>
#include <dh_kdl_msgs/PoseVel.h>

#include "./dynamics.hpp"
#include "./utils.hpp"

class RotationController
{
public:
  RotationController(const KDL::Tree& tree);

  void updateInternalDataStructures();

  void update(
    const dh_kdl_msgs::PoseVel& bs,
    const KDL::JntArray& q,
    const double& U,
    const double& roll_des,
    const double& pitch_des,
    const double& yaw_des,
    std::vector<double>& u_opt);

  void reconfigure(
    double pred_horizon,
    uint32_t pred_steps,
    double rot_decay,
    double angvel_decay,
    double rot_weight,
    double angvel_weight,
    int thrust_weight,
    int thrust_rate_weight);

private:
  KDL::TreeKDLModel kdl_model_;
  double mass_;

  // 静的ROSパラメータ
  const uint32_t num_rotors_;
  const std::vector<RotorProperty> rotor_props_;

  // 動的ROSパラメータに依存するパラメータ
  double dt_;                   // MPCの離散化間隔
  uint32_t Hp_;                 // 予測区間の分割数
  std::vector<double> T_refs_;  // 制御変数の誤差の減衰時定数[sec]
  Eigen::VectorXd Q_;
  Eigen::VectorXd S_;
  Eigen::VectorXd R_;

  MultiRotorDynamics cont_;                  // 連続時間線形状態方程式
  std::vector<ctrl::LinearDynamics> discs_;  // 離散時間線系状態方程式
  ctrl::C2D_RK4 c2d_;                        // 状態方程式を離散化
  Eigen::VectorXd x_;
  Eigen::VectorXd s_;
  Eigen::VectorXd u_;

  const Eigen::MatrixXd Cz_;
  const ctrl::LinearEquation E_e_;
  ctrl::LinearEquation F_f_;
  const ctrl::LinearEquation G_g_;

  dh_ros::Stopwatch stopwatch_;  // 計算時間計測用

  void updateDynamics(
    const double& roll,
    const double& pitch,
    const double& roll_des,
    const double& pitch_des,
    const KDL::JntArray& q);
  void updateX(const dh_kdl_msgs::PoseVel& bs);
  void updateS(const double& roll_des, const double& pitch_des, const double& yaw_des);
  void updateWeight_Q(double rot_weight, double angvel_weight);
  void updateWeight_S(int thrust_weight);
  void updateWeight_R(int thrust_rate_weight, double dt);
  ctrl::LinearEquation makeBaseInputCondition();
  void updateInputCondition(const double& U);
};
