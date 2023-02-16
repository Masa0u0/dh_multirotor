#pragma once

#include <memory>
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
    double U,
    double roll_des,
    double pitch_des,
    double yaw_des,
    std::vector<double>& u_opt);

private:
  // static rosparam
  const int num_rotors_;
  const std::vector<RotorProperty> rotor_props_;

  // dynamic rosparam
  double pred_horizon_;  // 予測区間[sec]
  int Hp_;               // 予測区間の分割数
  int Hu_;               // 制御入力の有効ステップ
  double weight_scale_;
  std::vector<double> T_refs_;  // 制御変数の誤差の減衰時定数[sec]
  std::vector<double> Q_values_;
  double R_value_;
  double S_value_;

  KDL::TreeKDLModel kdl_model_;
  double dt_;                                // MPCの離散化間隔
  MultiRotorDynamics cont_;                  // 連続時間線形状態方程式
  std::vector<ctrl::LinearDynamics> discs_;  // 離散時間線系状態方程式
  ctrl::C2D_RK4 c2d_;                        // 状態方程式を離散化
  Eigen::VectorXd x_;
  Eigen::VectorXd s_;

  Eigen::MatrixXd Cz_;
  Eigen::VectorXd R_;
  Eigen::VectorXd S_;
  Eigen::VectorXd Q_;
  ctrl::LinearEquation E_e_;
  ctrl::LinearEquation F_f_;
  ctrl::LinearEquation G_g_;

  dh_ros::Stopwatch stopwatch_;  // 計算時間計測用

  void getParams();
  void updateDynamics(double roll, double pitch, double roll_des, double pitch_des);
  void updateX(const dh_kdl_msgs::PoseVel& bs);
  void updateS(double roll_des, double pitch_des, double yaw_des);
  Eigen::MatrixXd makeCz();
  Eigen::VectorXd makeWeight(const std::vector<double>& values, const std::vector<double>& scales);
  Eigen::VectorXd makeWeight_R();
  Eigen::VectorXd makeWeight_S();
  Eigen::VectorXd makeWeight_Q();
  ctrl::LinearEquation makeBaseInputCondition();
  void updateInputCondition(double U);
};
