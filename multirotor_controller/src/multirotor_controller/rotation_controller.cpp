#include <dh_std_tools/vector.hpp>
#include <dh_eigen_tools/core.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_linear_control/util.hpp>

#include "../../include/multirotor_controller/rotation_controller.hpp"

#define WEIGHT_SCALER 1e+6  // QPの数値エラーを防ぐために重みにかける定数

using namespace std;
using namespace Eigen;
using namespace KDL;

RotationController::RotationController(const Tree& tree)
  : num_rotors_(dh_ros::getParam<int>("/num_rotors")),
    rotor_props_(getRotorProperties()),
    T_refs_(X_DIM),
    kdl_model_(tree),
    cont_(tree),
    c2d_(X_DIM, num_rotors_),
    x_(X_DIM),
    s_(X_DIM),
    u_(VectorXd::Zero(num_rotors_)),
    Cz_(MatrixXd::Identity(X_DIM, X_DIM)),
    Q_(X_DIM),
    S_(num_rotors_),
    R_(num_rotors_),
    E_e_(ctrl::LinearEquation(num_rotors_, 0)),
    F_f_(makeBaseInputCondition()),
    G_g_(ctrl::LinearEquation(X_DIM, 0))
{
}

void RotationController::updateInternalDataStructures()
{
  kdl_model_.updateInternalDataStructures();
  cont_.updateInternalDataStructures();

  mass_ = kdl_model_.treeMass();

  // 動的rosparamの初期値を取得
  auto pred_horizon = dh_ros::getParam<double>("~prediction_horizon");
  auto pred_steps = dh_ros::getParam<int>("~prediction_steps");
  auto rot_decay = dh_ros::getParam<double>("~rotation_decay");
  auto angvel_decay = dh_ros::getParam<double>("~angular_velocity_decay");
  auto rot_weight = dh_ros::getParam<double>("~rotation_weight");
  auto angvel_weight = dh_ros::getParam<double>("~angular_velocity_weight");
  auto thrust_weight = dh_ros::getParam<int>("~thrust_force_weight");
  auto thrust_rate_weight = dh_ros::getParam<int>("~thrust_force_rate_weight");

  // 内部変数を更新
  reconfigure(
    pred_horizon, pred_steps, rot_decay, angvel_decay, rot_weight, angvel_weight, thrust_weight,
    thrust_rate_weight);
}

void RotationController::update(
  const dh_kdl_msgs::PoseVel& bs,
  const JntArray& q,
  const double& U,
  const double& roll_des,
  const double& pitch_des,
  const double& yaw_des,
  vector<double>& u_opt)
{
  ROS_ASSERT(q.rows() == kdl_model_.getNrOfJoints());
  ROS_ASSERT(U >= 0.);
  ROS_ASSERT(u_opt.size() == num_rotors_);

  const auto& rpy = bs.pose.rpy;
  updateDynamics(rpy.x(), rpy.y(), roll_des, pitch_des, q);
  updateX(bs);
  updateS(roll_des, pitch_des, yaw_des);

  updateInputCondition(U);

  // stopwatch_.start();
  u_ = ctrl::solveLinearDenseMPC(
    discs_, Cz_, Hp_, Hp_, dt_, T_refs_, R_, S_, Q_, E_e_, F_f_, G_g_, x_, s_, u_);
  u_opt = eigen_tools::toStdVector(u_);
  // stopwatch_.stop();
}

void RotationController::reconfigure(
  double pred_horizon,
  uint32_t pred_steps,
  double rot_decay,
  double angvel_decay,
  double rot_weight,
  double angvel_weight,
  int thrust_weight,
  int thrust_rate_weight)
{
  ROS_ASSERT(pred_horizon > 0.);
  ROS_ASSERT(pred_steps > 0);
  ROS_ASSERT(rot_decay >= 0.);
  ROS_ASSERT(angvel_decay >= 0.);
  ROS_ASSERT(rot_weight > 0.);
  ROS_ASSERT(angvel_weight > 0.);

  dt_ = pred_horizon / pred_steps;
  Hp_ = pred_steps;
  T_refs_[ROLL] = T_refs_[PITCH] = T_refs_[YAW] = rot_decay;
  T_refs_[ANGVEL_X] = T_refs_[ANGVEL_Y] = T_refs_[ANGVEL_Z] = angvel_decay;

  discs_.resize(pred_steps, ctrl::LinearDynamics(X_DIM, num_rotors_));

  updateWeight_Q(rot_weight, angvel_weight);
  updateWeight_S(thrust_weight);
  updateWeight_R(thrust_rate_weight, dt_);
}

void RotationController::updateDynamics(
  const double& roll,
  const double& pitch,
  const double& roll_des,
  const double& pitch_des,
  const JntArray& q)
{
  double t;
  double roll_k, pitch_k;

  for (int k = 0; k < Hp_; ++k)
  {
    t = dt_ * k;  // 計画開始時刻(= 0)からの経過時間

    // 時刻tにおけるドローンの姿勢の参照値
    roll_k = ctrl::firstOrderPos(roll, roll_des, T_refs_[ROLL], t);
    pitch_k = ctrl::firstOrderPos(pitch, pitch_des, T_refs_[PITCH], t);

    cont_.update(roll_k, pitch_k, q);
    discs_[k] = c2d_.convert(cont_, dt_);
  }
}

void RotationController::updateX(const dh_kdl_msgs::PoseVel& bs)
{
  const auto& r = bs.pose.rpy;
  const auto& w = bs.twist.rot;
  x_ << r.x(), r.y(), r.z(), w.x(), w.y(), w.z();
}

void RotationController::updateS(
  const double& roll_des,
  const double& pitch_des,
  const double& yaw_des)
{
  s_ << roll_des, pitch_des, yaw_des, 0., 0., 0.;
}

void RotationController::updateWeight_Q(double rot_weight, double angvel_weight)
{
  // 姿勢と角速度のスケールは機体によって変化しないため，Qはスケーリングしない
  Q_(ROLL) = Q_(PITCH) = Q_(YAW) = rot_weight * WEIGHT_SCALER;
  Q_(ANGVEL_X) = Q_(ANGVEL_Y) = Q_(ANGVEL_Z) = angvel_weight * WEIGHT_SCALER;
}

void RotationController::updateWeight_S(int thrust_weight)
{
  double u_scale = mass_ * GRAVITY;
  double S_value = pow(10, thrust_weight) / sqr(u_scale) * WEIGHT_SCALER;

  for (uint32_t i = 0; i < num_rotors_; ++i)
  {
    S_(i) = S_value;
  }
}

void RotationController::updateWeight_R(int thrust_rate_weight, double dt)
{
  double delta_u_scale = mass_ * GRAVITY * dt;
  double R_value = pow(10, thrust_rate_weight) / sqr(delta_u_scale) * WEIGHT_SCALER;

  for (uint32_t i = 0; i < num_rotors_; ++i)
  {
    R_(i) = R_value;
  }
}

ctrl::LinearEquation RotationController::makeBaseInputCondition()
{
  const double min_thrust = 0.;
  const MatrixXd E = MatrixXd::Identity(num_rotors_, num_rotors_);
  const VectorXd ones = VectorXd::Ones(num_rotors_);

  ctrl::LinearEquation F_f(num_rotors_, num_rotors_ * 2 + 2);

  F_f.A.block(0, 0, num_rotors_, num_rotors_) = E;
  F_f.A.block(num_rotors_, 0, num_rotors_, num_rotors_) = -E;
  F_f.A.block(num_rotors_ * 2, 0, 1, num_rotors_) = ones.transpose();
  F_f.A.block(num_rotors_ * 2 + 1, 0, 1, num_rotors_) = -ones.transpose();

  for (int i = 0; i < num_rotors_; ++i)
  {
    string rotor_name = "/rotor_" + to_string(i);
    double max_thrust = rotor_props_[i].motor_constant * sqr(rotor_props_[i].max_velocity);
    F_f.b(i) = max_thrust;
    F_f.b(num_rotors_ + i) = -min_thrust;
  }

  return F_f;
}

void RotationController::updateInputCondition(const double& U)
{
  F_f_.b(num_rotors_ * 2) = U;
  F_f_.b(num_rotors_ * 2 + 1) = -U;
}
