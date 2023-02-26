#include <dh_std_tools/vector.hpp>
#include <dh_eigen_tools/core.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_linear_control/util.hpp>

#include "../../include/multirotor_controller/rotation_controller.hpp"

#define WEIGHT_SCALE 1e-6

using namespace std;
using namespace Eigen;
using namespace KDL;

RotationController::RotationController(const Tree& tree)
  : num_rotors_(dh_ros::getParam<int>("/num_rotors")),
    rotor_props_(getRotorProperties()),
    T_refs_(X_DIM),
    Q_values_(X_DIM),
    kdl_model_(tree),
    cont_(tree),
    c2d_(X_DIM, num_rotors_),
    x_(X_DIM),
    s_(X_DIM),
    u_(VectorXd::Zero(num_rotors_)),
    Cz_(MatrixXd::Identity(X_DIM, X_DIM)),
    R_(num_rotors_),
    S_(num_rotors_),
    Q_(X_DIM),
    E_e_(ctrl::LinearEquation(num_rotors_, 0)),
    F_f_(makeBaseInputCondition()),
    G_g_(ctrl::LinearEquation(X_DIM, 0))
{
  getParams();

  dt_ = pred_horizon_ / Hp_;
  discs_.resize(Hp_, ctrl::LinearDynamics(X_DIM, num_rotors_));
}

void RotationController::updateInternalDataStructures()
{
  kdl_model_.updateInternalDataStructures();
  cont_.updateInternalDataStructures();

  mass_ = kdl_model_.treeMass();

  updateWeight_R();
  updateWeight_S();
  updateWeight_Q();
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
  ROS_ASSERT(U > 0.);
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
  int input_weight,
  int input_rate_weight)
{
  ROS_ASSERT(pred_horizon > 0.);
  ROS_ASSERT(pred_steps > 0);
  ROS_ASSERT(rot_decay >= 0.);
  ROS_ASSERT(angvel_decay >= 0.);
  ROS_ASSERT(rot_weight > 0.);
  ROS_ASSERT(angvel_weight > 0.);

  pred_horizon_ = pred_horizon;
  Hp_ = pred_steps;
  T_refs_[ROLL] = T_refs_[PITCH] = T_refs_[YAW] = rot_decay;
  T_refs_[ANGVEL_X] = T_refs_[ANGVEL_Y] = T_refs_[ANGVEL_Z] = angvel_decay;
  Q_values_[ROLL] = Q_values_[PITCH] = Q_values_[YAW] = rot_decay;
  Q_values_[ANGVEL_X] = Q_values_[ANGVEL_Y] = Q_values_[ANGVEL_Z] = angvel_decay;
  S_value_ = pow(10, input_weight);
  R_value_ = pow(10, input_rate_weight);

  dt_ = pred_horizon / pred_steps;

  updateWeight_R();
  updateWeight_S();
  updateWeight_Q();
}

void RotationController::getParams()
{
  pred_horizon_ = dh_ros::getParam<double>("~prediction_horizon");
  Hp_ = dh_ros::getParam<int>("~prediction_steps");
  T_refs_[ROLL] = dh_ros::getParam<double>("~rotation_decay");
  T_refs_[PITCH] = dh_ros::getParam<double>("~rotation_decay");
  T_refs_[YAW] = dh_ros::getParam<double>("~rotation_decay");
  T_refs_[ANGVEL_X] = dh_ros::getParam<double>("~angular_velocity_decay");
  T_refs_[ANGVEL_Y] = dh_ros::getParam<double>("~angular_velocity_decay");
  T_refs_[ANGVEL_Z] = dh_ros::getParam<double>("~angular_velocity_decay");
  Q_values_[ROLL] = dh_ros::getParam<double>("~rotation_weight");
  Q_values_[PITCH] = dh_ros::getParam<double>("~rotation_weight");
  Q_values_[YAW] = dh_ros::getParam<double>("~rotation_weight");
  Q_values_[ANGVEL_X] = dh_ros::getParam<double>("~angular_velocity_weight");
  Q_values_[ANGVEL_Y] = dh_ros::getParam<double>("~angular_velocity_weight");
  Q_values_[ANGVEL_Z] = dh_ros::getParam<double>("~angular_velocity_weight");
  S_value_ = pow(10, dh_ros::getParam<double>("~thrust_force_weight"));
  R_value_ = pow(10, dh_ros::getParam<double>("~thrust_force_rate_weight"));

  ROS_ASSERT(pred_horizon_ > 0.);
  ROS_ASSERT(Hp_ > 0);
  ROS_ASSERT(dh_std::all_ge(Q_values_, 0.));
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

void RotationController::updateWeight(
  const vector<double>& values,
  const vector<double>& scales,
  VectorXd& weight)
{
  uint32_t dim = weight.size();

  ROS_ASSERT(values.size() == dim);
  ROS_ASSERT(scales.size() == dim);
  ROS_ASSERT(dh_std::all_ge(values, 0.));
  ROS_ASSERT(dh_std::all_gt(scales, 0.));

  // 重みをスケーリング
  for (int i = 0; i < dim; ++i)
  {
    weight(i) = values[i] / (WEIGHT_SCALE * pow(scales[i], 2.));
  }
}

void RotationController::updateWeight_R()
{
  double u_scale = mass_ * GRAVITY;
  double delta_u_scale = u_scale * dt_;
  vector<double> delta_u_scales(num_rotors_, delta_u_scale);
  vector<double> R_values(num_rotors_, R_value_);
  updateWeight(R_values, delta_u_scales, R_);
}

void RotationController::updateWeight_S()
{
  double u_scale = mass_ * GRAVITY;
  vector<double> u_scales(num_rotors_, u_scale);
  vector<double> S_values(num_rotors_, S_value_);
  updateWeight(S_values, u_scales, S_);
}

void RotationController::updateWeight_Q()
{
  double scale_euler = PI;
  double scale_angvel = PI;
  vector<double> z_scales{ scale_euler,  scale_euler,  scale_euler,
                           scale_angvel, scale_angvel, scale_angvel };
  updateWeight(Q_values_, z_scales, Q_);
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
