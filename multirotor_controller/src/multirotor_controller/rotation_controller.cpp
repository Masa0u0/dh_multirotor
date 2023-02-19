#include <dh_std_tools/vector.hpp>
#include <dh_eigen_tools/core.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_linear_control/util.hpp>

#include "../../include/multirotor_controller/rotation_controller.hpp"

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
    s_(X_DIM)
{
  getParams();

  dt_ = pred_horizon_ / Hp_;
  discs_.resize(Hp_, ctrl::LinearDynamics(X_DIM, num_rotors_));

  // LMPCが可変制約に対応していないため，とりあえず毎周期新しいMPCを作成するためのオブジェクト
  Cz_ = makeCz();
  E_e_ = ctrl::LinearEquation(num_rotors_, 0);
  F_f_ = makeBaseInputCondition();
  G_g_ = ctrl::LinearEquation(X_DIM, 0);
}

void RotationController::updateInternalDataStructures()
{
  kdl_model_.updateInternalDataStructures();
  cont_.updateInternalDataStructures();

  R_ = makeWeight_R();
  S_ = makeWeight_S();
  Q_ = makeWeight_Q();
}

void RotationController::update(
  const dh_kdl_msgs::PoseVel& bs,
  double U,
  double roll_des,
  double pitch_des,
  double yaw_des,
  vector<double>& u_opt)
{
  ROS_ASSERT(U > 0.);
  ROS_ASSERT(u_opt.size() == num_rotors_);

  const auto& rpy = bs.pose.rpy;
  updateDynamics(rpy.x(), rpy.y(), roll_des, pitch_des);
  updateX(bs);
  updateS(roll_des, pitch_des, yaw_des);

  updateInputCondition(U);

  // stopwatch_.start();
  ctrl::LinearDenseMPC mpc(discs_, Cz_, Hp_, Hu_, dt_, T_refs_, R_, S_, Q_, E_e_, F_f_, G_g_);
  u_opt = eigen_tools::toStdVector(mpc.step(x_, s_));
  // stopwatch_.stop();
}

void RotationController::getParams()
{
  pred_horizon_ = dh_ros::getParam<double>("~rotation_controller/prediction_horizon");
  Hp_ = dh_ros::getParam<int>("~rotation_controller/prediction_steps");
  Hu_ = dh_ros::getParam<int>("~rotation_controller/input_steps");
  weight_scale_ = dh_ros::getParam<double>("~rotation_controller/weight_scale");
  T_refs_[ROLL] = dh_ros::getParam<double>("~rotation_controller/decay/rotation/roll");
  T_refs_[PITCH] = dh_ros::getParam<double>("~rotation_controller/decay/rotation/pitch");
  T_refs_[YAW] = dh_ros::getParam<double>("~rotation_controller/decay/rotation/yaw");
  T_refs_[ANGVEL_X] = dh_ros::getParam<double>("~rotation_controller/decay/angular_velocity/x");
  T_refs_[ANGVEL_X] = dh_ros::getParam<double>("~rotation_controller/decay/angular_velocity/y");
  T_refs_[ANGVEL_X] = dh_ros::getParam<double>("~rotation_controller/decay/angular_velocity/z");
  Q_values_[ROLL] = dh_ros::getParam<double>("~rotation_controller/state_weight/rotation/roll");
  Q_values_[PITCH] = dh_ros::getParam<double>("~rotation_controller/state_weight/rotation/pitch");
  Q_values_[YAW] = dh_ros::getParam<double>("~rotation_controller/state_weight/rotation/yaw");
  Q_values_[ANGVEL_X] = dh_ros::getParam<double>("~rotation_controller/state_weight/"
                                                 "angular_velocity/x");
  Q_values_[ANGVEL_X] = dh_ros::getParam<double>("~rotation_controller/state_weight/"
                                                 "angular_velocity/y");
  Q_values_[ANGVEL_X] = dh_ros::getParam<double>("~rotation_controller/state_weight/"
                                                 "angular_velocity/z");
  R_value_ = dh_ros::getParam<double>("~rotation_controller/input_rate_weight");
  S_value_ = dh_ros::getParam<double>("~rotation_controller/input_weight");

  ROS_ASSERT(pred_horizon_ > 0.);
  ROS_ASSERT(0 < Hu_ && Hu_ <= Hp_);
  ROS_ASSERT(weight_scale_ > 0.);
  ROS_ASSERT(dh_std::all_ge(Q_values_, 0.));
  ROS_ASSERT(R_value_ >= 0.);
  ROS_ASSERT(S_value_ > 0.);
}

void RotationController::updateDynamics(
  double roll,
  double pitch,
  double roll_des,
  double pitch_des)
{
  double t;
  double roll_k, pitch_k;

  for (int k = 0; k < Hp_; ++k)
  {
    t = dt_ * k;  // 計画開始時刻(= 0)からの経過時間

    // 時刻tにおけるドローンの姿勢の参照値
    roll_k = ctrl::firstOrderPos(roll, roll_des, T_refs_[ROLL], t);
    pitch_k = ctrl::firstOrderPos(pitch, pitch_des, T_refs_[PITCH], t);

    cont_.update(roll_k, pitch_k);
    discs_[k] = c2d_.convert(cont_, dt_);
  }
}

void RotationController::updateX(const dh_kdl_msgs::PoseVel& bs)
{
  const auto& r = bs.pose.rpy;
  const auto& w = bs.twist.rot;
  x_ << r.x(), r.y(), r.z(), w.x(), w.y(), w.z();
}

void RotationController::updateS(double roll_des, double pitch_des, double yaw_des)
{
  s_ << roll_des, pitch_des, yaw_des, 0., 0., 0.;
}

MatrixXd RotationController::makeCz()
{
  return MatrixXd::Identity(X_DIM, X_DIM);
}

VectorXd RotationController::makeWeight(const vector<double>& values, const vector<double>& scales)
{
  ROS_ASSERT(values.size() == scales.size());
  ROS_ASSERT(dh_std::all_ge(values, 0.));
  ROS_ASSERT(dh_std::all_gt(scales, 0.));

  int dim = values.size();

  // 重みをスケーリング
  VectorXd scaled_values(dim);
  for (int i = 0; i < dim; ++i)
  {
    scaled_values(i) = values[i] / (weight_scale_ * pow(scales[i], 2.));
  }

  return scaled_values;
}

VectorXd RotationController::makeWeight_R()
{
  double mass = kdl_model_.treeMass();
  double u_scale = mass * GRAVITY;
  double delta_u_scale = u_scale * dt_;
  vector<double> delta_u_scales(num_rotors_, delta_u_scale);
  vector<double> R_values(num_rotors_, R_value_);
  return makeWeight(R_values, delta_u_scales);
}

VectorXd RotationController::makeWeight_S()
{
  double mass = kdl_model_.treeMass();
  double u_scale = mass * GRAVITY;
  vector<double> u_scales(num_rotors_, u_scale);
  vector<double> S_values(num_rotors_, S_value_);
  return makeWeight(S_values, u_scales);
}

VectorXd RotationController::makeWeight_Q()
{
  double scale_euler = PI;
  double scale_angvel = PI;
  vector<double> z_scales{ scale_euler,  scale_euler,  scale_euler,
                           scale_angvel, scale_angvel, scale_angvel };
  return makeWeight(Q_values_, z_scales);
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

void RotationController::updateInputCondition(double U)
{
  F_f_.b(num_rotors_ * 2) = U;
  F_f_.b(num_rotors_ * 2 + 1) = -U;
}
