#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>

#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_common_msgs/eigen_msg.hpp>
#include <dh_kdl_msgs/PoseVel.h>

#include "../../include/multirotor_eskf/eskf_ros.hpp"

#define GRAVITY 9.80665
#define WARN_PERIOD 3.
#define I_3 (Matrix3d::Identity())

using namespace std;
using namespace Eigen;
using namespace dh_std;

ErrorStateKalmanFilterRos::ErrorStateKalmanFilterRos(ros::NodeHandle& nh)
  : gyro_noise_density_(dh_ros::getParam<double>("~gyro_noise_density")),
    gyro_random_walk_(dh_ros::getParam<double>("~gyro_random_walk")),
    acc_noise_density_(dh_ros::getParam<double>("~acc_noise_density")),
    acc_random_walk_(dh_ros::getParam<double>("~acc_random_walk")),
    ref_mag_(
      dh_ros::getParam<double>("~reference_magnetic_strength/north"),
      dh_ros::getParam<double>("~reference_magnetic_strength/east"),
      dh_ros::getParam<double>("~reference_magnetic_strength/down")),
    is_ready_(false),
    imu_subscribed_(false),
    mag_subscribed_(false),
    bar_subscribed_(false),
    gps_subscribed_(false),
    vel_subscribed_(false)
{
  posevel_pub_ = nh.advertise<dh_kdl_msgs::PoseVel>("/estimated_state", 1);

  imu_sub_ = nh.subscribe("/imu", 1, &ErrorStateKalmanFilterRos::imuCb, this);
  mag_sub_ = nh.subscribe("/magnetic_field", 1, &ErrorStateKalmanFilterRos::magCb, this);
  bar_sub_ = nh.subscribe("/air_pressure", 1, &ErrorStateKalmanFilterRos::barCb, this);
  gps_sub_ = nh.subscribe("/gps", 1, &ErrorStateKalmanFilterRos::gpsCb, this);
  vel_sub_ = nh.subscribe("/ground_speed", 1, &ErrorStateKalmanFilterRos::velCb, this);
}

bool ErrorStateKalmanFilterRos::allSubscribed()
{
  bool ok = true;

  if (!imu_subscribed_)
  {
    ROS_WARN_THROTTLE(WARN_PERIOD, "IMU data is not subscribed yet.");
    ok = false;
  }

  if (!mag_subscribed_)
  {
    ROS_WARN_THROTTLE(WARN_PERIOD, "Magnetometer data is not subscribed yet.");
    ok = false;
  }

  if (!bar_subscribed_)
  {
    ROS_WARN_THROTTLE(WARN_PERIOD, "Barometer data is not subscribed yet.");
    ok = false;
  }

  if (!gps_subscribed_)
  {
    ROS_WARN_THROTTLE(WARN_PERIOD, "GPS position data is not subscribed yet.");
    ok = false;
  }

  if (!vel_subscribed_)
  {
    ROS_WARN_THROTTLE(WARN_PERIOD, "GPS velocity data is not subscribed yet.");
    ok = false;
  }

  return ok;
}

void ErrorStateKalmanFilterRos::initialize()
{
  gpsToCartesian(gps_.latitude, gps_.longitude, gps_.altitude, p_m_.x(), p_m_.y(), p_m_.z());

  // tf::linVelMsgToEigen(vel_.vel.vel, v_m_);
  v_m_.setZero();

  const auto& a = imu_.linear_acceleration;
  const auto& m = mag_.magnetic_field;
  imuToQuaternion(
    a.x, a.y, a.z, m.x, m.y, m.z, ref_mag_.x(), ref_mag_.y(), ref_mag_.z(), q_m_.x(), q_m_.y(),
    q_m_.z(), q_m_.w());

  eskf_.initialize(
    Vector3d(0, 0, -GRAVITY),  // Acceleration due to gravity in global frame
    ErrorStateKalmanFilter::makeState(
      p_m_,              // init pos
      v_m_,              // init vel
      q_m_,              // init quaternion
      Vector3d::Zero(),  // init accel bias
      Vector3d::Zero()   // init gyro bias
      ),
    ErrorStateKalmanFilter::makeP(
      sqr(1.) * I_3, sqr(0.1) * I_3, sqr(1.) * I_3, sqr(10 * 0.001 * 0.00124) * I_3,
      sqr(10 * 0.001 * 0.276) * I_3),
    sqr(0.00124), sqr(0.276), sqr(0.001 * 0.00124), sqr(0.001 * 0.276),  // TODO
    ErrorStateKalmanFilter::delayTypes::applyUpdateToNew, 100);  // TODO: 他のも試してみる

  t_last_ = ros::Time::now();
}

void ErrorStateKalmanFilterRos::updatePoseVelMsg()
{
  tf::vectorEigenToKDL(eskf_.getPos(), posevel_.pose.pos);

  auto q = eskf_.getQuat();
  auto& rpy = posevel_.pose.rpy;
  quaternionToEuler(q.x(), q.y(), q.z(), q.w(), rpy(0), rpy(1), rpy(2));

  tf::vectorEigenToKDL(eskf_.getVel(), posevel_.twist.vel);

  Vector3d w = w_m_ - eskf_.getGyroBias();
  tf::vectorEigenToKDL(w, posevel_.twist.rot);
}

void ErrorStateKalmanFilterRos::imuCb(const ImuMsg& msg)
{
  imu_subscribed_ = true;
  imu_ = msg;

  if (!is_ready_)
  {
    if (allSubscribed())
    {
      initialize();
      is_ready_ = true;
      ROS_INFO_STREAM(ros::this_node::getName() << " is ready.");
    }
    return;
  }

  ros::Duration diff = imu_.header.stamp - t_last_;
  t_last_ = imu_.header.stamp;
  ROS_ASSERT(diff.toSec() > 0.);
  lTime stamp(imu_.header.stamp.sec, imu_.header.stamp.nsec);

  tf::vectorMsgToEigen(imu_.linear_acceleration, a_m_);
  tf::vectorMsgToEigen(imu_.angular_velocity, w_m_);

  eskf_.predictIMU(a_m_, w_m_, diff.toSec(), stamp);
  // eskf_.predictIMU(Vector3d(0, 0, GRAVITY), Vector3d::Zero(), diff.toSec(), stamp);

  updatePoseVelMsg();
  posevel_pub_.publish(posevel_);
}

void ErrorStateKalmanFilterRos::magCb(const MagMsg& msg)
{
  mag_subscribed_ = true;
  mag_ = msg;

  lTime stamp(mag_.header.stamp.sec, mag_.header.stamp.nsec);
  lTime now = getNow();

  // FIXME: 観測が状態に依存してるのはマズい気がする
  const Vector3d a = a_m_ - eskf_.getAccelBias();
  const geometry_msgs::Vector3& m = mag_.magnetic_field;
  imuToQuaternion(
    a.x(), a.y(), a.z(), m.x, m.y, m.z, ref_mag_.x(), ref_mag_.y(), ref_mag_.z(), q_m_.x(),
    q_m_.y(), q_m_.z(), q_m_.w());

  // TODO: 加速度センサのノイズの分散からクォータニオンのノイズの共分散を正しく計算する
  // sensor_msgs::Imuのlinear_acceleration_covarianceを用いる
  const double acc_noise_var = sqr(acc_noise_density_) * 1000.;
  const double quat_var = acc_noise_var / sqr(GRAVITY);  // これはテキトーにスケーリングしてるだけ

  eskf_.measureQuat(q_m_, quat_var * I_3, stamp, now);
  // eskf_.measureQuat(Quaterniond::Identity(), quat_var * I_3, stamp, now);
}

void ErrorStateKalmanFilterRos::barCb(const BarMsg& msg)
{
  bar_subscribed_ = true;
  bar_ = msg;

  // TODO
}

void ErrorStateKalmanFilterRos::gpsCb(const GpsMsg& msg)
{
  gps_subscribed_ = true;
  gps_ = msg;

  lTime stamp(gps_.header.stamp.sec, gps_.header.stamp.nsec);
  lTime now = getNow();

  // TODO: 基準点を考慮した変換手法に修正
  gpsToCartesian(gps_.latitude, gps_.longitude, gps_.altitude, p_m_.x(), p_m_.y(), p_m_.z());
  // cout << p_m_ << endl;
  Matrix3d cov = Map<Matrix3d>(gps_.position_covariance.data());

  eskf_.measurePos(p_m_, cov, stamp, now);
  // eskf_.measurePos(Vector3d::Zero(), cov, stamp, now);
}

void ErrorStateKalmanFilterRos::velCb(const VelMsg& msg)
{
  vel_subscribed_ = true;
  vel_ = msg;

  lTime stamp(vel_.header.stamp.sec, vel_.header.stamp.nsec);
  lTime now = getNow();

  tf::linVelMsgToEigen(vel_.vel.vel, v_m_);
  Matrix3d cov = Map<Matrix3d>(vel_.vel.covariance.data());

  eskf_.measureVel(v_m_, cov, stamp, now);
  // eskf_.measureVel(Vector3d::Zero(), cov, stamp, now);
}

lTime ErrorStateKalmanFilterRos::getNow()
{
  ros::Time now = ros::Time::now();
  return lTime(now.sec, now.nsec);
}
