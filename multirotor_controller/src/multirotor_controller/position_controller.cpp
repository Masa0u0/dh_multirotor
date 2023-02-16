#include <ros/ros.h>

#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>

#include "../../include/multirotor_controller/position_controller.hpp"

PositionController::PositionController()
{
  getParams();

  kp_ = dh_std::sqr(wn_);
  kd_ = 2. * zeta_ * wn_;
}

void PositionController::update(
  const KDL::Vector& pos,
  const KDL::Vector& pos_des,
  const KDL::Vector& vel,
  const KDL::Vector& vel_des,
  KDL::Vector& acc_out)
{
  acc_out = kp_ * (pos_des - pos) + kd_ * (vel_des - vel);
}

void PositionController::getParams()
{
  wn_ = dh_ros::getParam<double>("~position_controller/natural_frequency");
  zeta_ = dh_ros::getParam<double>("~position_controller/damping_ratio");

  ROS_ASSERT(wn_ > 0.);
  ROS_ASSERT(zeta_ > 0.);
}
