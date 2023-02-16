#include <ros/ros.h>

#include "../../include/multirotor_controller/acceleration_controller.hpp"
#include "../../include/multirotor_controller/const.hpp"

using namespace std;
using namespace KDL;

AccelerationController::AccelerationController(const Tree& tree) : inertia_solver_(tree)
{
}

void AccelerationController::updateInternalDataStructures()
{
  inertia_solver_.updateInternalDataStructures();
  mass_ = inertia_solver_.JntToMass();
}

void AccelerationController::update(
  const KDL::Vector& acc_des,
  double yaw_des,
  double& U_out,
  double& roll_out,
  double& pitch_out)
{
  double x = mass_ * acc_des.x();
  double y = mass_ * acc_des.y();
  double z = mass_ * (acc_des.z() + GRAVITY);
  ROS_ASSERT(z > 0.);

  double cos_yaw = cos(yaw_des);
  double sin_yaw = sin(yaw_des);

  pitch_out = atan2(x * cos_yaw + y * sin_yaw, z);
  roll_out = atan2(cos(pitch_out) * (x * sin_yaw - y * cos_yaw), z);
  U_out = z / (cos(pitch_out) * cos(roll_out));
  ROS_ASSERT(U_out > 0.);
}
