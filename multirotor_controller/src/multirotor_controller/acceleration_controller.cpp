#include <ros/ros.h>

#include <dh_std_tools/algorithm.hpp>

#include "../../include/multirotor_controller/acceleration_controller.hpp"
#include "../../include/multirotor_controller/const.hpp"

using namespace std;
using namespace KDL;

AccelerationController::AccelerationController(const Tree& tree)
  : rotor_props_(getRotorProperties()), inertia_solver_(tree)
{
  max_U_ = 0.;
  for (const auto& prop : rotor_props_)
  {
    const double max_thrust = prop.motor_constant * sqr(prop.max_velocity);
    max_U_ += max_thrust;
  }
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
  const double x = mass_ * acc_des.x();
  const double y = mass_ * acc_des.y();
  const double z = mass_ * (acc_des.z() + GRAVITY);

  const double cos_yaw = cos(yaw_des);
  const double sin_yaw = sin(yaw_des);

  pitch_out = atan2(x * cos_yaw + y * sin_yaw, z);
  roll_out = atan2(cos(pitch_out) * (x * sin_yaw - y * cos_yaw), z);
  U_out = dh_std::clamp(z / (cos(pitch_out) * cos(roll_out)), 0., max_U_);
}
