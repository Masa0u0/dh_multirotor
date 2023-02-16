#pragma once

#include <kdl/tree.hpp>
#include <kdl/frames.hpp>

#include <dh_kdl/treejnttoinertiasolver.hpp>

class AccelerationController
{
public:
  AccelerationController(const KDL::Tree& tree);

  void updateInternalDataStructures();

  void update(
    const KDL::Vector& acc_des,
    double yaw_des,
    double& U_out,
    double& roll_out,
    double& pitch_out);

private:
  KDL::TreeJntToInertiaSolver inertia_solver_;
  double mass_;
};
