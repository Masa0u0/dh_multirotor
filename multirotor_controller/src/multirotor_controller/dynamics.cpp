#include <ros/ros.h>
#include <Eigen/Core>
#include <eigen_conversions/eigen_kdl.h>

#include <dh_ros_tools/rosparam.hpp>
#include <dh_kdl/util.hpp>
#include <dh_kdl/conversion/kdl_eigen.hpp>

#include "../../include/multirotor_controller/dynamics.hpp"
#include "../../include/multirotor_controller/const.hpp"
#include "../../include/multirotor_controller/utils.hpp"

using namespace std;
using namespace Eigen;
using namespace KDL;

MultiRotorDynamics::MultiRotorDynamics(const Tree& tree)
  : kdl_model_(tree),
    ez_(0., 0., 1.),
    num_rotors_(dh_ros::getParam<int>("/num_rotors")),
    rotor_props_(getRotorProperties())
{
  resize(X_DIM, num_rotors_);
}

void MultiRotorDynamics::updateInternalDataStructures()
{
  kdl_model_.updateInternalDataStructures();
}

void MultiRotorDynamics::update(const double& roll, const double& pitch, const JntArray& q)
{
  updateA(roll, pitch);
  updateB(q);
}

void MultiRotorDynamics::updateA(const double& roll, const double& pitch)
{
  eulerrateFromAngvelLocal(roll, pitch, rpyvel_angvel_kdl_);
  tf::rotationKDLToEigen(rpyvel_angvel_kdl_, rpyvel_angvel_eigen_);
  A.block(0, 3, 3, 3) = rpyvel_angvel_eigen_;
}

void MultiRotorDynamics::updateB(const JntArray& q)
{
  kdl_model_.treeInertia(q, P_base_cog_, I_cog_kdl_);
  tf::rotInertiaKDLToEigen(I_cog_kdl_, I_cog_eigen_);
  I_cog_eigen_.computeInverseWithCheck(I_cog_inv_, invertible_);
  ROS_ASSERT(invertible_);

  for (int i = 0; i < num_rotors_; ++i)
  {
    kdl_model_.fkPos(q, rotor_props_[i].link_name, T_base_rotor_);
    P_cog_rotor_kdl_ = T_base_rotor_.p - P_base_cog_;
    tf::vectorKDLToEigen(P_cog_rotor_kdl_, P_cog_rotor_eigen_);
    const auto& d = rotor_props_[i].direction;
    const auto& c = rotor_props_[i].moment_constant;
    B.block(3, i, 3, 1) = I_cog_inv_ * (P_cog_rotor_eigen_.cross(ez_) - (d * c) * ez_);
  }
}
