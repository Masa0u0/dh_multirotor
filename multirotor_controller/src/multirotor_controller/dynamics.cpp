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
    num_rotors_(dh_ros::getParam<int>("/num_rotors")),
    rotor_props_(getRotorProperties())
{
  resize(X_DIM, num_rotors_);
}

void MultiRotorDynamics::updateInternalDataStructures()
{
  kdl_model_.updateInternalDataStructures();
  setB();
}

void MultiRotorDynamics::update(double roll, double pitch)
{
  eulerrateFromAngvelLocal(roll, pitch, rpyvel_angvel_kdl_);
  tf::rotationKDLToEigen(rpyvel_angvel_kdl_, rpyvel_angvel_eigen_);
  A.block(0, 3, 3, 3) = rpyvel_angvel_eigen_;
}

void MultiRotorDynamics::setB()
{
  const Vector3d ez(0., 0., 1.);
  const JntArray dummy_q(kdl_model_.getNrOfJoints());

  Vector P_base_cog;
  RotationalInertia I_cog_kdl;
  Eigen::Matrix3d I_cog_eigen;
  Eigen::Matrix3d I_cog_inv;
  bool invertible;
  kdl_model_.treeInertia(dummy_q, P_base_cog, I_cog_kdl);
  tf::rotInertiaKDLToEigen(I_cog_kdl, I_cog_eigen);
  I_cog_eigen.computeInverseWithCheck(I_cog_inv, invertible);
  ROS_ASSERT(invertible);

  Frame T_base_rotor;
  Vector P_cog_rotor_kdl;
  Vector3d P_cog_rotor_eigen;
  Vector3d v;
  for (int i = 0; i < num_rotors_; ++i)
  {
    kdl_model_.fkPos(dummy_q, rotor_props_[i].link_name, T_base_rotor);
    P_cog_rotor_kdl = T_base_rotor.p - P_base_cog;
    tf::vectorKDLToEigen(P_cog_rotor_kdl, P_cog_rotor_eigen);
    B.block(3, i, 3, 1) =
      I_cog_inv
      * (P_cog_rotor_eigen.cross(ez) - rotor_props_[i].direction * rotor_props_[i].moment_constant * ez);
  }
}
