#pragma once

#include <Eigen/Core>

#include <dh_linear_control/state_spaces.hpp>
#include <dh_kdl/treekdlmodel.hpp>

#include "./const.hpp"
#include "./utils.hpp"

/**
 * @brief クアッドロータの連続時間状態方程式．
 */
class MultiRotorDynamics : public ctrl::LinearDynamics
{
public:
  /**
   * @brief Construct a new MultiRotorDynamics object
   *
   * @param tree 全身のTree
   */
  explicit MultiRotorDynamics(const KDL::Tree& tree);

  /**
   * @brief treeの更新に伴い，内部状態を更新する．
   */
  void updateInternalDataStructures();

  /**
   * @brief 状態方程式を更新する．
   *
   * @param roll {world}に対する{base}のロール角
   * @param pitch {world}に対する{base}のピッチ角
   * @param q アームの関節角
   */
  void update(const double& roll, const double& pitch, const KDL::JntArray& q);

private:
  KDL::TreeKDLModel kdl_model_;

  const Eigen::Vector3d ez_;
  const int num_rotors_;
  const std::vector<RotorProperty> rotor_props_;

  KDL::Rotation rpyvel_angvel_kdl_;
  Eigen::Matrix3d rpyvel_angvel_eigen_;
  bool invertible_;
  KDL::Vector P_base_cog_;
  KDL::Vector P_cog_rotor_kdl_;
  Eigen::Vector3d P_cog_rotor_eigen_;
  KDL::Frame T_base_rotor_;
  KDL::RotationalInertia I_cog_kdl_;  // CoG周りの回転慣性テンソル
  Eigen::Matrix3d I_cog_eigen_;       // CoG周りの回転慣性テンソル
  Eigen::Matrix3d I_cog_inv_;

  void updateA(const double& roll, const double& pitch);
  void updateB(const KDL::JntArray& q);
};
