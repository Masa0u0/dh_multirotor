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
  void update(double roll, double pitch);

private:
  KDL::TreeKDLModel kdl_model_;

  const int num_rotors_;
  const std::vector<RotorProperty> rotor_props_;

  // bool invertible_;
  // KDL::Vector cog_;
  // KDL::RotationalInertia rot_inertia_kdl_;  // CoG周りの回転慣性テンソル
  // Eigen::Matrix3d rot_inertia_eigen_;
  // Eigen::Matrix3d rot_inertia_inv_;
  // Eigen::Matrix3d R_world_base_eigen_;
  KDL::Rotation rpyvel_angvel_kdl_;
  Eigen::Matrix3d rpyvel_angvel_eigen_;

  void setB();
};
