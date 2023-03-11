#pragma once

#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <dh_kdl_msgs/PoseVel.h>

/**
 * @brief {world}から見た{base}の状態(位置，姿勢とその時間微分)を発行する．
 * Odometryの真の値をPoseVelに変換するだけ．
 */
class StateEstimatorGT
{
public:
  explicit StateEstimatorGT(ros::NodeHandle& nh);

private:
  dh_kdl_msgs::PoseVel bs_;
  KDL::Vector rpy_now_;
  KDL::Vector rpy_prev_;
  KDL::Vector jump_counts_;  // オイラー角の回転回数

  ros::Publisher bs_pub_;
  ros::Subscriber odom_sub_;

  void updatePosition(const geometry_msgs::Point& pos);
  void updateRotation(const geometry_msgs::Quaternion& quat);
  void updateLinearVelocity(
    const geometry_msgs::Quaternion& quat,
    const geometry_msgs::Vector3& local_linvel);
  void updateAngularVelocity(const geometry_msgs::Vector3& local_angvel);
  void updateJumpCounts();

  void odomCb(const nav_msgs::Odometry& msg);
};
