#include <kdl_conversions/kdl_msg.h>

#include <dh_ros_tools/rosparam.hpp>
#include <dh_kdl/util.hpp>

#include "../../include/multirotor_gazebo/state_estimator_gt.hpp"

using namespace std;
using namespace KDL;

StateEstimatorGT::StateEstimatorGT(ros::NodeHandle& nh)
{
  const string drone_name = dh_ros::getParam<string>("/drone_name");
  const string ns = ros::this_node::getNamespace();

  bs_pub_ = nh.advertise<dh_kdl_msgs::PoseVel>("/" + drone_name + "/base_state", 1, false);
  odom_sub_ =
    nh.subscribe("/" + drone_name + "/ground_truth/odometry", 1, &StateEstimatorGT::odomCb, this);
}

void StateEstimatorGT::updatePosition(const geometry_msgs::Point& pos)
{
  tf::pointMsgToKDL(pos, bs_.pose.pos);
}

void StateEstimatorGT::updateRotation(const geometry_msgs::Quaternion& quat)
{
  eulerFromQuaternion(quat.x, quat.y, quat.z, quat.w, rpy_now_(0), rpy_now_(1), rpy_now_(2));

  // ジャンプを考慮して更新
  updateJumpCounts();
  bs_.pose.rpy = (2 * PI) * jump_counts_ + rpy_now_;

  rpy_prev_ = rpy_now_;
}

void StateEstimatorGT::updateLinearVelocity(
  const geometry_msgs::Quaternion& quat,
  const geometry_msgs::Vector3& local_linvel)
{
  auto R_world_base = Rotation::Quaternion(quat.x, quat.y, quat.z, quat.w);
  tf::vectorMsgToKDL(local_linvel, bs_.twist.vel);
  bs_.twist.vel = R_world_base * bs_.twist.vel;  // {local} -> {world}
}

void StateEstimatorGT::updateAngularVelocity(const geometry_msgs::Vector3& local_angvel)
{
  tf::vectorMsgToKDL(local_angvel, bs_.twist.rot);  // Odometryのtwistは元からローカル！
}

void StateEstimatorGT::updateJumpCounts()
{
  for (int i = 0; i < 3; ++i)
  {
    const auto& now = rpy_now_[i];
    const auto& prev = rpy_prev_[i];

    // 前回の観測値から180deg以上ずれていたらジャンプしたとみなす
    if (now - prev > PI)
    {
      jump_counts_[i]--;
    }
    else if (now - prev < -PI)
    {
      jump_counts_[i]++;
    }
  }
}

void StateEstimatorGT::odomCb(const nav_msgs::Odometry& msg)
{
  updatePosition(msg.pose.pose.position);     // 位置(world座標系)
  updateRotation(msg.pose.pose.orientation);  // 姿勢(world座標系)
  updateLinearVelocity(msg.pose.pose.orientation, msg.twist.twist.linear);  // 並進速度(world座標系)
  updateAngularVelocity(msg.twist.twist.angular);  // 回転速度(local座標系！！！)

  bs_pub_.publish(bs_);
}
