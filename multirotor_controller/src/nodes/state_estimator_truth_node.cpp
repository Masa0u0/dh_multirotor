#include <ros/ros.h>
#include <kdl_conversions/kdl_msg.h>
#include <nav_msgs/Odometry.h>

#include <dh_ros_tools/rosparam.hpp>
#include <dh_kdl/util.hpp>
#include <dh_kdl_msgs/PoseVel.h>

#include "../../include/multirotor_controller/const.hpp"

using namespace std;
using namespace KDL;

/**
 * @brief {world}から見た{base}の状態(位置，姿勢とその時間微分)を発行する．
 * Gazeboから取得した真の状態を用いる．
 */
class StateEstimator
{
public:
  explicit StateEstimator(ros::NodeHandle& nh);

private:
  dh_kdl_msgs::PoseVel bs_;
  Vector rpy_now_;
  Vector rpy_prev_;
  Vector jump_counts_;  // オイラー角の回転回数

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

StateEstimator::StateEstimator(ros::NodeHandle& nh)
{
  string drone_name = dh_ros::getParam<string>("/drone_name");

  bs_pub_ = nh.advertise<dh_kdl_msgs::PoseVel>(ros::this_node::getNamespace() + "/base_state", 1, false);
  odom_sub_ =
    nh.subscribe("/" + drone_name + "/ground_truth/odometry", 1, &StateEstimator::odomCb, this);
}

void StateEstimator::updatePosition(const geometry_msgs::Point& pos)
{
  tf::pointMsgToKDL(pos, bs_.pose.pos);
}

void StateEstimator::updateRotation(const geometry_msgs::Quaternion& quat)
{
  eulerFromQuaternion(quat.x, quat.y, quat.z, quat.w, rpy_now_(0), rpy_now_(1), rpy_now_(2));

  // ジャンプを考慮して更新
  updateJumpCounts();
  bs_.pose.rpy = (2 * PI) * jump_counts_ + rpy_now_;

  rpy_prev_ = rpy_now_;
}

void StateEstimator::updateLinearVelocity(
  const geometry_msgs::Quaternion& quat,
  const geometry_msgs::Vector3& local_linvel)
{
  auto R_world_base = Rotation::Quaternion(quat.x, quat.y, quat.z, quat.w);
  tf::vectorMsgToKDL(local_linvel, bs_.twist.vel);
  bs_.twist.vel = R_world_base * bs_.twist.vel;  // {base} -> {world}
}

void StateEstimator::updateAngularVelocity(const geometry_msgs::Vector3& local_angvel)
{
  tf::vectorMsgToKDL(local_angvel, bs_.twist.rot);  // Odometryのtwistは元からローカル！
}

void StateEstimator::updateJumpCounts()
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

void StateEstimator::odomCb(const nav_msgs::Odometry& msg)
{
  updatePosition(msg.pose.pose.position);     // 位置(world座標系)
  updateRotation(msg.pose.pose.orientation);  // 姿勢(world座標系)
  updateLinearVelocity(msg.pose.pose.orientation, msg.twist.twist.linear);  // 並進速度(world座標系)
  updateAngularVelocity(msg.twist.twist.angular);  // 回転速度(local座標系！！！)

  bs_pub_.publish(bs_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_estimator_truth");
  ros::NodeHandle nh;
  StateEstimator node(nh);
  ros::spin();
}
