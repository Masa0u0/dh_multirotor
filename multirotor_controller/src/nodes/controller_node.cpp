#include <kdl_parser/kdl_parser.hpp>
#include <mav_msgs/Actuators.h>

#include <dh_std_tools/iostream.hpp>
#include <dh_eigen_tools/core.hpp>
#include <dh_ros_tools/rate.hpp>
#include <dh_ros_tools/stopwatch.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_linear_control/mpc/linear_dense.hpp>
#include <dh_linear_control/c2d/rk4.hpp>
#include <dh_linear_control/util.hpp>

#include <multirotor_msgs/Command.h>
#include <multirotor_msgs/ControllerFeedback.h>

#include "../../include/multirotor_controller/position_controller.hpp"
#include "../../include/multirotor_controller/acceleration_controller.hpp"
#include "../../include/multirotor_controller/rotation_controller.hpp"
#include "../../include/multirotor_controller/utils.hpp"

using namespace std;
using namespace KDL;

/**
 * @brief 位置制御器(PD制御)，加速度制御器(解析計算)，姿勢制御器(MPC)を組み合わせた制御器．
 * x, y, z, yawの目標値に追従する．
 */
class Controller
{
public:
  explicit Controller(ros::NodeHandle& nh);

  void iterate();

private:
  Tree tree_;

  const int num_rotors_;
  const vector<RotorProperty> rotor_props_;

  dh_kdl_msgs::PoseVel bs_;
  multirotor_msgs::Command cmd_;
  bool cmd_subscribed_;

  PositionController pos_controller_;
  AccelerationController acc_controller_;
  RotationController rot_controller_;
  multirotor_msgs::ControllerFeedback feedback_;
  mav_msgs::Actuators rotor_vels_;
  dh_ros::Stopwatch stopwatch_;

  // PubSub
  ros::Publisher rotor_vels_pub_;
  ros::Publisher feedback_pub_;
  ros::Subscriber bs_sub_;
  ros::Subscriber cmd_sub_;

  void rotorVelsFromCtrlInput(const vector<double>& u, mav_msgs::Actuators& rotor_vels);

  void bsCb(const dh_kdl_msgs::PoseVel& msg);
  void commandCb(const multirotor_msgs::Command& msg);
};

Controller::Controller(ros::NodeHandle& nh)
  : num_rotors_(dh_ros::getParam<int>("/num_rotors")),
    rotor_props_(getRotorProperties()),
    acc_controller_(tree_),
    rot_controller_(tree_)
{
  // URDFを取得
  string drone_name = dh_ros::getParam<string>("/drone_name");
  string description = dh_ros::getParam<string>("/" + drone_name + "/robot_description");

  // Treeを取得
  bool ok = kdl_parser::treeFromString(description, tree_);
  ROS_ASSERT(ok);
  acc_controller_.updateInternalDataStructures();
  rot_controller_.updateInternalDataStructures();

  feedback_.thrust_forces.resize(num_rotors_);
  rotor_vels_.angular_velocities.resize(num_rotors_);

  const string ns = ros::this_node::getNamespace();

  // PubSub
  rotor_vels_pub_ =
    nh.advertise<mav_msgs::Actuators>("/" + drone_name + "/command/motor_speed", 1, false);
  feedback_pub_ = nh.advertise<multirotor_msgs::ControllerFeedback>(ns + "/feedback", 1, false);
  bs_sub_ = nh.subscribe(ns + "/base_state", 1, &Controller::bsCb, this);
  cmd_sub_ = nh.subscribe(ns + "/command", 1, &Controller::commandCb, this);
}

void Controller::iterate()
{
  auto& acc_des = feedback_.desired_acceleration;
  auto& U = feedback_.thrust_force_sum;
  auto& roll_des = feedback_.desired_roll;
  auto& pitch_des = feedback_.desired_pitch;
  auto& yaw_des = feedback_.desired_yaw;
  auto& u = feedback_.thrust_forces;

  yaw_des = cmd_.target_yaw_angle;  // ヨー角はコマンドをそのまま目標値にする

  // stopwatch_.start();
  pos_controller_.update(
    bs_.pose.pos, cmd_.target_position, bs_.twist.vel, Vector::Zero(), acc_des);
  acc_controller_.update(acc_des, yaw_des, U, roll_des, pitch_des);
  rot_controller_.update(bs_, U, roll_des, pitch_des, yaw_des, u);
  // stopwatch_.stop();

  rotorVelsFromCtrlInput(u, rotor_vels_);

  rotor_vels_pub_.publish(rotor_vels_);
  feedback_pub_.publish(feedback_);
}

void Controller::rotorVelsFromCtrlInput(const vector<double>& u, mav_msgs::Actuators& rotor_vels)
{
  ROS_ASSERT(u.size() == num_rotors_);

  for (int i = 0; i < num_rotors_; ++i)
  {
    if (u[i] < -1e-3)
    {
      ROS_FATAL("The thrust force cannot be negative.");
      // TODO: 防御モードに移行
    }
    rotor_vels.angular_velocities[i] = sqrt(max(u[i], 0.) / rotor_props_[i].motor_constant);
  }
}

void Controller::bsCb(const dh_kdl_msgs::PoseVel& msg)
{
  bs_ = msg;

  // トピックが揃っていたら，状態を観測するたびに一回だけ制御器を回す．
  if (cmd_subscribed_)
  {
    iterate();
  }
}

void Controller::commandCb(const multirotor_msgs::Command& msg)
{
  cmd_ = msg;
  cmd_subscribed_ = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;
  Controller node(nh);
  ros::spin();
}
