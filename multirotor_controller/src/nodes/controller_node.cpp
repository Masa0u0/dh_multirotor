#include <kdl_parser/kdl_parser.hpp>
#include <dynamic_reconfigure/server.h>
#include <mav_msgs/Actuators.h>
#include <sensor_msgs/JointState.h>

#include <dh_std_tools/iostream.hpp>
#include <dh_std_tools/vector.hpp>
#include <dh_eigen_tools/core.hpp>
#include <dh_ros_tools/rate.hpp>
#include <dh_ros_tools/stopwatch.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_linear_control/mpc/linear_dense.hpp>
#include <dh_linear_control/c2d/rk4.hpp>
#include <dh_linear_control/util.hpp>

#include <multirotor_msgs/Command.h>
#include <multirotor_msgs/ControllerFeedback.h>
#include <multirotor_controller/ControllerConfig.h>

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
  using ConfigServer = dynamic_reconfigure::Server<multirotor_controller::ControllerConfig>;

public:
  explicit Controller(ros::NodeHandle& nh);

private:
  Tree tree_;
  TreeKDLModel kdl_model_;

  const uint32_t num_rotors_;
  const vector<string> required_joints_;  // プロペラ以外の可動関節の名前のリスト
  const bool transformable_;              // プロペラ以外の可動関節を持つか否か
  const vector<RotorProperty> rotor_props_;

  dh_kdl_msgs::PoseVel bs_;
  JntArray q_;
  multirotor_msgs::Command cmd_;
  bool js_subscribed_;
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
  ros::Subscriber js_sub_;
  ros::Subscriber cmd_sub_;

  // Dynamic Reconfigure
  ConfigServer server_;

  void runOnce();
  void rotorVelsFromCtrlInput(const vector<double>& u, mav_msgs::Actuators& rotor_vels);
  bool allSubscribed();

  void bsCb(const dh_kdl_msgs::PoseVel& msg);
  void jsCb(const sensor_msgs::JointState& msg);
  void commandCb(const multirotor_msgs::Command& msg);

  void dynamicReconfigureCb(const multirotor_controller::ControllerConfig& cfg, uint32_t level);
};

Controller::Controller(ros::NodeHandle& nh)
  : kdl_model_(tree_),
    num_rotors_(dh_ros::getParam<int>("/num_rotors")),
    required_joints_(dh_ros::getParam<vector<string>>("/required_joint_names")),
    transformable_(required_joints_.size() > 0),
    rotor_props_(getRotorProperties()),
    js_subscribed_(false),
    cmd_subscribed_(false),
    acc_controller_(tree_),
    rot_controller_(tree_)
{
  // URDFを取得
  const string drone_name = dh_ros::getParam<string>("/drone_name");
  const string description = dh_ros::getParam<string>("/robot_description");

  // Treeを取得
  const bool ok = kdl_parser::treeFromString(description, tree_);
  ROS_ASSERT(ok);
  kdl_model_.updateInternalDataStructures();
  acc_controller_.updateInternalDataStructures();
  rot_controller_.updateInternalDataStructures();

  q_.resize(kdl_model_.getNrOfJoints());
  feedback_.thrust_forces.resize(num_rotors_);
  rotor_vels_.angular_velocities.resize(num_rotors_);

  const string ns = ros::this_node::getNamespace();

  // PubSub
  rotor_vels_pub_ =
    nh.advertise<mav_msgs::Actuators>("/" + drone_name + "/command/motor_speed", 1, false);
  feedback_pub_ =
    nh.advertise<multirotor_msgs::ControllerFeedback>("/multirotor_controller/feedback", 1, false);
  bs_sub_ = nh.subscribe("/" + drone_name + "/base_state", 1, &Controller::bsCb, this);
  if (transformable_)
  {
    js_sub_ = nh.subscribe("/" + drone_name + "/joint_states", 1, &Controller::jsCb, this);
  }
  cmd_sub_ = nh.subscribe("/multirotor_controller/command", 1, &Controller::commandCb, this);

  // Dynamic Reconfigure
  ConfigServer::CallbackType f = boost::bind(&Controller::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void Controller::runOnce()
{
  auto& pos_des = feedback_.desired_position;
  auto& acc_des = feedback_.desired_acceleration;
  auto& roll_des = feedback_.desired_roll;
  auto& pitch_des = feedback_.desired_pitch;
  auto& yaw_des = feedback_.desired_yaw;
  auto& U = feedback_.thrust_force_sum;
  auto& u = feedback_.thrust_forces;

  // 位置とヨー角の目標値はコマンドどおり
  pos_des = cmd_.target_position;
  yaw_des = cmd_.target_yaw_angle;

  // stopwatch_.start();
  pos_controller_.update(bs_.pose.pos, pos_des, bs_.twist.vel, Vector::Zero(), acc_des);
  acc_controller_.update(acc_des, yaw_des, U, roll_des, pitch_des);
  rot_controller_.update(bs_, q_, U, roll_des, pitch_des, yaw_des, u);
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
      ROS_FATAL_STREAM(
        "Negative thrust force: "
        << "u = " << u[i]);
      // TODO: 防御モードに移行
    }
    rotor_vels.angular_velocities[i] = sqrt(max(u[i], 0.) / rotor_props_[i].motor_constant);
  }
}

bool Controller::allSubscribed()
{
  if (transformable_)
  {
    return js_subscribed_ && cmd_subscribed_;
  }
  else
  {
    return cmd_subscribed_;
  }
}

void Controller::bsCb(const dh_kdl_msgs::PoseVel& msg)
{
  bs_ = msg;

  // トピックが揃っていたら，状態を観測するたびに一回だけ制御器を回す．
  if (allSubscribed())
  {
    runOnce();
  }
}

void Controller::jsCb(const sensor_msgs::JointState& msg)
{
  if (msg.name.size() != msg.position.size())
  {
    ROS_ERROR_STREAM("The size of joint name and position is different.");
    js_subscribed_ = false;
    return;
  }

  for (const auto& jnt_name : required_joints_)
  {
    try
    {
      const auto msg_idx = dh_std::findIndex(msg.name, jnt_name);  // msg内でのインデックス
      const auto& jnt_pos = msg.position[msg_idx];
      const auto& kdl_idx = kdl_model_.jointIndex(jnt_name);  // Tree内でのインデックス
      q_(kdl_idx) = jnt_pos;
    }
    catch (const exception& e)
    {
      ROS_ERROR_STREAM(e.what());
      js_subscribed_ = false;
      return;
    }
  }

  js_subscribed_ = true;
}

void Controller::commandCb(const multirotor_msgs::Command& msg)
{
  cmd_ = msg;
  cmd_subscribed_ = true;
}

void Controller::dynamicReconfigureCb(
  const multirotor_controller::ControllerConfig& cfg,
  uint32_t level)
{
  pos_controller_.reconfigure(cfg.natural_frequency, cfg.damping_ratio);
  rot_controller_.reconfigure(
    cfg.prediction_horizon, cfg.prediction_steps, cfg.rotation_decay, cfg.angular_velocity_decay,
    cfg.rotation_weight, cfg.angular_velocity_weight, cfg.thrust_force_weight,
    cfg.thrust_force_rate_weight);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multirotor_controller");
  ros::NodeHandle nh;
  Controller node(nh);
  ros::spin();
}
