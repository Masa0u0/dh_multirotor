#include <string>
#include <iostream>
#include <termios.h>
#include <stdio.h>
#include <cstring>
#include <unistd.h>

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/struct.hpp>
#include <dh_std_tools/algorithm.hpp>
#include <dh_ros_tools/rosparam.hpp>

#include <multirotor_msgs/Command.h>

#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_R 0x43
#define KEYCODE_L 0x44

using namespace std;
using namespace dh_std;

/**
 * @brief キーボード入力を受け取り，{world}から見た{base}のPoseを発行する．
 */
class CommandHandler
{
public:
  explicit CommandHandler(ros::NodeHandle& nh);

  void run();

private:
  // rosparam
  double key_repeat_freq_;  // キーボードの連続入力の周波数(PC依存)
  double max_linvel_;       // 並進速度の大きさの最大値
  double max_angvel_;       // 回転速度の大きさの最大値
  Range<double> x_limit_;
  Range<double> y_limit_;
  Range<double> z_limit_;
  Range<double> yaw_limit_;

  // other
  termios tempcopy_, changed_;
  double freq_;
  double delta_pos_;              // 1度のキーボード入力での並進位置の変化量
  double delta_rot_;              // 1度のキーボード入力での回転位置の変化量
  multirotor_msgs::Command cmd_;  // 位置コマンド

  ros::Publisher cmd_pub_;

  void getParams();
  void prepare(int fd);
};

CommandHandler::CommandHandler(ros::NodeHandle& nh)
{
  getParams();

  freq_ = key_repeat_freq_ * 10.;  // 全てのキーボード入力を拾うためにオーバーサンプリング
  delta_pos_ = max_linvel_ / key_repeat_freq_;
  delta_rot_ = max_angvel_ / key_repeat_freq_;

  // z座標の初期値を制限の下限に設定
  cmd_.target_position.z(z_limit_.lower);

  cmd_pub_ = nh.advertise<multirotor_msgs::Command>("/multirotor_controller/command", 1, false);

  prepare(0);
}

void CommandHandler::run()
{
  auto& x = cmd_.target_position(0);
  auto& y = cmd_.target_position(1);
  auto& z = cmd_.target_position(2);
  auto& yaw = cmd_.target_yaw_angle;

  char c;
  ros::Rate rate(freq_);

  while (ros::ok())
  {
    if (read(0, &c, 1) < 0)
    {
      ROS_ERROR("Failed to read keyboard input.");
      break;
    }

    // いきなりCtrl+Cを押すとループから抜けられずにバグるため，必ず"q"で抜けるようにする．
    if (c == 'q')
    {
      ROS_INFO("q is detected. Command handling is terminated.");
      break;
    }

    switch (c)
    {
      case 'w':  // X+
      {
        x = dh_std::clamp(x + delta_pos_, x_limit_.lower, x_limit_.upper, "X");
        break;
      }
      case 's':  // X-
      {
        x = dh_std::clamp(x - delta_pos_, x_limit_.lower, x_limit_.upper, "X");
        break;
      }
      case 'a':  // Y+
      {
        y = dh_std::clamp(y + delta_pos_, y_limit_.lower, y_limit_.upper, "Y");
        break;
      }
      case 'd':  // Y-
      {
        y = dh_std::clamp(y - delta_pos_, y_limit_.lower, y_limit_.upper, "Y");
        break;
      }
      case KEYCODE_U:  // Z+
      {
        z = dh_std::clamp(z + delta_pos_, z_limit_.lower, z_limit_.upper, "Z");
        break;
      }
      case KEYCODE_D:  // Z-
      {
        z = dh_std::clamp(z - delta_pos_, z_limit_.lower, z_limit_.upper, "Z");
        break;
      }
      case KEYCODE_L:  // yaw+
      {
        yaw = dh_std::clamp(yaw + delta_rot_, yaw_limit_.lower, yaw_limit_.upper, "Yaw");
        break;
      }
      case KEYCODE_R:  // yaw-
      {
        yaw = dh_std::clamp(yaw - delta_rot_, yaw_limit_.lower, yaw_limit_.upper, "Yaw");
        break;
      }
    }

    // キーボード入力をリセット
    // リセットしないと同じコマンドが連続して入力されてしまう．
    c = 0;

    cmd_pub_.publish(cmd_);

    rate.sleep();
  }

  tcsetattr(0, TCSANOW, &tempcopy_);
}

void CommandHandler::getParams()
{
  key_repeat_freq_ = dh_ros::getParam<double>("~key_repeat_freq");
  max_linvel_ = dh_ros::getParam<double>("~max_linear_velocity");
  max_angvel_ = dh_ros::getParam<double>("~max_angular_velocity");
  x_limit_.lower = dh_ros::getParam<double>("~pose_limit/x/min");
  x_limit_.upper = dh_ros::getParam<double>("~pose_limit/x/max");
  y_limit_.lower = dh_ros::getParam<double>("~pose_limit/y/min");
  y_limit_.upper = dh_ros::getParam<double>("~pose_limit/y/max");
  z_limit_.lower = dh_ros::getParam<double>("~pose_limit/z/min");
  z_limit_.upper = dh_ros::getParam<double>("~pose_limit/z/max");
  yaw_limit_.lower = dh_ros::getParam<double>("~pose_limit/yaw/min");
  yaw_limit_.upper = dh_ros::getParam<double>("~pose_limit/yaw/max");

  ROS_ASSERT(key_repeat_freq_ > 0.);
  ROS_ASSERT(max_linvel_ > 0.);
  ROS_ASSERT(max_angvel_ > 0.);
  ROS_ASSERT(x_limit_.lower < 0. && 0. < x_limit_.upper);
  ROS_ASSERT(y_limit_.lower < 0. && 0. < y_limit_.upper);
  ROS_ASSERT(0. < z_limit_.lower && z_limit_.lower < z_limit_.upper);
  ROS_ASSERT(yaw_limit_.lower < 0. && 0. < yaw_limit_.upper);
}

void CommandHandler::prepare(int fd)
{
  tcgetattr(fd, &tempcopy_);
  memcpy(&changed_, &tempcopy_, sizeof(termios));

  changed_.c_lflag &= ~(ICANON | ECHO);
  changed_.c_cc[VEOL] = 1;
  changed_.c_cc[VEOF] = 2;

  // 入力受付のタイムリミットを設定
  // https://stackoverflow.com/questions/2917881/how-to-implement-a-timeout-in-read-function-call
  changed_.c_cc[VMIN] = 0.;
  changed_.c_cc[VTIME] = 10. / freq_;

  tcsetattr(fd, TCSANOW, &changed_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_teleop");
  ros::NodeHandle nh;
  CommandHandler node(nh);
  node.run();
}
