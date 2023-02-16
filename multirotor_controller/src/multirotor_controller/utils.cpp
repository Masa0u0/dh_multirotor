#include <dh_ros_tools/rosparam.hpp>

#include "../../include/multirotor_controller/utils.hpp"

using namespace std;

vector<RotorProperty> getRotorProperties()
{
  const string drone_name = dh_ros::getParam<string>("/drone_name");
  const int num_rotors = dh_ros::getParam<int>("/num_rotors");

  vector<RotorProperty> res(num_rotors);
  for (int i = 0; i < num_rotors; ++i)
  {
    string rotor_prefix = "/rotor_" + to_string(i);

    res[i].link_name = dh_ros::getParam<string>(rotor_prefix + "/link_name");
    res[i].max_velocity = dh_ros::getParam<double>(rotor_prefix + "/max_velocity");
    res[i].motor_constant = dh_ros::getParam<double>(rotor_prefix + "/motor_constant");
    res[i].moment_constant = dh_ros::getParam<double>(rotor_prefix + "/moment_constant");

    string direction = dh_ros::getParam<string>(rotor_prefix + "/direction");
    if (direction == "ccw")
    {
      res[i].direction = 1;
    }
    else if (direction == "cw")
    {
      res[i].direction = -1;
    }
    else
    {
      throw runtime_error("direction must be 'cw' or 'ccw'.");
    }
  }

  return res;
}
