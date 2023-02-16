#pragma once

#include <string>
#include <vector>

struct RotorProperty
{
  std::string link_name;
  int direction;
  double max_velocity;
  double motor_constant;
  double moment_constant;
};

std::vector<RotorProperty> getRotorProperties();
