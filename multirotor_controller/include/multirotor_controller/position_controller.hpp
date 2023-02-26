#pragma once

#include <kdl/frames.hpp>

class PositionController
{
public:
  PositionController();

  void update(
    const KDL::Vector& pos,
    const KDL::Vector& pos_des,
    const KDL::Vector& vel,
    const KDL::Vector& vel_des,
    KDL::Vector& acc_out);
  
  void reconfigure(double natural_freq, double damp_ratio);

private:
  // rosparam
  double wn_;
  double zeta_;

  double kp_;
  double kd_;

  void getParams();
};
