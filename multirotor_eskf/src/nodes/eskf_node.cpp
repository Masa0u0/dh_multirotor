#include "../../include/multirotor_eskf/eskf_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_estimator_eskf");
  ros::NodeHandle nh;
  ErrorStateKalmanFilterRos imu_filter(nh);
  ros::spin();
  return 0;
}
