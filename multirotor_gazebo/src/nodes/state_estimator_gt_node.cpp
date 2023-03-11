#include "../../include/multirotor_gazebo/state_estimator_gt.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_estimator_gt");
  ros::NodeHandle nh;
  StateEstimatorGT node(nh);
  ros::spin();
}
