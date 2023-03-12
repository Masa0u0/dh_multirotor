#pragma once

#include <vector>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>

enum dataType
{
  isImuData = 0,
  isMocapData = 1
};

struct imuData
{
  Eigen::Vector3d accel;
  Eigen::Vector3d gyro;
  ros::Time stamp;
};

struct mocapData
{
  Eigen::Vector3d pos;
  Eigen::Quaterniond quat;
  ros::Time stamp;
  ros::Time receivedTime;
};

class DataFiles
{
public:
  std::ifstream readerMixed;
  std::ifstream readerIMU;
  std::ifstream readerMocap;

  DataFiles(std::string path);

  int getNext(std::ifstream& file, mocapData& mocap, imuData& imu, int& type);
  int getNextTimeCorrected(
    std::ifstream& mocapFile,
    std::ifstream& imuFile,
    mocapData& mocap,
    imuData& imu,
    int& type);
  int getNextNotCorrected(std::ifstream& mixedFile, mocapData& mocap, imuData& imu, int& type);
  int getNextTimeReceived(
    std::ifstream& mocapFile,
    std::ifstream& imuFile,
    mocapData& mocap,
    imuData& imu,
    int& type);
};
