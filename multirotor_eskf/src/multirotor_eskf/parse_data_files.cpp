#include "../../include/multirotor_eskf/parse_data_files.hpp"

using namespace std;
using namespace Eigen;

int DataFiles::getNext(ifstream& file, mocapData& mocap, imuData& imu, int& type)
{
  string str;
  getline(file, str, ',');
  if (!str.compare("IMU"))
  {
    type = isImuData;

    getline(file, str, ',');
    imu.accel[0] = atof(str.c_str());

    getline(file, str, ',');
    imu.accel[1] = atof(str.c_str());

    getline(file, str, ',');
    imu.accel[2] = atof(str.c_str());

    getline(file, str, ',');
    imu.gyro[0] = atof(str.c_str());

    getline(file, str, ',');
    imu.gyro[1] = atof(str.c_str());

    getline(file, str, ',');
    imu.gyro[2] = atof(str.c_str());

    getline(file, str, ',');
    imu.stamp.sec = atoi(str.c_str());

    getline(file, str, '\n');
    imu.stamp.nsec = atoi(str.c_str());
    return 0;
  }
  if (!str.compare("Mocap"))
  {
    type = isMocapData;

    getline(file, str, ',');
    mocap.pos[0] = atof(str.c_str());

    getline(file, str, ',');
    mocap.pos[1] = atof(str.c_str());

    getline(file, str, ',');
    mocap.pos[2] = atof(str.c_str());

    getline(file, str, ',');
    double qx = atof(str.c_str());

    getline(file, str, ',');
    double qy = atof(str.c_str());

    getline(file, str, ',');
    double qz = atof(str.c_str());

    getline(file, str, ',');
    double qw = atof(str.c_str());

    mocap.quat = Quaterniond(qw, qx, qy, qz);

    getline(file, str, ',');
    mocap.stamp.sec = atoi(str.c_str());

    getline(file, str, ',');
    mocap.stamp.nsec = atoi(str.c_str());

    getline(file, str, ',');
    mocap.receivedTime.sec = atoi(str.c_str());

    getline(file, str, '\n');
    mocap.receivedTime.nsec = atoi(str.c_str());
    return 0;
  }

  file.close();

  return 1;
}

int DataFiles::getNextTimeCorrected(
  ifstream& mocapFile,
  ifstream& imuFile,
  mocapData& mocap,
  imuData& imu,
  int& type)
{
  static mocapData mocapTemp;
  static imuData imuTemp;
  static int primed = 0;
  int typeTemp;
  int error = 0;
  ros::Duration hack(0);
  if (!primed)
  {
    primed = 1;
    error = getNext(mocapFile, mocapTemp, imuTemp, typeTemp);
    error += getNext(imuFile, mocapTemp, imuTemp, typeTemp);

    ros::Duration dur = imuTemp.stamp - mocapTemp.stamp + hack;
    if (dur.toSec() >= 0)
    {
      typeTemp = isMocapData;
    }
    else
    {
      typeTemp = isImuData;
    }
    mocap = mocapTemp;
    imu = imuTemp;
    type = typeTemp;
    return error;
  }
  else
  {
    // the oldest one would have been sent, update the other one.
    ros::Duration dur = imuTemp.stamp - mocapTemp.stamp + hack;
    if (dur.toSec() >= 0)
    {
      error = getNext(mocapFile, mocapTemp, imuTemp, typeTemp);
    }
    else
    {
      error = getNext(imuFile, mocapTemp, imuTemp, typeTemp);
    }
    dur = imuTemp.stamp - mocapTemp.stamp + hack;
    if (dur.toSec() >= 0)
    {
      typeTemp = isMocapData;
    }
    else
    {
      typeTemp = isImuData;
    }
    mocap = mocapTemp;
    imu = imuTemp;
    type = typeTemp;
    return error;
  }
}

int DataFiles::getNextTimeReceived(
  ifstream& mocapFile,
  ifstream& imuFile,
  mocapData& mocap,
  imuData& imu,
  int& type)
{
  static mocapData mocapTemp;
  static imuData imuTemp;
  static int primed = 0;
  int typeTemp;
  int error = 0;
  ros::Duration hack(0);
  if (!primed)
  {
    primed = 1;
    error = getNext(mocapFile, mocapTemp, imuTemp, typeTemp);
    error += getNext(imuFile, mocapTemp, imuTemp, typeTemp);

    ros::Duration dur = imuTemp.stamp - mocapTemp.receivedTime + hack;
    if (dur.toSec() >= 0)
    {
      typeTemp = isMocapData;
    }
    else
    {
      typeTemp = isImuData;
    }
    mocap = mocapTemp;
    imu = imuTemp;
    type = typeTemp;
    return error;
  }
  else
  {
    // the oldest one would have been sent, update the other one.
    ros::Duration dur = imuTemp.stamp - mocapTemp.receivedTime + hack;
    if (dur.toSec() >= 0)
    {
      error = getNext(mocapFile, mocapTemp, imuTemp, typeTemp);
    }
    else
    {
      error = getNext(imuFile, mocapTemp, imuTemp, typeTemp);
    }
    dur = imuTemp.stamp - mocapTemp.receivedTime + hack;
    if (dur.toSec() >= 0)
    {
      typeTemp = isMocapData;
    }
    else
    {
      typeTemp = isImuData;
    }
    mocap = mocapTemp;
    imu = imuTemp;
    type = typeTemp;
    return error;
  }
}

int DataFiles::getNextNotCorrected(ifstream& mixedFile, mocapData& mocap, imuData& imu, int& type)
{
  static mocapData mocapTemp;
  static imuData imuTemp;
  static int primed = 0;
  int typeTemp;
  int error = 0;
  primed = 1;
  error = getNext(mixedFile, mocapTemp, imuTemp, typeTemp);
  mocap = mocapTemp;
  imu = imuTemp;
  type = typeTemp;
  return error;
}

DataFiles::DataFiles(string path)

{
  // init files and waste the header line.
  stringstream ssMixed;
  ssMixed << path << "/mixedTimeSeries.txt";
  readerMixed.open(ssMixed.str());
  char var[256];
  readerMixed.getline(var, sizeof(var), '\n');

  stringstream ssMocap;
  ssMocap << path << "/mocapTimeSeries.txt";
  readerMocap.open(ssMocap.str());
  readerMocap.getline(var, sizeof(var), '\n');

  stringstream ssIMU;
  ssIMU << path << "/accelGyroTimeSeries.txt";
  readerIMU.open(ssIMU.str());
  readerIMU.getline(var, sizeof(var), '\n');
}
