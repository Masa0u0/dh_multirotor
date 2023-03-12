#include <iostream>
#include <chrono>
#include <math.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_broadcaster.h>

#include "../../include/multirotor_eskf/eskf.hpp"
#include "../../include/multirotor_eskf/parse_data_files.hpp"
#include "../../include/multirotor_eskf/l_time.hpp"

#define SQ(x) (x * x)
#define GRAVITY 9.812  // London g value.
#define I_3 (Matrix3d::Identity())

using namespace std;
using namespace Eigen;

void postTF(ErrorStateKalmanFilter eskf, tf::TransformBroadcaster tb, string name);
double difference(ErrorStateKalmanFilter eskfRef, ErrorStateKalmanFilter eskfTest);

int main(int argc, char** argv)
{
  double sigma_accel =
    0.00124;                  // [m/s^2]  (value derived from Noise Spectral Density in datasheet)
  double sigma_gyro = 0.276;  // [rad/s] (value derived from Noise Spectral Density in datasheet)
  double sigma_accel_drift =
    0.001 * sigma_accel;  // [m/s^2 sqrt(s)] (Educated guess, real value to be measured)
  double sigma_gyro_drift =
    0.001 * sigma_gyro;  // [rad/s sqrt(s)] (Educated guess, real value to be measured)

  double sigma_init_pos = 1.0;                            // [m]
  double sigma_init_vel = 0.1;                            // [m/s]
  double sigma_init_dtheta = 1.0;                         // [rad]
  double sigma_init_accel_bias = 10 * sigma_accel_drift;  // [m/s^2]
  double sigma_init_gyro_bias = 10 * sigma_gyro_drift;    // [rad/s]

  double sigma_mocap_pos = 0.003;  // [m]
  double sigma_mocap_rot = 0.03;   // [rad]
  // initialise many ESKF's for testing:
  //  eskfSpoof - Will be fed data as if there is no lag, this is the comparison case
  //  eskfAsArrive - No time correction method used, just fed lagged data.
  //  eskfAverageIMU - Uses the average of the IMU measurements as representative since the delayed
  //  measurment should have arrived eskfNewIMU - Uses newest IMU measurement as representative
  //  since the delayed measurment should have arrived eskfFullLarson - Uses the method as described
  //  by Larson et al (same as above, but uses all IMU measurments rather than simplificationis)
  //  eskfUpdateToNew - Keeps the state in a buffer, calculates the update that would have been made
  //  if there was no lag, then applies directly to current state.
  ErrorStateKalmanFilter eskfSpoof(
    Vector3d(0, 0, -GRAVITY),  // Acceleration due to gravity in global frame
    ErrorStateKalmanFilter::makeState(
      Vector3d(0, 0, 1),                              // init pos
      Vector3d(0, 0, 0),                              // init vel
      Quaterniond(AngleAxisd(0, Vector3d(0, 0, 1))),  // init quaternion
      Vector3d(-1.26, -1.09, -1.977),                 // init accel bias
      Vector3d(0.114, -0.01, 0)                       // init gyro bias
      ),
    ErrorStateKalmanFilter::makeP(
      SQ(sigma_init_pos) * I_3, SQ(sigma_init_vel) * I_3, SQ(sigma_init_dtheta) * I_3,
      SQ(sigma_init_accel_bias) * I_3, SQ(sigma_init_gyro_bias) * I_3),
    SQ(sigma_accel), SQ(sigma_gyro), SQ(sigma_accel_drift), SQ(sigma_gyro_drift),
    ErrorStateKalmanFilter::delayTypes::noMethod, 100);
  ErrorStateKalmanFilter eskfAsArrive(
    Vector3d(0, 0, -GRAVITY),  // Acceleration due to gravity in global frame
    ErrorStateKalmanFilter::makeState(
      Vector3d(0, 0, 1),                              // init pos
      Vector3d(0, 0, 0),                              // init vel
      Quaterniond(AngleAxisd(0, Vector3d(0, 0, 1))),  // init quaternion
      Vector3d(-1.26, -1.09, -1.977),                 // init accel bias
      Vector3d(0.114, -0.01, 0)                       // init gyro bias
      ),
    ErrorStateKalmanFilter::makeP(
      SQ(sigma_init_pos) * I_3, SQ(sigma_init_vel) * I_3, SQ(sigma_init_dtheta) * I_3,
      SQ(sigma_init_accel_bias) * I_3, SQ(sigma_init_gyro_bias) * I_3),
    SQ(sigma_accel), SQ(sigma_gyro), SQ(sigma_accel_drift), SQ(sigma_gyro_drift),
    ErrorStateKalmanFilter::delayTypes::noMethod, 100);

  ErrorStateKalmanFilter eskfUpdateToNew(
    Vector3d(0, 0, -GRAVITY),  // Acceleration due to gravity in global frame
    ErrorStateKalmanFilter::makeState(
      Vector3d(0, 0, 1),                              // init pos
      Vector3d(0, 0, 0),                              // init vel
      Quaterniond(AngleAxisd(0, Vector3d(0, 0, 1))),  // init quaternion
      Vector3d(-1.26, -1.09, -1.977),                 // init accel bias
      Vector3d(0.114, -0.01, 0)                       // init gyro bias
      ),
    ErrorStateKalmanFilter::makeP(
      SQ(sigma_init_pos) * I_3, SQ(sigma_init_vel) * I_3, SQ(sigma_init_dtheta) * I_3,
      SQ(sigma_init_accel_bias) * I_3, SQ(sigma_init_gyro_bias) * I_3),
    SQ(sigma_accel), SQ(sigma_gyro), SQ(sigma_accel_drift), SQ(sigma_gyro_drift),
    ErrorStateKalmanFilter::delayTypes::applyUpdateToNew, 100);

  // Start section that manages collecting data from files and feeding them to the ESKFs.
  // eskfSpoof is a special case that uses separated files for IMU and Mocap, it finds the next one
  // based on timestamp (of data generration) The others use the data as they arrived during data
  // collection (randomised delay of the mocap ~ 18ms delay) All comparisons are made on the arrival
  // of IMU data, as this is the same for all of them including the spoof case.

  ros::init(argc, argv, "TestESKF");

  ros::NodeHandle node;

  ros::Publisher posePub =
    node.advertise<geometry_msgs::PoseWithCovariance>("predictiedLocation", 1);

  tf::TransformBroadcaster tb;

  DataFiles filesObj("../timeSeries/hardWave");

  int flag = 0;
  int spoofIMUcount = 0;
  int testIMUCount = 0;
  int mocapCount = 0;
  double asArriveErrorAcc = 0;
  double asArriveError = 0;
  double asArriveLargestError = 0;
  double upToNewError = 0;
  double upToNewErrorAcc = 0;
  double upToNewLargestError = 0;
  double asArriveErrorAccSqu = 0;
  double upToNewErrorAccSqu = 0;

  while (!flag)
  {
    imuData imu;
    mocapData mocap;
    int type;

    if (spoofIMUcount < testIMUCount)
    {
      // do spoof.
      flag = filesObj.getNextTimeCorrected(
        filesObj.readerMocap, filesObj.readerIMU, mocap, imu, type);  // emulates lag free data
      if (type == isImuData)
      {
        static ros::Time oldTime;
        ros::Duration diff = imu.stamp - oldTime;
        oldTime = imu.stamp;
        lTime stamp(imu.stamp.sec, imu.stamp.nsec);
        if (diff.toSec() > 1999)
          diff.fromSec(0.001);
        spoofIMUcount++;
        eskfSpoof.predictIMU(imu.accel, imu.gyro, diff.toSec(), stamp);
      }

      if (type == isMocapData)
      {
        lTime stamp(mocap.stamp.sec, mocap.stamp.nsec);
        lTime now(mocap.receivedTime.sec, mocap.receivedTime.nsec);
        eskfSpoof.measurePos(mocap.pos, SQ(sigma_mocap_pos) * I_3, stamp, now);
        eskfSpoof.measureQuat(mocap.quat, SQ(sigma_mocap_rot) * I_3, stamp, now);

        tf::StampedTransform meas;
        meas.setOrigin(tf::Vector3(mocap.pos[0], mocap.pos[1], mocap.pos[2]));
        meas.setRotation(
          tf::Quaternion(mocap.quat.x(), mocap.quat.y(), mocap.quat.z(), mocap.quat.w()));
        meas.stamp_ = ros::Time::now();
        meas.frame_id_ = "mocha_world";
        meas.child_frame_id_ = "meas";
        tb.sendTransform(meas);
      }
    }
    else
    {
      // do candidates.
      flag = filesObj.getNextNotCorrected(filesObj.readerMixed, mocap, imu, type);
      if (type == isImuData)
      {
        testIMUCount++;
        static ros::Time oldTime;
        ros::Duration diff = imu.stamp - oldTime;
        oldTime = imu.stamp;
        lTime stamp(imu.stamp.sec, imu.stamp.nsec);
        if (diff.toSec() > 1999)
          diff.fromSec(0.001);
        eskfAsArrive.predictIMU(imu.accel, imu.gyro, diff.toSec(), stamp);
        eskfUpdateToNew.predictIMU(imu.accel, imu.gyro, diff.toSec(), stamp);
      }

      if (type == isMocapData)
      {
        lTime stamp(mocap.stamp.sec, mocap.stamp.nsec);

        lTime now(mocap.receivedTime.sec, mocap.receivedTime.nsec);

        mocapCount++;

        eskfAsArrive.measurePos(mocap.pos, SQ(sigma_mocap_pos) * I_3, stamp, now);
        eskfAsArrive.measureQuat(mocap.quat, SQ(sigma_mocap_rot) * I_3, stamp, now);

        eskfUpdateToNew.measurePos(mocap.pos, SQ(sigma_mocap_pos) * I_3, stamp, now);
        eskfUpdateToNew.measureQuat(mocap.quat, SQ(sigma_mocap_rot) * I_3, stamp, now);
      }
    }

    if (testIMUCount == spoofIMUcount)
    {
      postTF(eskfSpoof, tb, "spoof");
      postTF(eskfAsArrive, tb, "asArrive");
      postTF(eskfUpdateToNew, tb, "UpToNew");
      double asArriveEr = difference(eskfSpoof, eskfAsArrive);
      double upToNewEr = difference(eskfSpoof, eskfUpdateToNew);

      if (mocapCount > 100)
      {  // stop max error just being the initialisation error
        asArriveErrorAcc += asArriveEr;
        upToNewErrorAcc += upToNewEr;
        asArriveErrorAccSqu += pow(asArriveEr, 2);
        upToNewErrorAccSqu += pow(upToNewEr, 2);
        if (asArriveEr > asArriveLargestError)
          asArriveLargestError = asArriveEr;
        if (upToNewEr > upToNewLargestError)
          upToNewLargestError = upToNewEr;
      }
    }
  }
  asArriveError = asArriveErrorAcc / (testIMUCount - 100);
  upToNewError = upToNewErrorAcc / (testIMUCount - 100);
  Vector3d accelBias = eskfUpdateToNew.getAccelBias();
  Vector3d gyroBias = eskfUpdateToNew.getGyroBias();
  cout << "accelBias" << accelBias << endl << "  gyroBias " << gyroBias << endl;
  cout << "asArrive pos error average = " << asArriveError << endl;
  cout << "asArrive largest error = " << asArriveLargestError << endl;
  cout << "asArrive varience error = "
       << asArriveErrorAccSqu / ((testIMUCount - 100)) - pow(asArriveError, 2) << endl;
  cout << "upToNew pos error average = " << upToNewError << endl;
  cout << "upToNew largest error = " << upToNewLargestError << endl;
  cout << "upToNew varience error = "
       << upToNewErrorAccSqu / ((testIMUCount - 100)) - pow(upToNewError, 2) << endl;
}

double difference(ErrorStateKalmanFilter eskfRef, ErrorStateKalmanFilter eskfTest)
{
  Vector3d posRef = eskfRef.getPos();
  Vector3d posTest = eskfTest.getPos();
  Vector3d out = posTest - posRef;
  return out.norm();
}

void postTF(ErrorStateKalmanFilter eskf, tf::TransformBroadcaster tb, string name)
{
  tf::StampedTransform pred;
  Vector3d pos = eskf.getPos();
  Quaterniond quat = eskf.getQuat();
  pred.setOrigin(tf::Vector3(pos[0], pos[1], pos[2]));
  pred.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
  pred.stamp_ = ros::Time::now();
  pred.frame_id_ = "mocha_world";
  pred.child_frame_id_ = name;
  tb.sendTransform(pred);
}
