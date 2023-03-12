#include "../../include/multirotor_eskf/eskf.hpp"
#include "../../include/multirotor_eskf/unrolled_joseph.hpp"

#define SQ(x) (x * x)
#define I_3 (Matrix3d::Identity())
#define I_dx (Matrix<double, dSTATE_SIZE, dSTATE_SIZE>::Identity())

using namespace Eigen;
using namespace std;

ESKF::ESKF(
  Vector3d a_gravity,
  const Matrix<double, STATE_SIZE, 1>& initialState,
  const Matrix<double, dSTATE_SIZE, dSTATE_SIZE>& initalP,
  double var_acc,
  double var_omega,
  double var_acc_bias,
  double var_omega_bias,
  int delayHandling,
  int bufferL)
  : var_acc_(var_acc),
    var_omega_(var_omega),
    var_acc_bias_(var_acc_bias),
    var_omega_bias_(var_omega_bias),
    a_gravity_(a_gravity),
    nominalState_(initialState),
    P_(initalP)
{
  // Jacobian of the state transition: page 59, eqn 269
  // Precompute constant part only
  F_x_.setZero();
  // dPos row
  F_x_.block<3, 3>(dPOS_IDX, dPOS_IDX) = I_3;
  // dVel row
  F_x_.block<3, 3>(dVEL_IDX, dVEL_IDX) = I_3;
  // dTheta row
  // dAccelBias row
  F_x_.block<3, 3>(dAB_IDX, dAB_IDX) = I_3;
  // dGyroBias row
  F_x_.block<3, 3>(dGB_IDX, dGB_IDX) = I_3;

  // how to handle delayed messurements.
  delayHandling_ = delayHandling;
  bufferL_ = bufferL;
  recentPtr = 0;
  firstMeasTime = lTime(INT32_MAX, INT32_MAX);

  // handle time delay methods
  if (delayHandling_ == larsonAverageIMU || delayHandling_ == larsonFull)
  {
    // init circular buffer for IMU
    imuHistoryPtr_ = new vector<imuMeasurement>(bufferL_);
    PHistoryPtr_ = new vector<pair<lTime, Matrix<double, dSTATE_SIZE, dSTATE_SIZE>>>(bufferL);
    for (int i = 0; i < bufferL; i++)
    {
      imuHistoryPtr_->at(i).time = lTime(0, 0);
    }
    Mptr = new Matrix<double, dSTATE_SIZE, dSTATE_SIZE>;
  }
  if (delayHandling_ == larsonNewestIMU)
  {
    // init newest value
    lastImu_.time = lTime(0, 0);
    Mptr = new Matrix<double, dSTATE_SIZE, dSTATE_SIZE>;
  }
  if (delayHandling_ == applyUpdateToNew || delayHandling_ == larsonAverageIMU)
  {
    // init circular buffer for state
    stateHistoryPtr_ = new vector<pair<lTime, Matrix<double, STATE_SIZE, 1>>>(bufferL_);
    for (int i = 0; i < bufferL; i++)
    {
      stateHistoryPtr_->at(i).first = lTime(0, 0);
    }
  }
}

Matrix<double, STATE_SIZE, 1> ESKF::makeState(
  const Vector3d& p,
  const Vector3d& v,
  const Quaterniond& q,
  const Vector3d& a_b,
  const Vector3d& omega_b)
{
  Matrix<double, STATE_SIZE, 1> out;
  out << p, v, quatToHamilton(q).normalized(), a_b, omega_b;
  return out;
}

Matrix<double, dSTATE_SIZE, dSTATE_SIZE> ESKF::makeP(
  const Matrix3d& cov_pos,
  const Matrix3d& cov_vel,
  const Matrix3d& cov_dtheta,
  const Matrix3d& cov_a_b,
  const Matrix3d& cov_omega_b)
{
  Matrix<double, dSTATE_SIZE, dSTATE_SIZE> P;
  P.setZero();
  P.block<3, 3>(dPOS_IDX, dPOS_IDX) = cov_pos;
  P.block<3, 3>(dVEL_IDX, dVEL_IDX) = cov_vel;
  P.block<3, 3>(dTHETA_IDX, dTHETA_IDX) = cov_dtheta;
  P.block<3, 3>(dAB_IDX, dAB_IDX) = cov_a_b;
  P.block<3, 3>(dGB_IDX, dGB_IDX) = cov_omega_b;
  return P;
}

Matrix3d ESKF::getDCM()
{
  return getQuat().matrix();
}

Quaterniond ESKF::quatFromHamilton(const Vector4d& qHam)
{
  return Quaterniond((Vector4d() << qHam.block<3, 1>(1, 0),  // x, y, z
                      qHam.block<1, 1>(0, 0)                 // w
                      )
                       .finished());
}

Vector4d ESKF::quatToHamilton(const Quaterniond& q)
{
  return (Vector4d() << q.coeffs().block<1, 1>(3, 0),  // w
          q.coeffs().block<3, 1>(0, 0)                 // x, y, z
          )
    .finished();
}

Matrix3d ESKF::getSkew(const Vector3d& in)
{
  Matrix3d out;
  out << 0, -in(2), in(1), in(2), 0, -in(0), -in(1), in(0), 0;
  return out;
}

Matrix3d ESKF::rotVecToMat(const Vector3d& in)
{
  double angle = in.norm();
  Vector3d axis = (angle == 0) ? Vector3d(1, 0, 0) : in.normalized();
  AngleAxisd angAx(angle, axis);
  return angAx.toRotationMatrix();
}

Quaterniond ESKF::rotVecToQuat(const Vector3d& in)
{
  double angle = in.norm();
  Vector3d axis = (angle == 0) ? Vector3d(1, 0, 0) : in.normalized();
  return Quaterniond(AngleAxisd(angle, axis));
}

Vector3d ESKF::quatToRotVec(const Quaterniond& q)
{
  AngleAxisd angAx(q);
  return angAx.angle() * angAx.axis();
}

void ESKF::predictIMU(const Vector3d& a_m, const Vector3d& omega_m, const double dt, lTime stamp)
{
  recentPtr++;
  // handle time delay methods
  if (delayHandling_ == larsonAverageIMU || delayHandling_ == larsonFull)
  {
    // store the imu data for later.

    imuMeasurement thisMeas;
    thisMeas.time = stamp;
    thisMeas.acc = a_m;
    thisMeas.gyro = omega_m;
    imuHistoryPtr_->at(recentPtr % bufferL_) = thisMeas;
  }
  if (delayHandling_ == larsonNewestIMU)
  {
    // store only the newest imu
    imuMeasurement thisMeas;
    thisMeas.time = stamp;
    thisMeas.acc = a_m;
    thisMeas.gyro = omega_m;
    lastImu_ = thisMeas;
  }

  // DCM of current state
  Matrix3d Rot = getDCM();
  // Accelerometer measurement
  Vector3d acc_body = a_m - getAccelBias();
  Vector3d acc_global = Rot * acc_body;
  // Gyro measruement
  Vector3d omega = omega_m - getGyroBias();
  Vector3d delta_theta = omega * dt;
  Quaterniond q_delta_theta = rotVecToQuat(delta_theta);
  Matrix3d R_delta_theta = q_delta_theta.toRotationMatrix();

  // Nominal state kinematics (eqn 259, pg 58)
  Vector3d delta_pos = getVel() * dt + 0.5f * (acc_global + a_gravity_) * dt * dt;
  nominalState_.block<3, 1>(POS_IDX, 0) += delta_pos;
  nominalState_.block<3, 1>(VEL_IDX, 0) += (acc_global + a_gravity_) * dt;
  nominalState_.block<4, 1>(QUAT_IDX, 0) = quatToHamilton(getQuat() * q_delta_theta).normalized();

  // // Jacobian of the state transition (eqn 269, page 59)
  // // Update dynamic parts only
  // // dPos row
  // F_x_.block<3, 3>(dPOS_IDX, dVEL_IDX).diagonal().fill(dt); // = I_3 * _dt
  // // dVel row
  // F_x_.block<3, 3>(dVEL_IDX, dTHETA_IDX) = -Rot * getSkew(acc_body) * dt;
  // F_x_.block<3, 3>(dVEL_IDX, dAB_IDX) = -Rot * dt;
  // // dTheta row
  // F_x_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) = R_delta_theta.transpose();
  // F_x_.block<3, 3>(dTHETA_IDX, dGB_IDX).diagonal().fill(-dt); // = -I_3 * dt;

  // Predict P and inject variance (with diagonal optimization)
  // P_ = F_x_*P_*F_x_.transpose();

  Matrix<double, dSTATE_SIZE, dSTATE_SIZE> Pnew;
  unrolledFPFt(P_, Pnew, dt, -Rot * getSkew(acc_body) * dt, -Rot * dt, R_delta_theta.transpose());
  P_ = Pnew;

  // Inject process noise
  P_.diagonal().block<3, 1>(dVEL_IDX, 0).array() += var_acc_ * SQ(dt);
  P_.diagonal().block<3, 1>(dTHETA_IDX, 0).array() += var_omega_ * SQ(dt);
  P_.diagonal().block<3, 1>(dAB_IDX, 0).array() += var_acc_bias_ * dt;
  P_.diagonal().block<3, 1>(dGB_IDX, 0).array() += var_omega_bias_ * dt;

  if (delayHandling_ == applyUpdateToNew || delayHandling_ == larsonAverageIMU)
  {
    // store state for later.
    pair<lTime, Matrix<double, STATE_SIZE, 1>> thisState;
    thisState.first = stamp;
    thisState.second = nominalState_;
    stateHistoryPtr_->at(recentPtr % bufferL_) = thisState;
  }
  if (delayHandling_ == larsonAverageIMU)
  {
    pair<lTime, Matrix<double, dSTATE_SIZE, dSTATE_SIZE>> thisP;
    thisP.first = stamp;
    thisP.second = P_;
    PHistoryPtr_->at(recentPtr % bufferL_) = thisP;
  }
}

// eqn 280, page 62
Matrix<double, 4, 3> ESKF::getQ_dtheta()
{
  Vector4d qby2 = 0.5f * getQuatVector();
  // Assing to letters for readability. Note Hamilton order.
  double w = qby2[0];
  double x = qby2[1];
  double y = qby2[2];
  double z = qby2[3];
  Matrix<double, 4, 3> Q_dtheta;
  Q_dtheta << -x, -y, -z, w, -z, y, z, w, -x, -y, x, w;
  return Q_dtheta;
}

// get best time from history of state
int ESKF::getClosestTime(vector<pair<lTime, Matrix<double, STATE_SIZE, 1>>>* ptr, lTime stamp)
{
  // we find the first time in the history that is older, or take the oldest one if the buffer does
  // not extend far enough
  int complete = 0;
  int index = recentPtr;
  while (!complete)
  {
    if (ptr->at(index % bufferL_).first <= stamp)
    {
      if (!ptr->at(index % bufferL_).first.isZero())
        return index % bufferL_;

      else
      {
        return recentPtr % bufferL_;
      }
    }
    index--;  // scroll back in time.
    if (index <= recentPtr - bufferL_)
      complete = 1;
  }
  return recentPtr % bufferL_;
}

// get best time from history of imu
int ESKF::getClosestTime(vector<imuMeasurement>* ptr, lTime stamp)
{
  // we find the first time in the history that is older, or take the oldest one if the buffer does
  // not extend far enough
  int complete = 0;
  int index = recentPtr;
  while (!complete)
  {
    if (ptr->at(index % bufferL_).time <= stamp)
    {
      if (!ptr->at(index % bufferL_).time.isZero())
        return index % bufferL_;

      else
      {
        return recentPtr % bufferL_;
      }
    }
    index--;  // scroll back in time.
    if (index <= recentPtr - bufferL_)
      complete = 1;
  }
  return recentPtr % bufferL_;
}

void ESKF::measurePos(
  const Vector3d& pos_meas,
  const Matrix3d& pos_covariance,
  lTime stamp,
  lTime now)
{
  // delta measurement
  if (firstMeasTime == lTime(INT32_MAX, INT32_MAX))
    firstMeasTime = now;

  Vector3d delta_pos;
  if (delayHandling_ == noMethod || delayHandling_ == larsonAverageIMU)
  {
    delta_pos = pos_meas - getPos();
    cout << "noMethod delta Pos: " << delta_pos << endl;
  }

  if (delayHandling_ == applyUpdateToNew)
  {
    if (lastMeasurement < stateHistoryPtr_->at((recentPtr + 1) % bufferL_).first)
      firstMeasTime = now;
    if (stamp > firstMeasTime)
    {
      int bestTimeIndex = getClosestTime(stateHistoryPtr_, stamp);
      delta_pos = pos_meas - stateHistoryPtr_->at(bestTimeIndex).second.block<3, 1>(POS_IDX, 0);
    }
    else
      delta_pos = pos_meas - getPos();
    cout << "UpToNew Pos: " << delta_pos << endl;
  }
  if (delayHandling_ == larsonAverageIMU)
  {
    if (lastMeasurement < imuHistoryPtr_->at((recentPtr + 1) % bufferL_).time)
      firstMeasTime = now;
  }
  lastMeasurement = now;
  // H is a trivial observation of purely the position
  Matrix<double, 3, dSTATE_SIZE> H;
  H.setZero();
  H.block<3, 3>(0, dPOS_IDX) = I_3;

  // Apply update
  update_3D(delta_pos, pos_covariance, H, stamp, now);
}

void ESKF::measureQuat(
  const Quaterniond& q_gb_meas,
  const Matrix3d& theta_covariance,
  lTime stamp,
  lTime now)
{
  // Transform the quaternion measurement to a measurement of delta_theta:
  // a rotation in the body frame from nominal to measured.
  // This is identical to the form of dtheta in the error_state,
  // so this becomes a trivial measurement of dtheta.
  if (firstMeasTime == lTime(INT32_MAX, INT32_MAX))
    firstMeasTime = now;
  Quaterniond q_gb_nominal = getQuat();
  if (delayHandling_ == noMethod || delayHandling_ == larsonAverageIMU)
  {
    q_gb_nominal = getQuat();
  }
  if (delayHandling_ == applyUpdateToNew)
  {
    if (stamp > firstMeasTime)
    {
      int bestTimeIndex = getClosestTime(stateHistoryPtr_, stamp);
      q_gb_nominal =
        quatFromHamilton(stateHistoryPtr_->at(bestTimeIndex).second.block<4, 1>(QUAT_IDX, 0));
    }
    else
      q_gb_nominal = getQuat();
  }
  Quaterniond q_bNominal_bMeas = q_gb_nominal.conjugate() * q_gb_meas;
  Vector3d delta_theta = quatToRotVec(q_bNominal_bMeas);
  // Because of the above construction, H is a trivial observation of dtheta
  Matrix<double, 3, dSTATE_SIZE> H;
  H.setZero();
  H.block<3, 3>(0, dTHETA_IDX) = I_3;

  // Apply update
  update_3D(delta_theta, theta_covariance, H, stamp, now);
}

void ESKF::update_3D(
  const Vector3d& delta_measurement,
  const Matrix3d& meas_covariance,
  const Matrix<double, 3, dSTATE_SIZE>& H,
  lTime stamp,
  lTime now)
{
  // generate M matrix for time correction methods
  int bestTimeIndex;
  int normalPass = 1;
  if (delayHandling_ == larsonAverageIMU)
  {
    if (stamp > firstMeasTime)
      normalPass = 0;
  }
  if (delayHandling_ == larsonAverageIMU && !normalPass)
  {
    imuMeasurement avMeas = getAverageIMU(stamp);
    double dt = (now - stamp).toSec();
    Vector3d acc_body = avMeas.acc - getAccelBias();
    Vector3d omega = avMeas.gyro - getGyroBias();
    Vector3d delta_theta = omega * dt;
    Quaterniond q_delta_theta = rotVecToQuat(delta_theta);
    Matrix3d R_delta_theta = q_delta_theta.toRotationMatrix();
    bestTimeIndex = getClosestTime(stateHistoryPtr_, stamp);

    Matrix3d Rot =
      quatFromHamilton(stateHistoryPtr_->at(bestTimeIndex).second.block<4, 1>(QUAT_IDX, 0))
        .matrix();
    // dPos row
    F_x_.block<3, 3>(dPOS_IDX, dVEL_IDX).diagonal().fill(dt);  // = I_3 * _dt
    // dVel row
    F_x_.block<3, 3>(dVEL_IDX, dTHETA_IDX) = -Rot * getSkew(acc_body) * dt;
    F_x_.block<3, 3>(dVEL_IDX, dAB_IDX) = -Rot * dt;
    // dTheta row
    F_x_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) = R_delta_theta.transpose();
    F_x_.block<3, 3>(dTHETA_IDX, dGB_IDX).diagonal().fill(-dt);  // = -I_3 * dt;
  }

  // Kalman gain
  Matrix<double, dSTATE_SIZE, 3> PHt = P_ * H.transpose();
  Matrix<double, dSTATE_SIZE, 3> K;
  if ((delayHandling_ == noMethod || delayHandling_ == applyUpdateToNew
       || delayHandling_ == larsonAverageIMU))
  {
    K = PHt * (H * PHt + meas_covariance).inverse();
  }
  if (delayHandling_ == larsonAverageIMU && !normalPass)
  {
    K = F_x_ * K;
  }
  // Correction error state
  Matrix<double, dSTATE_SIZE, 1> errorState = K * delta_measurement;
  // Update P (simple form)
  // P = (I_dx - K*H)*P;
  // Update P (Joseph form)
  Matrix<double, dSTATE_SIZE, dSTATE_SIZE> I_KH = I_dx - K * H;
  if (delayHandling_ == noMethod || delayHandling_ == applyUpdateToNew)
  {
    P_ = I_KH * P_ * I_KH.transpose() + K * meas_covariance * K.transpose();
  }
  if (delayHandling_ == larsonAverageIMU && !normalPass)
  {
    P_ = P_ - K * H * PHistoryPtr_->at(bestTimeIndex).second * F_x_;
  }

  injectErrorState(errorState);
}

ESKF::imuMeasurement ESKF::getAverageIMU(lTime stamp)
{
  Vector3d accelAcc(0, 0, 0);
  Vector3d gyroAcc(0, 0, 0);
  int complete = 0;
  int index = recentPtr;
  int count = 0;
  while (!complete)
  {
    if (imuHistoryPtr_->at(index % bufferL_).time >= stamp)
    {
      if (!imuHistoryPtr_->at(index % bufferL_).time.isZero())
      {
        // should acc
        accelAcc += imuHistoryPtr_->at(index % bufferL_).acc;
        gyroAcc += imuHistoryPtr_->at(index % bufferL_).gyro;
        count++;
      }
    }
    else
    {
      break;
    }
    index--;  // scroll back in time.
    if (index <= recentPtr - bufferL_)
      complete = 1;
  }
  accelAcc = accelAcc / count;
  gyroAcc = gyroAcc / count;
  ESKF::imuMeasurement ret;
  ret.acc = accelAcc;
  ret.gyro = gyroAcc;
  ret.time = imuHistoryPtr_->at(index % bufferL_).time;
  return ret;
}

void ESKF::injectErrorState(const Matrix<double, dSTATE_SIZE, 1>& error_state)
{  // Inject error state into nominal state (eqn 282, pg 62)
  nominalState_.block<3, 1>(POS_IDX, 0) += error_state.block<3, 1>(dPOS_IDX, 0);
  nominalState_.block<3, 1>(VEL_IDX, 0) += error_state.block<3, 1>(dVEL_IDX, 0);
  Vector3d dtheta = error_state.block<3, 1>(dTHETA_IDX, 0);
  Quaterniond q_dtheta = rotVecToQuat(dtheta);
  nominalState_.block<4, 1>(QUAT_IDX, 0) = quatToHamilton(getQuat() * q_dtheta).normalized();
  nominalState_.block<3, 1>(AB_IDX, 0) += error_state.block<3, 1>(dAB_IDX, 0);
  nominalState_.block<3, 1>(GB_IDX, 0) += error_state.block<3, 1>(dGB_IDX, 0);

  // Reflect this tranformation in the P matrix, aka ESKF Reset
  // Note that the document suggests that this step is optional
  // eqn 287, pg 63
  Matrix3d G_theta = I_3 - getSkew(0.5f * dtheta);
  P_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) =
    G_theta * P_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) * G_theta.transpose();
}
