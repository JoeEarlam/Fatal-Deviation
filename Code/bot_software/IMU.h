#ifndef IMU_H
#define IMU_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

struct Acc_t {
  float x, y, z;
};

struct Gyr_t {
  float x, y, z;
};

struct IMU_t {
  uint32_t time;
  Acc_t a;
  Gyr_t g;
};

class BotMotion {
public:
  BotMotion();
  bool begin();
  void recalibrate();
  void update();
  IMU_t getIMU();

private:

  Adafruit_MPU6050 _MPU;
  IMU_t _imuData;
  Gyr_t _gyrCal;
  int16_t _temp;

  void _filter(IMU_t &filterIn);
};

BotMotion::BotMotion() {
}

bool BotMotion::begin() {
  if (!_MPU.begin()) {
    return false;
  }

  _MPU.setAccelerometerRange(MPU6050_RANGE_16_G);
  _MPU.setGyroRange(MPU6050_RANGE_2000_DEG);
  _MPU.setFilterBandwidth(MPU6050_BAND_94_HZ);  //about 2/3 RC packet freq

  recalibrate();

  return true;
}

//fixme: doesn't work idk
//ripped from https://github.com/jremington/MPU-6050-Fusion/blob/main/MPU6050_Mahony_calgyro.ino
void BotMotion::_filter(IMU_t &filterIn) {
  static float Kp = 30;                       //filter gain
  static float Ki = 0;                        //filter intergral
  static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms

  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  //error terms
  float qa, qb, qc;
  static float q[4] = { 1.0, 0.0, 0.0, 0.0 }; //mahoney filter quaternisation
  
  static float last;
  float now = filterIn.time;
  float deltat = (now - last) * 1.0e-6;  //seconds since last update
  last = now;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  float cumAccel = filterIn.a.x * filterIn.a.x + filterIn.a.y * filterIn.a.y + filterIn.a.z * filterIn.a.z;

  // ignore accelerometer if false (tested OK, SJR)
  if (cumAccel > 0.0) {

    // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
    recipNorm = 1.0 / sqrt(cumAccel);
    filterIn.a.x *= recipNorm;
    filterIn.a.y *= recipNorm;
    filterIn.a.z *= recipNorm;

    // Estimated direction of gravity in the body frame (factor of two divided out)
    vx = q[1] * q[3] - q[0] * q[2];
    vy = q[0] * q[1] + q[2] * q[3];
    vz = q[0] * q[0] - 0.5f + q[3] * q[3];

    // Error is cross product between estimated and measured direction of gravity in body frame
    // (half the actual magnitude)
    ex = (filterIn.a.y * vz - filterIn.a.z * vy);
    ey = (filterIn.a.z * vx - filterIn.a.x * vz);
    ez = (filterIn.a.x * vy - filterIn.a.y * vx);

    // Compute and apply to gyro term the integral feedback, if enabled
    if (Ki > 0.0f) {
      ix += Ki * ex * deltat;  // integral error scaled by Ki
      iy += Ki * ey * deltat;
      iz += Ki * ez * deltat;
      filterIn.g.x += ix;  // apply integral feedback
      filterIn.g.y += iy;
      filterIn.g.z += iz;
    }

    // Apply proportional feedback to gyro term
    filterIn.g.x += Kp * ex;
    filterIn.g.y += Kp * ey;
    filterIn.g.z += Kp * ez;
  }

  // Integrate rate of change of quaternion, given by gyro term
  // rate of change = current orientation quaternion (qmult) gyro rate

  deltat = 0.5 * deltat;
  filterIn.g.x *= deltat;  // pre-multiply common factors
  filterIn.g.y *= deltat;
  filterIn.g.z *= deltat;
  qa = q[0];
  qb = q[1];
  qc = q[2];

  //add qmult*delta_t to current orientation
  q[0] += (-qb * filterIn.g.x - qc * filterIn.g.y - q[3] * filterIn.g.z);
  q[1] += (qa * filterIn.g.x + qc * filterIn.g.z - q[3] * filterIn.g.y);
  q[2] += (qa * filterIn.g.y - qb * filterIn.g.z + q[3] * filterIn.g.x);
  q[3] += (qa * filterIn.g.z + qb * filterIn.g.y - qc * filterIn.g.x);

  // Normalise quaternion
  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] = q[0] * recipNorm;
  q[1] = q[1] * recipNorm;
  q[2] = q[2] * recipNorm;
  q[3] = q[3] * recipNorm;
}

void BotMotion::recalibrate() {

}

void BotMotion::update() {
  sensors_event_t readA, readG, readT;
  IMU_t newData;
  _MPU.getEvent(&readA, &readG, &readT);
  newData.time = micros();
  newData.a.x = readA.acceleration.x;
  newData.a.y = readA.acceleration.y;
  newData.a.z = readA.acceleration.z;
  newData.g.x = readG.gyro.x;
  newData.g.y = readG.gyro.y;
  newData.g.z = readG.gyro.z;
  _temp = readT.temperature;

  //_filter(newData);
  _imuData = newData;
}

IMU_t BotMotion::getIMU() {
  return _imuData;
}

#endif