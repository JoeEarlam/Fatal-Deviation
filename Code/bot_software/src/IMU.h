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
  void update();
  IMU_t getIMU();

private:

  Adafruit_MPU6050 _MPU;
  IMU_t _imuData;
  Gyr_t _gyrCal;
  int16_t _temp;

};

BotMotion::BotMotion() {
}

bool BotMotion::begin() {
  if (!_MPU.begin()) {
    return false;
  }

  _MPU.setAccelerometerRange(MPU6050_RANGE_16_G);
  _MPU.setGyroRange(MPU6050_RANGE_2000_DEG);
  
  return true;
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

  _imuData = newData;
}

IMU_t BotMotion::getIMU() {
  return _imuData;
}

#endif