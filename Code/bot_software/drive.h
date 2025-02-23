#ifndef DRIVE_H
#define DRIVE_H


#include "defines.h"
#include "types.h"
#include "motor.h"
#include "IMU.h"
#include <PID_v1.h>

#define MOT_FL 0
#define MOT_FR 1
#define MOT_RL 2
#define MOT_RR 3

struct Drive_t {
  MotorPWM_t FL, FR, RL, RR;
};

class BotDrive {
public:

  BotDrive();
  void begin();
  void setInputs(Radio_t inputs);
  void setIMU(IMU_t imuData);

  uint32_t doMaths();
  bool newDriveData();

  void setDriveAlgo(uint8_t algo);
  void setTunings(float p, float i);

  Drive_t getDrive();

  uint16_t getPIDout();

private:

  const uint16_t _deadZone = 5;

  //base unit mm
  const double _wheelBase = 123;
  const double _trackWidth = 112;

  Radio_t _inputs;
  Drive_t _driveData;
  IMU_t _imuData;

  uint8_t _algoType;

  bool _newDriveData;

  double _PIDtgtRads, _PIDcurrRads, _PIDsteeringOut;
  double _PID_Kp = 10, _PID_Ki = 0, _PID_Kd = 0;
  PID *_PID;

  Drive_t _tankDrive(Radio_t in);
  Drive_t _tankDriveExp(Radio_t in);
  Drive_t _tankDriveGyro(Radio_t in);

  void _scale(int16_t &a, int16_t &b);
  void _scale(int16_t &a, int16_t &b, int16_t &c, int16_t &d);
  int16_t _deadZoneInput(int16_t input);
  int16_t _runPID(int16_t input);
};


BotDrive::BotDrive() {
  _PID = new PID(&_PIDcurrRads, &_PIDsteeringOut, &_PIDtgtRads, _PID_Kp, _PID_Ki, _PID_Kd, DIRECT);
}

void BotDrive::begin() {
  _PID->SetSampleTime(IBUS_PERIOD);
  _PID->SetOutputLimits(-100, 100);
  _PID->SetMode(AUTOMATIC);
}

void BotDrive::setTunings(float p, float i) {
  _PID_Kp = p;
  _PID_Ki = i;

  if (_PID_Kp < 0) _PID_Kp = 0.1;
  if (_PID_Ki < 0) _PID_Ki = 0;
  _PID->SetTunings(_PID_Kp, _PID_Ki, _PID_Kd);
}

uint32_t BotDrive::doMaths() {
  uint32_t startTime = micros();
  switch (_algoType) {
    case 0:
      _driveData = _tankDriveExp(_inputs);
      break;
    case 1:
      _driveData = _tankDriveGyro(_inputs);
      break;
    case 2:
      _driveData = _tankDriveGyro(_inputs);
      break;
    default:
      _driveData = _tankDriveExp(_inputs);
      break;
  }
  _newDriveData = true;
  return startTime - micros();
}

void BotDrive::setDriveAlgo(uint8_t algo) {
  _algoType = algo;
}

int16_t BotDrive::_runPID(int16_t input) {
  _PIDcurrRads = _imuData.g.z;
  _PIDtgtRads = input;

  _PID->Compute();
  return (_deadZoneInput(_PIDsteeringOut));
}

Drive_t BotDrive::_tankDriveExp(Radio_t in) {
  Drive_t out;

  if (abs(in.steering) > 0) in.steering = pow(in.steering, 3) / 10000;

  if (in.throttle > 0) {
    out.FL.duty = -in.steering + in.throttle;
    out.FR.duty = in.steering + in.throttle;
    if (out.FL.duty < 0) out.FL.duty = 0;
    if (out.FR.duty < 0) out.FR.duty = 0;
  } else if (in.throttle < 0) {
    out.FL.duty = in.steering + in.throttle;
    out.FR.duty = -in.steering + in.throttle;
  } else {
    out.FL.duty = -in.steering + in.throttle;
    out.FR.duty = in.steering + in.throttle;
  }

  _scale(out.FL.duty, out.FR.duty);

  out.RL = out.FL;
  out.RR = out.FR;

  return out;
}

Drive_t BotDrive::_tankDriveGyro(Radio_t in) {
  Drive_t out;

  if (abs(in.steering) > 0) in.steering = pow(in.steering, 3) / 10000;

  int16_t pidSteering = _runPID(in.steering);

  if (in.throttle > 0) {
    out.FL.duty = -pidSteering + in.throttle;
    out.FR.duty = pidSteering + in.throttle;
    _scale(out.FL.duty, out.FR.duty);

    if (out.FL.duty < 5) {
      out.RL.coast = 1;
    } else out.RL = out.FL;

    if (out.FR.duty < 5) {
      out.RR.coast = 1;
    } else out.RR = out.FR;

  } else if (in.throttle < 0) {
    out.FL.duty = in.steering + in.throttle;
    out.FR.duty = -in.steering + in.throttle;
    _scale(out.FL.duty, out.FR.duty);
    out.RL = out.FL;
    out.RR = out.FR;
  } else {
    out.FL.duty = -in.steering + in.throttle;
    out.FR.duty = in.steering + in.throttle;
    out.RL = out.FL;
    out.RR = out.FR;
  }

  return out;
}

void BotDrive::_scale(int16_t &a, int16_t &b) {
  int16_t absA = abs(a);
  int16_t absB = abs(b);
  int16_t max = 0;

  if ((absA < 100) && (absB < 100)) return;
  else {
    if (absA > max) max = absA;
    if (absB > max) max = absB;

    double sf = 100 / double(max);

    a = a * sf;
    b = b * sf;
  }
}

void BotDrive::_scale(int16_t &a, int16_t &b, int16_t &c, int16_t &d) {
  int max = 0;
  int vals[4] = { abs(a), abs(b), abs(c), abs(d) };

  for (uint8_t i = 0; i < 4; i++) {
    if (vals[i] > max) max = vals[i];
  }

  if (max > 100) {
    double sf = max / 100;
    a = a / sf;
    b = b / sf;
    c = c / sf;
    d = d / sf;
  }
}

void BotDrive::setInputs(Radio_t inputs) {
  _inputs = inputs;
  _inputs.steering = _deadZoneInput(_inputs.steering);
  _inputs.throttle = _deadZoneInput(_inputs.throttle);
}

int16_t BotDrive::_deadZoneInput(int16_t input) {
  if (abs(input) < _deadZone) {
    return 0;
  } else return input;
}

void BotDrive::setIMU(IMU_t imuData) {
  _imuData = imuData;
}

Drive_t BotDrive::getDrive() {
  return _driveData;
}

/*
  True if new maths is ready
*/
bool BotDrive::newDriveData() {
  if (_newDriveData == true) {
    _newDriveData = false;
    return true;
  } else return false;
}

uint16_t BotDrive::getPIDout() {
  return uint16_t(_PIDsteeringOut);
}

#endif