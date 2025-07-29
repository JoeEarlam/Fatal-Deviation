#ifndef DRIVE_H
#define DRIVE_H

#include "defines.h"
#include "types.h"
#include "motor.h"
#include "IMU.h"
#include <PID_v1.h>


struct Drive_t {
  MotorPWM_t FL, FR, RL, RR;
};

enum DriveState_t {
  IDLE,
  STATIONARY,
  MOVING,
};

enum DriveMode_t {
  TANK_OPENLOOP,
  TANK_CLOSEDLOOP,
  COMPUTE,
};

class BotDrive {
public:

  BotDrive();
  void begin();
  void setInputs(Radio_t inputs);
  void setIMU(IMU_t imuData);
  void setTunings(float p, float i);

  uint32_t doMaths();

  void setDriveAlgo(uint8_t algo);

  Drive_t getDrive();

private:
  DriveMode_t _driveMode = TANK_CLOSEDLOOP;
  DriveState_t _driveState;

  const uint16_t _deadZoneMov = MAX_PWM / 32;
  const uint16_t _deadZoneStat = MAX_PWM / 8;

  uint16_t _deadZoneThrot = _deadZoneMov;
  uint16_t _deadZoneSteer = _deadZoneStat;

  const float _maxGyroRads = 30;  //rate @ full send in rad/s

  Drive_t _driveData;
  IMU_t _imuData;

  int16_t _throttle;
  int16_t _steering;

  uint8_t _algoType;
  bool _enablePID = true;
  bool _enableExpThrot = true;

  double _PIDtgtRads, _PIDcurrRads, _PIDoutput;
  double _PID_Kp = 1, _PID_Ki = 0, _PID_Kd = 0;
  PID *_PID;

  int16_t _deadZoneInput(int16_t input, int16_t dz);
  float _runPID(float input);

  Drive_t _tankDrive(int16_t t, int16_t s);
  Drive_t _notForward(int16_t t, int16_t s);

  int16_t _expFunction(int16_t input, int16_t exp);
  float _steerToRads(float input);
  int16_t _radsToSteer(float input);
};

BotDrive::BotDrive() {
  _PID = new PID(&_PIDcurrRads, &_PIDoutput, &_PIDtgtRads, _PID_Kp, _PID_Ki, _PID_Kd, DIRECT);
}

void BotDrive::begin() {
  _PID->SetSampleTime(IBUS_PERIOD);
  _PID->SetOutputLimits(-_maxGyroRads, _maxGyroRads);
  _PID->SetMode(AUTOMATIC);
}

uint32_t BotDrive::doMaths() {
  uint32_t startTime = micros();

  int16_t t = _throttle;
  int16_t s = _steering;

  if (_enableExpThrot) t = _expFunction(t, 2);

  if (_driveState == IDLE) {
    _driveData.FL.coast = true;
    _driveData.FR.coast = true;
    _driveData.RL.coast = true;
    _driveData.RR.coast = true;
  } else {
    float pidOutRads;

    switch (_driveMode) {
      case TANK_OPENLOOP:
        _driveData = _tankDrive(t, s);
        break;
      case TANK_CLOSEDLOOP:
        pidOutRads = _runPID(_steerToRads(s));
        if (_enablePID && (_driveState == MOVING)) s = _radsToSteer(pidOutRads);

        _driveData = _tankDrive(t, s);
        break;
      case COMPUTE:
        _driveData = _tankDrive(t, s);  //bronk
        break;
    }
  }

  return startTime - micros();
}

float BotDrive::_runPID(float input) {
  _PIDtgtRads = input;
  _PIDcurrRads = _imuData.g.z;

  _PID->Compute();

  return _PIDoutput;
}



Drive_t BotDrive::_tankDrive(int16_t t, int16_t s) {
  Drive_t out;

  if (t > 0) {
    out.FL.duty = -s + t;
    out.FR.duty = s + t;

    out.FL.duty = constrain(out.FL.duty, -MAX_PWM, MAX_PWM);
    out.FR.duty = constrain(out.FR.duty, -MAX_PWM, MAX_PWM);

    if (out.FL.duty < 0) out.FL.duty = 0;
    else if (out.FL.duty <= (MAX_PWM / 8) && (out.FR.duty > MAX_PWM / 2)) {
      out.RL.duty = out.FR.duty / 8;
    } else {
      out.RL = out.FL;
    }

    if (out.FR.duty < 0) out.FR.duty = 0;
    else if (out.FR.duty <= (MAX_PWM / 8) && (out.FL.duty > MAX_PWM / 2)) {
      out.RR.duty = out.RL.duty / 8;
    } else {
      out.RR = out.FR;
    }

  } else {
    out = _notForward(t, s);
  }

  return out;
}

Drive_t BotDrive::_notForward(int16_t t, int16_t s) {
  Drive_t out;

  if (t > 0) {
    out.FL.duty = 0;
    out.FR.duty = 0;
    out.RL.duty = 0;
    out.RR.duty = 0;
  } else if (t < 0) {
    out.FL.duty = s + t;
    out.FR.duty = -s + t;

    out.FL.duty = constrain(out.FL.duty, -MAX_PWM, MAX_PWM);
    out.FR.duty = constrain(out.FR.duty, -MAX_PWM, MAX_PWM);

    if (out.FL.duty > 0) out.FL.duty = 0;
    if (out.FR.duty > 0) out.FR.duty = 0;

    out.RL = out.FL;
    out.RR = out.FR;

  } else {
    out.FL.duty = -s;
    out.FR.duty = s;
    out.FL.duty = constrain(out.FL.duty, -MAX_PWM, MAX_PWM);
    out.FR.duty = constrain(out.FR.duty, -MAX_PWM, MAX_PWM);
    out.RL = out.FL;
    out.RR = out.FR;
  }

  return out;
}


int16_t BotDrive::_expFunction(int16_t input, int16_t exp) {
  int32_t output = pow(input, exp) / pow(MAX_PWM + 1, exp - 1);

  if (output > (MAX_PWM - (MAX_PWM / 64))) output = MAX_PWM;
  if (output < (-MAX_PWM + (MAX_PWM / 64))) output = -MAX_PWM;

  if ((exp % 2 == 0) && (input < 0)) return int16_t(-output);
  else return int16_t(output);
}

float BotDrive::_steerToRads(float input) {
  const float scale = MAX_PWM / _maxGyroRads;
  float output = input / scale;

  return output;
}

int16_t BotDrive::_radsToSteer(float input) {
  const float scale = MAX_PWM / _maxGyroRads;
  float output = input * scale;

  return output;
}

void BotDrive::setInputs(Radio_t inputs) {
  static uint32_t lastActiveTime;
  _steering = _deadZoneInput(inputs.steering, _deadZoneSteer);
  _throttle = _deadZoneInput(inputs.throttle, _deadZoneThrot);

  if ((_steering != 0) || (_throttle != 0)) {
    lastActiveTime = millis();
    _deadZoneSteer = _deadZoneMov;
    _driveState = MOVING;
  }

  if ((_steering == 0) && (_throttle == 0)) {
    _driveState = STATIONARY;
  }

  if (millis() > lastActiveTime + 2000) {
    _deadZoneSteer = _deadZoneStat;
    _driveState = IDLE;
  }
}

int16_t BotDrive::_deadZoneInput(int16_t input, int16_t dz) {
  input = constrain(input, -MAX_PWM, MAX_PWM);

  if (dz == 0) return input;
  else {
    if (abs(input) < dz) {
      return 0;
    } else {
      int16_t scaledOutput = map(abs(input), dz, MAX_PWM, 0, MAX_PWM);
      if (input > 0) input = scaledOutput;
      if (input < 0) input = -scaledOutput;
      return input;
    }
  }
}

void BotDrive::setTunings(float p, float i) {
  _PID_Kp = p;
  _PID_Ki = i;

  if (_PID_Kp < 0) _PID_Kp = 0.1;
  if (_PID_Ki < 0) _PID_Ki = 0;
  _PID->SetTunings(_PID_Kp, _PID_Ki, _PID_Kd);
}

void BotDrive::setDriveAlgo(uint8_t algo) {
  _algoType = algo;
}


void BotDrive::setIMU(IMU_t imuData) {
  _imuData = imuData;
}

Drive_t BotDrive::getDrive() {
  return _driveData;
}


#endif