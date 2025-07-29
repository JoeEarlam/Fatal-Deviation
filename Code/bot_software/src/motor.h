#ifndef MOTOR_H
#define MOTOR_H

#define MAX_PWM 4095

#include <Adafruit_PWMServoDriver.h>

struct MotorPWM_t {
  int16_t duty = 0;
  bool coast = false;
};

class Motor {
public:

  Motor(Adafruit_PWMServoDriver &pwm, uint16_t pinFwd, uint16_t pinRev, uint16_t pinLim, uint16_t pinEn);
  void begin();
  void setMotor(MotorPWM_t mot);
  void setCurrentLimit(uint16_t percent);

private:
  Adafruit_PWMServoDriver *_pwm;

  uint16_t _PIN_FWD;
  uint16_t _PIN_REV;
  uint16_t _PIN_ILIM;
  uint16_t _PIN_EN;

  int16_t lastDuty;
  uint32_t _lastSetTime;
  uint16_t _maxSlewRate = MAX_PWM * 10;  //pwms a second

  uint16_t _iLimitDuty = 4096;

  void _setPins(int16_t thisDuty);
};

Motor::Motor(Adafruit_PWMServoDriver &pwm, uint16_t pinFwd, uint16_t pinRev, uint16_t pinLim, uint16_t pinEn) {
  this->_pwm = &pwm;
  this->_PIN_FWD = pinFwd;
  this->_PIN_REV = pinRev;
  this->_PIN_ILIM = pinLim;
  this->_PIN_EN = pinEn;
}

void Motor::begin() {
  _pwm->setPin(_PIN_ILIM, _iLimitDuty);
  _pwm->setPin(_PIN_EN, 4096);
}

void Motor::setCurrentLimit(uint16_t percent) {
  if (percent > 100) return;
  _iLimitDuty = map(percent, 0, 100, 0, 4096);
  _pwm->setPin(_PIN_ILIM, _iLimitDuty);
}

void Motor::setMotor(MotorPWM_t mot) {
  int16_t thisDuty = mot.duty;
  thisDuty = constrain(thisDuty, -MAX_PWM, MAX_PWM);

  if (abs(thisDuty) > 0) {
    _pwm->setPin(_PIN_EN, 4096);
  }

  _setPins(thisDuty);

  if (mot.coast) {
    _pwm->setPin(_PIN_EN, 0);
  }
}

void Motor::_setPins(int16_t thisDuty) {
  uint32_t thisTime = millis();

  uint32_t timeDelta = thisTime - _lastSetTime;
  uint16_t dutySlew = _maxSlewRate / (1000 / timeDelta);

  if (abs(lastDuty - thisDuty) > dutySlew) {
    if (thisDuty > lastDuty) thisDuty = lastDuty + dutySlew;
    if (thisDuty < lastDuty) thisDuty = lastDuty - dutySlew;
  }

  thisDuty = constrain(thisDuty, -MAX_PWM, MAX_PWM);

  if (thisDuty > 0) {
    _pwm->setPin(_PIN_FWD, thisDuty);
    _pwm->setPin(_PIN_REV, 0);
  } else if (thisDuty < 0) {
    _pwm->setPin(_PIN_FWD, 0);
    _pwm->setPin(_PIN_REV, abs(thisDuty));
  } else {
    _pwm->setPin(_PIN_FWD, 0);
    _pwm->setPin(_PIN_REV, 0);
  }

  lastDuty = thisDuty;
  _lastSetTime = thisTime;
}

#endif