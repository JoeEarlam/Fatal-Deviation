#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

enum RadioState_t {
  RADIO_DC,
  RADIO_OK,
};

enum BattState_t {
  BATT_OK,
  BATT_LOW,
};

enum BotState_t {
  BOT_OK,
  BOT_NOT_OK,
};

struct Radio_t {
  int16_t steering;
  int16_t throttle;
  int16_t button;
  int16_t toggle;
  int16_t knobL;
  int16_t knobR;
};

struct Pwr_t {
  float iFL, iFR, iRL, iRR;
  float v;

  float watts = (iFL + iFR + iRL + iRR) * v;
};

#endif