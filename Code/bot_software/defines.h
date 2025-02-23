#ifndef DEFINES_H
#define DEFINES_H

//pins
#define PIN_WEAP 0
#define PIN_IBUS 1
#define PIN_DRV_FAULT 2
#define PIN_NEO_BRD 27
#define PIN_NEO_PWR 11
#define PIN_NEO_XIAO 12
#define PIN_PWM_EN 28
#define PIN_V_BATT A3

//PWM output channels
#define NSLEEP_FL 0
#define INA_FL 1
#define INB_FL 2
#define VREF_FL 3

#define NSLEEP_RL 4
#define INA_RL 5
#define INB_RL 6
#define VREF_RL 7

#define NSLEEP_RR 8
#define INA_RR 10
#define INB_RR 9
#define VREF_RR 11

#define NSLEEP_FR 12
#define INA_FR 14
#define INB_FR 13
#define VREF_FR 15

//misc
#define LED_COUNT 4
#define RGB_SPEED 16
#define IBUS_PERIOD 7
#define LOW_BATT_VOLTS 9.5

#endif