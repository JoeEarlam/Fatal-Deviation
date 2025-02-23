#include "bot_software.h"

#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_ADS1X15.h>
#include <TaskScheduler.h>
#include <Servo.h>

#include "motor.h"
#include "drive.h"
#include "IMU.h"
#include "serialbuf.h"

Adafruit_NeoPixel stripB(LED_COUNT, PIN_NEO_BRD, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripX(1, PIN_NEO_XIAO, NEO_GRB + NEO_KHZ800);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_ADS1015 ads;

Motor FL(pwm, INA_FL, INB_FL, VREF_FL, NSLEEP_FL);
Motor FR(pwm, INA_FR, INB_FR, VREF_FR, NSLEEP_FR);
Motor RL(pwm, INA_RL, INB_RL, VREF_RL, NSLEEP_RL);
Motor RR(pwm, INA_RR, INB_RR, VREF_RR, NSLEEP_RR);

//SerialPIO softSer(3, SerialPIO::NOPIN); //extra serial for telemetry output

BotDrive drive;
BotMotion motion;
FlySkyIBus iBus;
Servo weaponOut;
SerialBuf serialBuf(Serial);

//core0
Task readRadio(0, TASK_FOREVER, &readRadioCB);
Task readAnalogue(IBUS_PERIOD * 4, TASK_FOREVER, &readAnalogueCB);
Task readImu(IBUS_PERIOD, TASK_FOREVER, &readImuCB);

Task driveMaths(0, TASK_ONCE, &driveMathsCB);
Task writePWM(0, TASK_ONCE, &writePWMCB);

Scheduler ts0;

//core1
Task rgb(10, TASK_FOREVER, &rgbCB);
Task debugSerial(0, TASK_FOREVER, &debugSerialCB);

Scheduler ts1;


Radio_t rxFrame;
Drive_t driveVals;
IMU_t imuData;
Pwr_t pwrData;

RadioState_t radioState;
BotState_t botState;

void setup() {
  rp2040.idleOtherCore();

  setupSerial();
  setupiBus();
  setupInputs();
  setupOutputs();
  setupDrive();
  motion.begin();
  setupADC();
  setupTasks();
  setupRGB();

  if (watchdog_enable_caused_reboot()) {
    botState = BOT_NOT_OK;
  }

  rp2040.wdt_begin(500);
  rp2040.resumeOtherCore();
}

void setup1() {
  delayMicroseconds(100);
}

void readRadioCB() {
  static uint32_t lastRadioTime;
  uint16_t chanData[IBUS_CHANNELS];

  iBus.loop();

  if (iBus.readChannels(chanData)) {
    lastRadioTime = millis();
    radioState = RADIO_OK;
    rxFrame = parseChannels(chanData);
    weaponOut.writeMicroseconds(rxFrame.button);
    driveMaths.restart();
  }

  if (millis() > (lastRadioTime + 100)) {
    lastRadioTime = millis();
    radioState = RADIO_DC;
    rxFrame = { 0 };
    driveMaths.restart();
  }
}

void driveMathsCB() {
  drive.setIMU(imuData);
  drive.setDriveAlgo(rxFrame.toggle);
  drive.setInputs(rxFrame);
  drive.doMaths();

  debugSerial.restart();
  writePWM.restart();
}

void writePWMCB() {
  driveVals = drive.getDrive();

  FL.setMotor(driveVals.FL);
  FR.setMotor(driveVals.FR);
  RL.setMotor(driveVals.RL);
  RR.setMotor(driveVals.RR);
  rp2040.wdt_reset();
}

void debugSerialCB() {
  serialBuf.sendBuffer();
}

void readImuCB() {
  motion.update();
  imuData = motion.getIMU();
}

void readAnalogueCB() {
  pwrData = readPowerData();

  if (pwrData.state == BATT_LOW) {
    FL.setCurrentLimit(10);
    FR.setCurrentLimit(10);
    RL.setCurrentLimit(10);
    RR.setCurrentLimit(10);
  }
}

void rgbCB() {
  static uint16_t hue;
  static uint32_t lastMillis;
  uint32_t currentMillis = millis();

  hue = hue + (currentMillis - lastMillis) * RGB_SPEED;

  if (radioState == RADIO_DC) {
    stripB.fill(stripB.Color(0, 0, 255));
    stripX.fill(stripX.Color(0, 0, 255));
  } else {
    if (pwrData.state == BATT_LOW) {
      stripB.fill(stripB.Color(255, 0, 0));
      stripX.fill(stripX.Color(255, 0, 0));
    } else {
      stripB.rainbow(hue);
      stripX.rainbow(hue);
    }
  }

  stripB.show();
  stripX.show();

  lastMillis = currentMillis;
}

Radio_t parseChannels(uint16_t (&chanData)[IBUS_CHANNELS]) {
  Radio_t thisFrame;

  thisFrame.steering = map(chanData[0], 1000, 2000, 100, -100);
  thisFrame.throttle = map(chanData[1], 1000, 2000, -100, 100);
  thisFrame.button = chanData[2];
  thisFrame.toggle = map(chanData[3], 1000, 2000, 0, 2);
  thisFrame.knobL = map(chanData[4], 1000, 2000, 0, 100);
  thisFrame.knobR = map(chanData[5], 1000, 2000, 0, 100);
  return thisFrame;
}

Pwr_t readPowerData() {
  Pwr_t pwr;

  pwr.v = readBattVolts();
  pwrData.state = getBattState();

  pwr.iFL = readShunt(MOT_FL);
  pwr.iFR = readShunt(MOT_FR);
  pwr.iRL = readShunt(MOT_RL);
  pwr.iRR = readShunt(MOT_RR);

  return pwr;
}

BattState_t getBattState() {
  static uint32_t lowBattMillis;

  if ((pwrData.v < LOW_BATT_VOLTS) && (lowBattMillis == 0)) {
    lowBattMillis = millis() + 2000;
  }

  if ((pwrData.v > LOW_BATT_VOLTS) && (millis() < lowBattMillis)) {
    lowBattMillis = 0;
  }

  if ((pwrData.v < LOW_BATT_VOLTS) && (millis() > lowBattMillis)) {
    return BATT_LOW;
  } else return BATT_OK;
}

//Fixme: needs cal
//10K/2K2 voltage divider
//10V = 1.803V @ ADC
float readBattVolts() {
  float adc = analogRead(PIN_V_BATT);
  const float vRef = 3.3;
  const uint16_t fsr = 1023;

  adc = adc * (vRef / fsr);

  return adc * 5.55;
}

//10mR shunt
//10mV/A
float readShunt(uint8_t shunt) {
  float adc;

  switch (shunt) {
    case MOT_FL:
      adc = ads.readADC_SingleEnded(0);
      break;
    case MOT_FR:
      adc = ads.readADC_SingleEnded(2);
      break;
    case MOT_RL:
      adc = ads.readADC_SingleEnded(1);
      break;
    case MOT_RR:
      adc = ads.readADC_SingleEnded(3);
      break;
    default:
      return 0;
  }

  adc = ads.computeVolts(adc) * 100;

  return adc;
}

void loop() {
  ts0.execute();
}

void loop1() {
  ts1.execute();
}
