#include "src/bot_software.h"

#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_ADS1X15.h>
#include <TaskScheduler.h>
#include <Servo.h>

#include "src/motor.h"
#include "src/drive.h"
#include "src/IMU.h"
#include "src/serialbuf.h"

Adafruit_NeoPixel stripB(LED_COUNT, PIN_NEO_BRD, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel stripX(1, PIN_NEO_XIAO, NEO_GRB + NEO_KHZ800);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_ADS1015 ads;

Motor FL(pwm, INA_FL, INB_FL, VREF_FL, NSLEEP_FL);
Motor FR(pwm, INA_FR, INB_FR, VREF_FR, NSLEEP_FR);
Motor RL(pwm, INA_RL, INB_RL, VREF_RL, NSLEEP_RL);
Motor RR(pwm, INA_RR, INB_RR, VREF_RR, NSLEEP_RR);

BotDrive drive;
BotMotion motion;
FlySkyIBus iBus;
Servo weaponOut;
SerialBuf serialBuf(Serial);

Scheduler ts0;
Scheduler ts1;

//core0
Task readRadio(0, TASK_FOREVER, &readRadioCB, &ts0);
Task mainTask(0, TASK_ONCE, &mainTaskCB, &ts0);
Task readAnalogue(IBUS_PERIOD * 4, TASK_FOREVER, &readAnalogueCB, &ts0);
Task readImu(IBUS_PERIOD, TASK_FOREVER, &readImuCB, &ts0);
Task writePWM(0, TASK_ONCE, &writePWMCB, &ts0);

//core1
Task inputHandler(0, TASK_ONCE, &inputHandlerCB, &ts1);
Task weaponHandler(0, TASK_ONCE, &weaponCB, &ts1);
Task rgb(10, TASK_FOREVER, &rgbCB, &ts1);
Task debugSerial(0, TASK_FOREVER, &debugSerialCB, &ts1);

Radio_t rxFrame;
Drive_t driveVals;
IMU_t imuData;
Pwr_t pwrData;

RadioState_t radioState;
BotState_t botState;
BattState_t battState;

void setup() {
  rp2040.idleOtherCore();

  Serial.begin();
  setupiBus();
  setupInputs();
  setupOutputs();
  setupDrive();
  motion.begin();
  setupADC();
  setupRGB();

  if (watchdog_enable_caused_reboot()) {
    botState = BOT_NOT_OK;
  }

  ts0.enableAll();
  ts1.enableAll();

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
    mainTask.restart();
  }

  if (millis() > (lastRadioTime + 100)) {
    lastRadioTime = millis();
    radioState = RADIO_DC;
    rxFrame = { 0 };
    mainTask.restart();
  }
}

void mainTaskCB() {
  if (radioState == RADIO_DC) {
    rxFrame = { 0 };
    drive.setInputs(rxFrame);
    drive.doMaths();
    writePWM.restart();
  } else {
    inputHandler.restart();
    drive.setIMU(imuData);
    drive.setInputs(rxFrame);
    drive.doMaths();
    writePWM.restart();
  }

  Msg_t msg;
  sprintf(msg.txt, "S:%d T:%d FL:%d FR:%d GY:", rxFrame.steering, rxFrame.throttle, driveVals.FL.duty, driveVals.FR.duty);
  Serial.println(imuData.g.z);
  serialBuf.pushMsg(msg);
}

void inputHandlerCB() {
  static bool lastButtVal;
  static int16_t lastSwVal;

  if (rxFrame.button != lastButtVal) {
    lastButtVal = rxFrame.button;
    weaponOut.writeMicroseconds(WEAPON_UP);
    weaponHandler.restartDelayed(500);
  }

  if (rxFrame.toggle != lastSwVal) {
    lastSwVal = rxFrame.toggle;
    weaponHandler.restart();
  }
}

void weaponCB() {
  weaponOut.writeMicroseconds(rxFrame.toggle);
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
}

void rgbCB() {
  if (radioState == RADIO_DC) {
    fillRGB(0, 0, 255);
  } else {
    if (battState == BATT_OK) {
      fillRainbow();
    } else if (botState == BOT_NOT_OK) {
      fillRGB(127, 127, 0);
    } else {
      fillRGB(255, 0, 0);
    }
  }
}

void fillRainbow() {
  static uint16_t hue;
  static uint32_t lastMillis;
  uint32_t currentMillis = millis();

  hue = hue + (currentMillis - lastMillis) * RGB_SPEED;

  stripB.rainbow(hue);
  stripX.rainbow(hue);
  stripX.show();
  stripB.show();

  lastMillis = currentMillis;
}

void fillRGB(uint8_t r, uint8_t g, uint8_t b) {
  stripB.fill(stripB.Color(r, g, b));
  stripX.fill(stripX.Color(g, r, b));
  stripB.show();
  stripX.show();
}

Radio_t parseChannels(uint16_t (&chanData)[IBUS_CHANNELS]) {
  Radio_t thisFrame;

  thisFrame.steering = map(chanData[0], 1050, 1950, MAX_PWM, -MAX_PWM);
  thisFrame.throttle = map(chanData[1], 1050, 1950, -MAX_PWM, MAX_PWM);
  thisFrame.button = map(chanData[2], 1000, 2000, 0, 1);
  thisFrame.toggle = chanData[3];
  thisFrame.knobL = map(chanData[4], 1000, 2000, 0, 100);
  thisFrame.knobR = map(chanData[5], 1000, 2000, 10, 200);

  return thisFrame;
}

//10mR shunt
//10mV/A
Pwr_t readPowerData() {
  Pwr_t pwr;

  pwr.v = readBattVolts();
  battState = getBattState();

  pwr.iFL = ads.computeVolts(ads.readADC_SingleEnded(0)) * 100;
  pwr.iFR = ads.computeVolts(ads.readADC_SingleEnded(2)) * 100;
  pwr.iRL = ads.computeVolts(ads.readADC_SingleEnded(1)) * 100;
  pwr.iRR = ads.computeVolts(ads.readADC_SingleEnded(3)) * 100;

  return pwr;
}

BattState_t getBattState() {
  float lowBattRst = LOW_BATT_VOLTS + 0.2;
  static bool lowBattEvent;

  if (pwrData.v < LOW_BATT_VOLTS) {
    lowBattEvent = true;
  }
  if (pwrData.v > lowBattRst) {
    lowBattEvent = false;
  }

  if (lowBattEvent) {
    return BATT_LOW;
  } else {
    return BATT_OK;
  }
}

//10K/2K2 voltage divider
//10V = 1.803V @ ADC
float readBattVolts() {
  float adc = analogRead(PIN_V_BATT);

  const float vRef = 3.3;
  const uint16_t fsr = 1023;

  adc = adc * (vRef / fsr);

  return adc * 5.57;  //(10/1.803v)+0.02 for calibration
}


void loop() {
  ts0.execute();
}

void loop1() {
  ts1.execute();
}
