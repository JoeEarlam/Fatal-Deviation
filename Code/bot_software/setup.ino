void setupSerial() {
  Serial.begin();
  //softSer.begin(115200);
}

void setupInputs() {
  analogReadResolution(10);
  pinMode(PIN_DRV_FAULT, INPUT);
}

void setupOutputs() {
  weaponOut.attach(PIN_WEAP);
  weaponOut.writeMicroseconds(1500);

  pinMode(PIN_PWM_EN, OUTPUT);
  digitalWrite(PIN_PWM_EN, LOW);
}

void setupiBus() {
  Serial1.setFIFOSize(64);  //default 32 is too small
  Serial1.begin(115200);
  iBus.begin(Serial1);
}

void setupADC() {
  if (!ads.begin()) {
    botState = BOT_NOT_OK;
  } else {
    ads.setGain(GAIN_SIXTEEN);
  }
}

void setupDrive() {
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(1600);
  Wire.setClock(400000);

  drive.begin();
  FL.begin();
  FR.begin();
  RL.begin();
  RR.begin();
}

void setupTasks() {
  ts0.addTask(readRadio);
  ts0.addTask(writePWM);
  ts0.addTask(driveMaths);
  ts0.addTask(readAnalogue);
  ts0.addTask(readImu);

  readRadio.enable();
  driveMaths.enable();
  writePWM.enable();
  readAnalogue.enable();
  readImu.enable();

  ts1.addTask(debugSerial);
  ts1.addTask(rgb);

  rgb.enable();
  debugSerial.enable();
}

void setupRGB() {
  pinMode(PIN_NEO_PWR, OUTPUT);
  digitalWrite(PIN_NEO_PWR, HIGH);
  delay(1);

  stripX.begin();
  stripX.show();
  stripX.setBrightness(16);

  stripB.begin();
  stripB.show();
  stripB.setBrightness(16);
}