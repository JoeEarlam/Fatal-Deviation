void setupInputs() {
  analogReadResolution(10);
  pinMode(PIN_DRV_FAULT, INPUT);
}

void setupOutputs() {
  weaponOut.attach(PIN_WEAP);
  weaponOut.writeMicroseconds(WEAPON_DN);

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
  pwm.setPWMFreq(1000);
  Wire.setClock(400000);

  drive.begin();
  FL.begin();
  FR.begin();
  RL.begin();
  RR.begin();

  uint16_t iLim = 90;
  FL.setCurrentLimit(iLim);
  FR.setCurrentLimit(iLim);
  RL.setCurrentLimit(iLim);
  RR.setCurrentLimit(iLim);
}

void setupRGB() {
  pinMode(PIN_NEO_PWR, OUTPUT);
  digitalWrite(PIN_NEO_PWR, HIGH);
  delay(1);

  stripX.begin();
  stripX.show();
  stripX.setBrightness(127);

  stripB.begin();
  stripB.show();
  stripB.setBrightness(127);
}