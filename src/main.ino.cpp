# 1 "C:\\Users\\Dev\\AppData\\Local\\Temp\\tmp1_36no3w"
#include <Arduino.h>
# 1 "D:/Code/Arduino/ATC-722.6/src/main.ino"
# 30 "D:/Code/Arduino/ATC-722.6/src/main.ino"
#include <Arduino.h>
#include "include/pins.h"
#include "include/sensors.h"
#include "include/core.h"
#include "include/input.h"
#include "include/ui.h"
#include "include/serial_config.h"
#include "include/config.h"

#include <EEPROM.h>
#include <SoftTimer.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <AutoPID.h>

#define DISPLAYTYPE1 


Task pollDisplay(200, updateDisplay);
Task pollData(33, datalog);
Task pollStick(100, pollstick);
Task pollGear(200, decideGear);
Task pollSensors(80, pollsensors);
Task pollTrans(50, polltrans);
Task pollFuelControl(1000, fuelControl);
Task pollBoostControl(100, boostControl);

Task pollSerialWatch(100, serialWatch);
Task keypadPressWatch(100, keypadWatch);

#ifdef ECU
Task pollInjectionControl(100, injectionControl);
#endif

#define NEXTION 

#ifdef NEXTION
#include "include/nextion.h"
Nextion screen = Nextion();

void fastRefreshScreen(Task *me);
Task pollNextionFast(100, fastRefreshScreen);

void averageRefreshScreen(Task *me);
Task pollNextionAverage(500, averageRefreshScreen);

void slowRefreshScreen(Task *me);
Task pollNextionSlow(1000, slowRefreshScreen);

int realRPM = 0;
int realATF = 0;
int realLOAD = 0;
int realTPS = 0;
int realRPM2 = 0;
int realATF2 = 0;
int realtps_Bar = 0;
int realload_Bar = 0;


void fastRefreshScreen(Task *me)
{

  struct SensorVals sensor = readSensors();


  screen.setText("rpmValue", sensor.curRPM);
  realRPM = screen.mapInt(sensor.curRPM, 0, 7000, 0, 253);
  realRPM = realRPM < 0 ? 360 + realRPM : realRPM;
  screen.setVal("rpmGauge", realRPM);

  screen.setText("atfValue", sensor.curAtfTemp);
  realATF = screen.mapInt(sensor.curAtfTemp, 0, 140, 0, 253);
  realATF = realATF < 0 ? 360 + realATF : realATF;
  screen.setVal("atfGauge", realATF);

  screen.setText("loadValue", sensor.curLoad);
  realLOAD = screen.mapInt(sensor.curLoad, 0, 100, 0, 277);
  realLOAD = realLOAD < 0 ? 360 + realLOAD : realLOAD;
  screen.setVal("loadGauge", realLOAD);

  screen.setText("tpsValue", sensor.curTps);
  realTPS = screen.mapInt(sensor.curTps, 0, 100, 0, 277);
  realTPS = realTPS < 0 ? 360 + realTPS : realTPS;
  screen.setVal("tpsGauge", realTPS);

  screen.setText("ratioValue", String(config.transferRatio));


  screen.setText("n2Value", String(n2Speed));

  screen.setText("n3Value", String(n3Speed));

  realRPM2 = screen.mapInt(sensor.curRPM, 0, 7000, 0, 252);
  realRPM2 = realRPM2 < 0 ? 360 + realRPM2 : realRPM2;
  screen.setVal("rpmGauge2", realRPM2);

  realATF2 = screen.mapInt(sensor.curAtfTemp, 140, 0, 90, 270);
  realATF2 = realATF2 < 0 ? 360 + realATF2 : realATF2;
  screen.setVal("atfGauge2", realATF2);
# 183 "D:/Code/Arduino/ATC-722.6/src/main.ino"
}

void averageRefreshScreen(Task *me)
{

  struct SensorVals sensor = readSensors();
# 229 "D:/Code/Arduino/ATC-722.6/src/main.ino"
  screen.setPic("gear", wantedGear == 8 ? 3 : wantedGear == 7 ? 4 : (wantedGear < 6 && wantedGear > 0) ? 6 : 3);

  screen.setPic("curGear", gear == 5 ? 11 : gear == 4 ? 10 : gear == 3 ? 9 : gear == 2 ? 8 : gear == 1 ? 7 : 8);

  screen.setText("gear_number", String(gear));

  screen.setText("speed_val", String(sensor.curSpeed));

  screen.setText("p1to2_val", String(config.oneTotwo));

  screen.setText("p2to3_val", String(config.twoTothree));

  screen.setText("p3to4_val", String(config.threeTofour));

  screen.setText("p4to5_val", String(config.fourTofive));

  screen.setText("p5to4_val", String(config.fiveTofour));

  screen.setText("p4to3_val", String(config.fourTothree));

  screen.setText("p3to2_val", String(config.threeTotwo));

  screen.setText("p2to1_val", String(config.twoToone));

screen.setText("tps_Bar", sensor.curTps);
int realtps_Bar = screen.mapInt(sensor.curTps, 0, 100, 0, 100);
screen.setVal("tps_Bar", realtps_Bar);

screen.setText("load_Bar", sensor.curLoad);
int realload_Bar = screen.mapInt(sensor.curLoad, 0, 100, 0, 100);
screen.setVal("load_Bar", realload_Bar);
# 279 "D:/Code/Arduino/ATC-722.6/src/main.ino"
}

void slowRefreshScreen(Task *me)
{
  struct SensorVals sensor = readSensors();

  screen.setText("bat_value", String((int)(sensor.curBattery / 1000)) + "." + String((int)(sensor.curBattery % 1000) / 10));
  int realBatt = screen.mapInt(sensor.curBattery, 11000, 16000, 0, 100);
  screen.setVal("bat_value", realBatt);

}

#endif
void setup();
#line 293 "D:/Code/Arduino/ATC-722.6/src/main.ino"
void setup()
{
  delay(5000);

  initConfig();





  analogWriteFrequency(spc, 1000);
  analogWriteFrequency(mpc, 1000);
  analogWriteFrequency(boostCtrl, 30);
  analogWriteFrequency(rpmMeter, 50);

  Serial.begin(115200);

  if (radioEnabled)
  {
    Serial1.begin(9600);
    if (debugEnabled && !datalogger)
    {
      Serial.println("Radio initialized.");
    }
  }

#ifdef DISPLAYTYPE1
  U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, 10, 17, 5);
#else
  U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, 10, 17, 5);
#endif
  u8g2.begin();


  pinMode(y3, OUTPUT);
  pinMode(y4, OUTPUT);
  pinMode(y5, OUTPUT);
  pinMode(spc, OUTPUT);
  pinMode(mpc, OUTPUT);
  pinMode(tcc, OUTPUT);
  pinMode(rpmMeter, OUTPUT);
  pinMode(boostCtrl, OUTPUT);
  pinMode(speedoCtrl, OUTPUT);
  pinMode(fuelPumpCtrl, OUTPUT);
  pinMode(hornPin, OUTPUT);
  pinMode(reversePin, OUTPUT);


  pinMode(boostPin, INPUT);
  pinMode(tpsPin, INPUT);
  pinMode(oilPin, INPUT);
  pinMode(atfPin, INPUT);
  pinMode(n2pin, INPUT_PULLUP);
  pinMode(n3pin, INPUT_PULLUP);
  pinMode(speedPin, INPUT);
  pinMode(rpmPin, INPUT);
  pinMode(batteryPin, INPUT);
  pinMode(keypadPin, INPUT);
  pinMode(lowGearPin, INPUT);


  *portConfigRegister(boostCtrl) = PORT_PCR_MUX(1) | PORT_PCR_PE;




  *portConfigRegister(speedPin) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  *portConfigRegister(rpmPin) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  *portConfigRegister(hornPin) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  *portConfigRegister(reversePin) = PORT_PCR_MUX(1) | PORT_PCR_PE;

  pinMode(autoSwitch, INPUT);

  if (!resistiveStick)
  {
    pinMode(gupSwitch, INPUT);
    pinMode(gdownSwitch, INPUT);
    *portConfigRegister(gupSwitch) = PORT_PCR_MUX(1) | PORT_PCR_PE;
    *portConfigRegister(gdownSwitch) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  }
  else
  {
    pinMode(gupSwitchalt, INPUT_PULLUP);
    pinMode(gdownSwitch, INPUT_PULLUP);
  }

  pinMode(fuelInPin, INPUT);

  *portConfigRegister(fuelInPin) = PORT_PCR_MUX(1) | PORT_PCR_PE;



  *portConfigRegister(autoSwitch) = PORT_PCR_MUX(1) | PORT_PCR_PE;


  pinMode(whitepin, INPUT);
  pinMode(bluepin, INPUT);
  pinMode(greenpin, INPUT);
  pinMode(yellowpin, INPUT);

  *portConfigRegister(whitepin) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  *portConfigRegister(bluepin) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  *portConfigRegister(greenpin) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  *portConfigRegister(yellowpin) = PORT_PCR_MUX(1) | PORT_PCR_PE;







  pinMode(exhaustPresPin, INPUT);
  pinMode(exhaustTempPin, INPUT);





  analogWrite(y3, 255);
  analogWrite(y4, 0);
  analogWrite(y5, 0);
  analogWrite(spc, 0);
  analogWrite(mpc, 0);
  analogWrite(tcc, 0);
  analogWrite(speedoCtrl, 0);



  if (rpmSpeed && fuelPumpControl)
  {
    analogWrite(fuelPumpCtrl, 255);
  }

  digitalWrite(rpmPin, HIGH);

  attachInterrupt(digitalPinToInterrupt(n2pin), N2SpeedInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(n3pin), N3SpeedInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(speedPin), vehicleSpeedInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(rpmPin), rpmInterrupt, RISING);







  if (debugEnabled && !datalogger)
  {
    Serial.println(F("Started."));
  }


  SoftTimer.add(&pollDisplay);
  SoftTimer.add(&pollData);
  SoftTimer.add(&pollStick);
  SoftTimer.add(&pollGear);
  SoftTimer.add(&pollSensors);
  SoftTimer.add(&pollTrans);
  SoftTimer.add(&pollFuelControl);
  SoftTimer.add(&pollBoostControl);
  SoftTimer.add(&pollSerialWatch);
  SoftTimer.add(&keypadPressWatch);

#ifdef ECU
  SoftTimer.add(&pollInjectionControl);
#endif

#ifdef NEXTION

  Serial1.begin(115200);
  screen.setupScreen(Serial1);
  screen.setPage("0");
  screen.setDim(100);

  SoftTimer.add(&pollNextionFast);
  SoftTimer.add(&pollNextionAverage);
  SoftTimer.add(&pollNextionSlow);

#endif
}