#include <Arduino.h>
#include "include/ui.h"
#include "include/pins.h"
#include "include/sensors.h"
#include "include/core.h"
#include "include/config.h"
#include "include/maps.h"
#include "include/input.h"
#include "include/serial_config.h"

#include <SoftTimer.h>
#include <AutoPID.h>

const double speedoKp = 1; //80,21 Pid Proporional Gain. Initial ramp up i.e Spool, Lower if over boost
double speedoKi = .0001;   //40,7 Pid Integral Gain. Overall change while near Target Boost, higher value means less change, possible boost spikes
const double speedoKd = 0; //100, 1 Pid Derivative Gain.
double pidSpeedo, speedoPWM, pidSpeedoLim;
int updateCount;

//AutoPID speedoPID(&pidSpeedoLim, &pidSpeedo, &speedoPWM, 0, 255, speedoKp, speedoKi, speedoKd);

boolean infoBoost = false;

void updateDisplay(Task *me)
{
  digitalToggleFast(LED_BUILTIN);
  
  if (w124rpm)
  {
    rpmMeterUpdate();
  }
  if (w124speedo)
  {
    updateSpeedo();
  }
}

void updateSpeedo()
{
  struct SensorVals sensor = readSensors();
  //pidSpeedo = double(sensor.curSpeed) * 0.70;
  //pidSpeedoLim = double(speedoRPM) / 6;
  // speedoPID.setBangBang(1);
  //speedoPID.setTimeStep(200);
  //speedoPID.run();
  
  int speedPWM = map(sensor.curSpeed, 0, 255, 0, 255);
  analogWrite(speedoCtrl, speedPWM); // Wake up speedometer motor so it wont stick
}

void rpmMeterUpdate()
{
  struct SensorVals sensor = readSensors();

  int rpmPWM = map(sensor.curRPM, 0, config.maxRPM, 0, 255);
  analogWrite(rpmMeter, rpmPWM);
}
void datalog(Task *me)
{
  static long counter = 0;

  static uint32_t timerLog = 0;
  if (datalogger)
  {
    struct SensorVals sensor = readSensors();

    // if (debugEnabled)
    // {
    //   debugEnabled = false;
    // }
    if (millis() < timerLog)
    {
      return;
    }
    timerLog = millis() + 1000;
    Serial.print(counter);

    Serial.print(F(";"));
    Serial.print(sensor.curSpeed);
    Serial.print(F(";"));
    Serial.print(sensor.curRPM);
    Serial.print(F(";"));
    Serial.print(sensor.curTps);
    Serial.print(F(";"));
    Serial.print(gear);
    Serial.print(F(";"));
    Serial.print(sensor.curOilTemp);
    Serial.print(F(";"));
    Serial.print(sensor.curAtfTemp);
    Serial.print(F(";"));
    Serial.print(sensor.curLoad);
    Serial.print(F(";"));
    Serial.print(sensor.curBoost);
    Serial.print(F(";"));
    Serial.print(sensor.curExPres);
    Serial.print(F(";"));
    Serial.print(sensor.curBoostLim);
    Serial.print(F(";"));
    Serial.print(sensor.curPresDiff);
    Serial.print(F(";"));
    Serial.print(n2Speed);
    Serial.print(F(";"));
    Serial.print(n3Speed);
    Serial.print(F(";"));
    Serial.print(sensor.curEvalGear);
    Serial.print(F(";"));
    Serial.print(sensor.curRatio);
    Serial.print(F(";"));
    Serial.print(sensor.curSlip);
    Serial.print(F(";"));
    Serial.print(sensor.curBattery);
    Serial.print(F(";"));
    Serial.print(boostPWM);
    Serial.print(F(";"));
    Serial.print(wantedGear);
    Serial.print(F(";"));
    Serial.println(sensor.curExTemp);
  }
  counter++;
}
