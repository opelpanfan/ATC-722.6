
#include <Arduino.h>
#include "include/pins.h"
#include "include/calc.h"
#include "include/core.h"
#include "include/sensors.h"
#include "include/maps.h"
#include "include/eeprom.h"
#include "include/input.h"
#include "include/config.h"
#include "include/ui.h"
#include "include/serial_config.h"
#include <SoftTimer.h>
#include <AutoPID.h>
#include <FlexCAN_T4.h>

#define cbsize 16
#define CANBUS true
byte wantedGear = 100;

// INPUT

// Pid tuning parameters, for boostCtrl
const double boostKp = 21; //80,21 Pid Proporional Gain. Initial ramp up i.e Spool, Lower if over boost
double boostKi = 7;        //40,7 Pid Integral Gain. Overall change while near Target Boost, higher value means less change, possible boost spikes
const double boostKd = 0;  //100, 1 Pid Derivative Gain.
double pidBoost, boostPWM, pidBoostLim;
int boostOverride = 150;
//Load PID controller
AutoPID boostPID(&pidBoost, &pidBoostLim, &boostPWM, 0, 255, boostKp, boostKi, boostKd);

#ifdef ECU
// Pid tuning parameters, for injectionCtrl
const double injectKp = 7; //80,21 Pid Proporional Gain. Initial ramp up, Lower if over
double injectKi = 20;      //40,7 Pid Integral Gain. Overall change while near Target
const double injectKd = 0; //100, 1 Pid Derivative Gain.
double pidInject, injectPWM, pidInjectLim;
//Load PID controller
AutoPID injectPID(&pidInject, &pidInjectLim, &injectPWM, 0, 255, injectKp, injectKi, injectKd);
#endif
bool justStarted = true;
boolean garageShift, garageShiftMove, tpsConfigMode, tpsInitPhase1, tpsInitPhase2 = false;
double garageTime, lastShift, lastInput, hornPressTime, lastPress;
int lockVal = 0;

double canTPS, canRPM, canCoolant, canSpeed;

#ifdef CANBUS
Circular_Buffer<uint32_t, cbsize> ids;
Circular_Buffer<uint32_t, cbsize, 10> storage;

void canSniff(const CAN_message_t &msg)
{ // global callback
  
  //Uncomment this to view incoming CAN messages
  // Serial.print("MB: "); Serial.print(msg.mb);
  // Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  // Serial.print("  ID: 0x"); Serial.print(msg.id, HEX );
  // Serial.print("  EXT: "); Serial.print(msg.flags.extended );
  // Serial.print("  LEN: "); Serial.print(msg.len);
  
  // Serial.print(" DATA: ");
  // for ( uint8_t i = 0; i <msg.len ; i++ ) {
  //   Serial.print(msg.buf[i]); Serial.print(" ");
  // }
  // Serial.print("  TS: "); Serial.println(msg.timestamp);

  uint32_t frame[10] = {msg.id};

  
  if (!storage.find(frame, 10, 0, 0, 0))
  {
    if (storage.size() == storage.capacity())
    {
      Serial.print("Buffer full, couldn't add CAN ID to the list!");
      return;
    }
    frame[0] = msg.id;
    for (uint8_t i = 0; i < 8; i++)
      frame[i + 1] = msg.buf[i];
    frame[9] = 1;
    storage.push_back(frame, 10);
    ids.push_back(msg.id);
    ids.sort_ascending();
  }
  else
  {
    frame[9]++;
    for (uint8_t i = 0; i < 8; i++)
      frame[i + 1] = msg.buf[i];
    storage.replace(frame, 10, 0, 0, 0);
  }

  //CAN ID 230 - HEX to DEC = 560
  if (frame[0] == 560)
  {
    int autoState = digitalRead(autoSwitch);
    if (autoState == HIGH)
    {
      stickCtrl = true;
      fullAuto = true;
    }
    else
    {
      stickCtrl = false;
      fullAuto = false;
    }
    if (frame[1] == 8)
    {
      wantedGear = 8;
      gear = 2; // force reset gear to 2
      shiftPending = false;
      shiftBlocker = false;
      garageShiftMove = false;
      if (debugEnabled)
      {
        Serial.println("Park requested via canbus");
      }
    }
    if (frame[1] == 7)
    {
      wantedGear = 7;
      gear = 2; // force reset gear to 2
      garageShiftMove = false;
      if (debugEnabled)
      {
        Serial.println("Reverse requested via canbus");
      }
    }
    if (frame[1] == 6)
    {
      wantedGear = 6;
      garageShiftMove = false;
      if (debugEnabled)
      {
        Serial.println("Neutral requested via canbus");
      }
    }
    if (frame[1] == 22) //16 HexToDec 22
    {
      wantedGear = 6;
      garageShiftMove = false;
      if (debugEnabled)
      {
        Serial.println("Neutral requested via canbus");
      }
    }

    if (frame[1] == 5)
    {
      wantedGear = 2;
      garageShiftMove = false;
      if (debugEnabled)
      {
        Serial.println("D requested via canbus");
      }
    }

    if (frame[1] == 21) // 15 HexToDec = 21
    {
      wantedGear = 2;
      garageShiftMove = false;
      if (debugEnabled)
      {
        Serial.println("D requested via canbus");
      }
    }

    if (frame[1] == 10)
    {
      gearDown();
      if (debugEnabled)
      {
        Serial.println("Downshift requested via canbus");
      }
    }
     if (frame[1] == 26) // 1A HexToDec = 26
    {
      gearDown();
      if (debugEnabled)
      {
        Serial.println("Downshift requested via canbus");
      }
    }
    if (frame[1] == 9)
    {
      gearUp();
      if (debugEnabled)
      {
        Serial.println("Upshift requested via canbus");
      }
    }
      if (frame[1] == 25) // 19 HexToDec = 25
    {
      gearUp();
      if (debugEnabled)
      {
        Serial.println("Upshift requested via canbus");
      }
    }
  }
  if (useCanSensors)
  {
    // CAB-BUS COOLANT
    // ID608  7 6D 3B 02 25 FF 01 7E
    // 6D is a coolant data - 40
    // Quote:
    //     if(rxId  == 0x608){
    //      {
    //          T=(rxBuf[0]-40);
    // 6D > hex to dec = 109
    // 109-40=69 *C
    // Thats how it works
    // CAN ID 608 - HEX to DEC = 1544
    if (frame[0] == 1544)
    {
      canCoolant = (frame[1]) - 40;
    }

    // CAN-BUS TPS
    // 210 8 02 FF 00 02 00 08 00 FF
    // pressed maximum
    // 210 8 02 FF FA 02 00 08 81 FF
    // calculation for TPS
    // 100*A/255
    // 100*FA/255 = 98%
    // FA HEX = 250 DEC
    // CAN ID210 //hex 528
    if (frame[0] == 528)
    {
      canTPS = 100 * (frame[3]) / 255; // (frame[7] << 8);
    }

    // CAN-BUS RPM
    // ID308 8 00 02 78 00 00 FF FF FF // idle
    // 256 x 02 + 78 (02 to dec & 78 to dec)
    // 256 x 2 + 120 = 632 RPM
    // CAN ID308
    if (frame[0] == 776)
    {
      canRPM = 256 * (frame[2]) + (frame[3]);
    }

// CAN-BUS SPEED ID200 (HexToDec - 512)
// ID200 8 00 18 02 9F 02 9A 02 9C // sample message
    if (frame[0] == 512)
    {
      //int canSpeedPulses   = ((8 * ((frame[3]) + ((frame[5]))) + (((frame[4]) + (frame[6])) / 2) / 15));
      int rpm_right = (((frame[3] & 0b00111111) << 8) | frame[4]) / 2;    //RPM RAW value is x2
      int rpm_left  = (((frame[5] & 0b00111111) << 8) | frame[6]) / 2;    //RPM RAW value is x2
      int canSpeedPulses   = (rpm_right + rpm_left) / 2;
      float tireDiameter = ((config.tireProfile * 2) + (config.tireInches * 25.4)) + config.tireOffset;
      float tireCircumference = 3.14 * tireDiameter;     
      canSpeed = (tireCircumference * canSpeedPulses * 60) / 1000000;
    }
  }
}
#endif
// Polling for stick control
// This is W202 electronic gear stick, should work on any pre-canbus sticks.
void pollstick(Task *me)
{

  digitalToggle(13);

  if (justStarted)
  {
    #ifdef CANBUS
        FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
        FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1; 

        Can0.setBaudRate(500000);
        Can0.enableFIFO(1);
        Can0.enableFIFOInterrupt(1);
        //Can0.setFIFOFilter(ACCEPT_ALL);
        Can0.setFIFOFilter(REJECT_ALL);
        Can0.setFIFOFilter(0, 1544, STD, NONE); //608 - coolant
        Can0.setFIFOFilter(1, 528, STD, NONE);  //210 - TPS
        Can0.setFIFOFilter(2, 776, STD, NONE);  //308 - RPM
        Can0.setFIFOFilter(3, 512, STD, NONE);  //200 - speed
        Can0.setFIFOFilter(4, 560, STD, NONE);  //230 - shifter
        Can0.enhanceFilter(FIFO);
        Can0.onReceive(canSniff);
        justStarted = false;
    #endif
  }
#ifndef CANBUS
  if (!resistiveStick)
  {
    // Read the stick.
    int whiteState = digitalRead(whitepin);
    int blueState = digitalRead(bluepin);
    int greenState = digitalRead(greenpin);
    int yellowState = digitalRead(yellowpin);
    int autoState = digitalRead(autoSwitch);
    garageShiftMove = true;
    // Determine position
    if (whiteState == HIGH && blueState == HIGH && greenState == HIGH && yellowState == LOW)
    {
      wantedGear = 8;
      gear = 2; // force reset gear to 2
      shiftPending = false;
      shiftBlocker = false;
      garageShiftMove = false;
    } // P
    if (whiteState == LOW && blueState == HIGH && greenState == HIGH && yellowState == HIGH)
    {
      wantedGear = 7;
      gear = 2; // force reset gear to 2
      garageShiftMove = false;
    } // R
    if (whiteState == HIGH && blueState == LOW && greenState == HIGH && yellowState == HIGH)
    {
      wantedGear = 6;
      garageShiftMove = false;
    } // N
    if (whiteState == LOW && blueState == LOW && greenState == HIGH && yellowState == LOW)
    {
      wantedGear = 5;
      garageShiftMove = false; // these should not be necessary after wantedGear <5, but don't want to risk this keeping y5 alive for some reason.
    }
    if (whiteState == LOW && blueState == LOW && greenState == LOW && yellowState == HIGH)
    {
      wantedGear = 4;
      garageShiftMove = false;
    }
    if (whiteState == LOW && blueState == HIGH && greenState == LOW && yellowState == LOW)
    {
      wantedGear = 3;
      garageShiftMove = false;
    }
    if (whiteState == HIGH && blueState == LOW && greenState == LOW && yellowState == LOW)
    {
      wantedGear = 2;
      garageShiftMove = false;
    }
    if (whiteState == HIGH && blueState == HIGH && greenState == LOW && yellowState == HIGH)
    {
      wantedGear = 1;
      garageShiftMove = false;
    }

    if (autoState == HIGH)
    {
      if (!stickCtrl)
      {
        if (debugEnabled)
        {
          Serial.println(F("pollstick: stickCtrl on "));
        }
        stickCtrl = true;
        fullAuto = true;
      }
    }
    else
    {
      if (stickCtrl)
      {
        if (debugEnabled)
        {
          Serial.println(F("pollstick: stickCtrl off "));
        }
        stickCtrl = false;
        fullAuto = false;
      }
    }
  }
  else
  {
    int blueState = analogRead(bluepin);
    if (blueState > 450 && blueState < 750)
    {
      wantedGear = 8;
      gear = 2; // force reset gear to 2
      shiftPending = false;
      shiftBlocker = false;
      garageShiftMove = false;
    }
    if (blueState > 300 && blueState < 400)
    {
      wantedGear = 7;
      gear = 2; // force reset gear to 2
      garageShiftMove = false;
    }
    if (blueState > 200 && blueState < 300)
    {
      wantedGear = 6;
      garageShiftMove = false;
    }
    if (blueState > 100 && blueState < 200)
    {
      wantedGear = 5;
      garageShiftMove = false;
    }
  }
#endif
}

// For manual microswitch control, gear up
void gearUp()
{
  if (wantedGear < 6 && !fullAuto && gear < 5)
  { // Do nothing if we're on N/R/P
    if (!shiftBlocker)
    {
      stickCtrl = false;
      newGear = gear;
      newGear++;
      shiftPending = true;
      gearchangeUp(newGear);
    }

    if (debugEnabled)
    {
      Serial.println(F("gearup: Gear up requested"));
    }
  }
}

// For manual microswitch control, gear down
void gearDown()
{
  if (wantedGear < 6 && !fullAuto && gear > 1)
  { // Do nothing if we're on N/R/P
    if (!shiftBlocker)
    {
      stickCtrl = false;
      newGear = gear;
      newGear--;
      shiftPending = true;
      gearchangeDown(newGear);
    }

    if (debugEnabled)
    {
      Serial.println(F("geardown: Gear down requested"));
    }
  }
}

// Polling for manual switch keys
void pollkeys()
{
  int gupState = 0;
  int gdownState = 0;

  if (!resistiveStick)
  {
    gupState = digitalRead(gupSwitch);     // Gear up
    gdownState = digitalRead(gdownSwitch); // Gear down

    if (gdownState == LOW && gupState == HIGH)
    {
      if (debugEnabled)
      {
        Serial.println(F("pollkeys: Gear up button"));
      }
      gearUp();
    }
    else if (gupState == LOW && gdownState == HIGH)
    {

      if (debugEnabled)
      {
        Serial.println(F("pollkeys: Gear down button"));
      }
      gearDown();
    }
  }
  else
  {

    gupState = analogRead(gupSwitchalt);  // Gear up
    gdownState = analogRead(gdownSwitch); // Gear down

    if (gupState < 20)
    {
      if (debugEnabled)
      {
        Serial.println(F("pollkeys: Gear up button"));
      }
      gearUp();
    }
    if (gdownState < 100)
    {
      if (debugEnabled)
      {
        Serial.println(F("pollkeys: Gear down button"));
      }
      gearDown();
    }
    /* Serial.print(gdownState);
  Serial.print("-");
  Serial.println(gupState);*/
  }
}

void hornOn()
{
  // Simple horn control
  digitalWrite(hornPin, HIGH);
  horn = true;
  hornPressTime = millis();
  if (debugEnabled)
  {
    Serial.println("Horn pressed");
  }
}

void hornOff()
{
  digitalWrite(hornPin, LOW);
  horn = false;
  if (debugEnabled)
  {
    Serial.println("Horn depressed");
  }
}

void boostControl(Task *me)
{
  if (boostLimit)
  {
    struct SensorVals sensor = readSensors();
    pidBoost = double(sensor.curBoost);
    pidBoostLim = double(sensor.curBoostLim);
    // pidBoost = double(sensor.curTps);
    // pidBoostLim = double(50);
    boostPID.setBangBang(100, 20);
    boostPID.setTimeStep(50);
    boostPID.run();

    // Just a sanity check to make sure PID library is not doing anything stupid.
    if (truePower)
    {
      analogWrite(boostCtrl, int(boostPWM));
      // if (debugEnabled) { Serial.print("BoostPWM = "); Serial.println(boostPWM); }
    }
    else
    {
      analogWrite(boostCtrl, 0);
    }

    if (exhaustPresSensor)
    {
      if (sensor.curBoost > 150 && ((sensor.curExPres - sensor.curBoost) > 50))
      {
        analogWrite(boostCtrl, 0);
        Serial.println("Exhaust pressure 0.5bar greater than boost, overriding boost control for relief. ");
      }
    }

    /*if (debugEnabled)
    {
      Serial.print(F("boostControl (allowedBoostPressure/bootSensor):"));
      Serial.print(sensor.curBoostLim);
      Serial.print(F("-"));
      Serial.print(sensor.curBoost);
    }*/
  }
}

void fuelControl(Task *me)
{
  if (fuelPumpControl)
  {
    struct SensorVals sensor = readSensors();

    if ((sensor.curRPM > config.fuelMaxRPM || millis() < 5000) && !fuelPumps)
    {
      analogWrite(fuelPumpCtrl, 255);

      if (debugEnabled)
      {
        Serial.print(F("[fuelControl->fuelControl] Fuel Pump RPM limit hit/Prestart init, enabling pumps: "));
        Serial.println(config.fuelMaxRPM);
      }
      fuelPumps = true;
    }
    else if (sensor.curRPM < config.fuelMaxRPM && fuelPumps && millis() > 5000)
    {
      analogWrite(fuelPumpCtrl, 0);
      if (debugEnabled)
      {
        Serial.print(F("[fuelControl->fuelControl] Fuel Pump RPM disabled due low rpm/timelimit "));
      }
      fuelPumps = false;
    }
  }
}

// Polling time for transmission control
// R/N/P modulation pressure regulation
// idle SPC regulation
// Boost control
void polltrans(Task *me)
{
  struct SensorVals sensor = readSensors();
  unsigned int shiftDelay = 2000;

  if (shiftBlocker)
  {
    if (tpsSensor)
    {
      shiftDelay = readPercentualMap(shiftTimeMap, spcPercentVal, sensor.curAtfTemp);
    }
    else
    {
      shiftDelay = 1200;
    }
    shiftDuration = millis() - shiftStartTime;
    if (shiftDuration > shiftDelay && shiftDone)
    {
      if (debugEnabled)
      {
        Serial.print(F("[polltrans->switchGearStop] shiftDelay-spcPercentVal-atfTemp "));
        Serial.print(shiftDelay);
        Serial.print(F("-"));
        Serial.print(spcPercentVal);
        Serial.print(F("-"));
        Serial.println(atfRead());
      }
      switchGearStop();
    }
    if (preShift && !preShiftDone)
    {
      doPreShift();
    }
    else if (!preShift && preShiftDone)
    {
      doShift();
    }
    else if (postShift && !postShiftDone)
    {
      doPostShift();
      if (gear == 7)
      {
        digitalWrite(reversePin, HIGH);
      }
      else
      {
        digitalWrite(reversePin, LOW);
      }
    }
  }

  //Raw value for pwm control (0-255) for SPC solenoid, see page 9: http://www.all-trans.by/assets/site/files/mercedes/722.6.1.pdf
  // "Pulsed constantly while idling in Park or Neutral at approximately 40% Duty cycle" <- 102/255 = 0.4
  // MPC = varying with load, SPC constant 33%
  //int mpcVal = readMap(mpcNormalMap, sensor.curLoad, sensor.curAtfTemp);

  if (!shiftBlocker)
  {
    // Pulsed constantly while idling in Park or Neutral at approximately 33% Duty cycle.
    if (wantedGear == 6 || wantedGear == 8)
    {
      analogWrite(spc, 20);
      garageShift = true;
      garageTime = millis();
    }
    // Pulsed constantly while idling in Park or Neutral at approximately 40% Duty cycle, also for normal mpc operation
    if (wantedGear == 8 || wantedGear == 6 || (wantedGear <= 6 && !shiftPending && !shiftBlocker && (millis() - lastShiftPoint) > 5000))
    {
      // int mpcSetVal = (100 - mpcVal) * 2.55;
      int mpcSetVal = 102;
      //  analogWrite(mpc, mpcSetVal);
    }

    if ((wantedGear == 7 || (wantedGear < 6 && !shiftPending)) && garageShift && (millis() - garageTime > 1000))
    {
      analogWrite(spc, 0);
      garageShift = false;
    }

    // 3-4 Shift solenoid is pulsed continuously while in Park and during selector lever movement (Garage Shifts).
    // Testing whether we actually need this.
    if (wantedGear > 5 && garageShiftMove && stickCtrl)
    {
      analogWrite(y5, 255);
      // delay(500);
    }
    if (!garageShiftMove)
    {
      analogWrite(y5, 0);
    }

    if (tccLock)
    {
      // Enable torque converter lock when tps is less than 40%, current speed is more than 80km/h and gear is within allowed range.
      if (sensor.curTps < 40 && sensor.curSpeed > 30 && gear >= config.firstTccGear && gear > 1 && sensor.curRPM < 2500)
      {
        if (lockVal <= 255)
        {
          lockVal = lockVal + 85;
          analogWrite(tcc, lockVal);
        }
        else
        {
          analogWrite(tcc, 255);
        }
      }
      else
      {
        if (lockVal >= 85)
        {
          lockVal = lockVal - 85;
          analogWrite(tcc, lockVal);
        }
        else
        {
          analogWrite(tcc, 0);
        }
      }
    }
    // "1-2/4-5 Solenoid is pulsed during ignition crank." stop doing this after we get ourselves together.
    if (ignition)
    {
      analogWrite(y3, 0);
      ignition = false;
    }
    if (evalGear && !shiftBlocker && millis() - lastShiftPoint > 5000 & wrongGearPoint < 5 & !shiftConfirmed)
    {
      int evaluatedGear = evaluateGear();
      if (millis() - lastShiftPoint > 5100)
      {
        if (evaluatedGear != gear)
        {
          wrongGearPoint++;
        }
      }
      if (millis() - lastShiftPoint > 5500)
      {
        if (evaluatedGear != gear)
        {
          wrongGearPoint++;
        }
      }
      if (millis() - lastShiftPoint > 6000)
      {
        if (evaluatedGear != gear)
        {
          wrongGearPoint++;
        }
      }
      if (millis() - lastShiftPoint > 6500)
      {
        if (evaluatedGear != gear)
        {
          wrongGearPoint++;
        }
      }
      if (millis() - lastShiftPoint > 7000)
      {
        if (evaluatedGear != gear)
        {
          wrongGearPoint++;
        }
        if (wrongGearPoint < 3)
        {
          shiftConfirmed = true;
        }
      }
    }
    if (wrongGearPoint >= 3)
    {
      int evaluatedGear = evaluateGear();
      if (evaluatedGear < 6 && wantedGear < 6)
      {
        gear = evaluateGear();
        wrongGearPoint = 0;
      }
    }
    if (evalGear & sensor.curSpeed < 10)
    {
      // gear = evaluateGear();
    }
  }

  if (radioEnabled)
  {
    radioControl();
  }
  if (manual)
  {
    pollkeys();
  }
  if (horn && (millis() - hornPressTime > 300))
  {
    hornOff();
  }
  if (sensor.curRPM > 0)
  {
    carRunning = true;
  }
  else
  {
    carRunning = false;
  }
}

int adaptSPC(int mapId, int xVal, int yVal)
{
  int current = 0;
#ifdef ASPC
  int modVal = 5;
  int aSpcUpState = digitalRead(aSpcUpSwitch);     // Adapt pressure up
  int aSpcDownState = digitalRead(aSpcDownSwitch); // Adapt pressure down
  static int prevaSpcUpState = 0;
  static int prevaSpcDownState = 0;
  int current = readEEPROM(mapId, xVal, yVal);

  if (aSpcDownState != prevaSpcDownState || aSpcUpState != prevaSpcUpState)
  {
    if (aSpcDownState == LOW && aSpcUpState == HIGH)
    {
      prevaSpcUpState = aSpcUpState;

      if (debugEnabled)
      {
        Serial.println(F("adaptSPC: More pressure button"));
        Serial.print(F("adaptSPC: request values: "));
        Serial.print(mapId);
        Serial.print(F("-"));
        Serial.print(xVal);
        Serial.print(F("-"));
        Serial.println(yVal);
        Serial.print(F("adaptSPC: old adapt pressure is: "));
        Serial.println(current);
      }
      current = current + modVal;
      writeEEPROM(mapId, xVal, yVal, current);

      if (debugEnabled)
      {
        Serial.print(F("adaptSPC: New adapt pressure is: "));
        Serial.println(current);
      }
    }
    else if (aSpcUpState == LOW && aSpcDownState == HIGH)
    {
      prevaSpcDownState = aSpcDownState;

      if (debugEnabled)
      {
        Serial.println(F("adaptSPC: Less pressure button"));
        Serial.print(F("adaptSPC: request values: "));
        Serial.print(mapId);
        Serial.print(F("-"));
        Serial.print(xVal);
        Serial.print(F("-"));
        Serial.println(yVal);
        Serial.print(F("adaptSPC: old adapt pressure is: "));
        Serial.println(current);
      }
      current = current - modVal;
      writeEEPROM(mapId, xVal, yVal, current);

      if (debugEnabled)
      {
        Serial.print(F("adaptSPC: New adapt pressure is: "));
        Serial.println(current);
      }
    }
  }
#endif
  return current;
}

void injectionControl(Task *me)
{
#ifdef ECU
  struct SensorVals sensor = readSensors();
  int fuelRequire = sensor.curLoad / sensor.curLambda; // eg. 100% load / 100% lambda = 1x fueling, 100% load / 10% lambda = 10x fueling
  int fuelAmount = readMap(injectionMap, fuelRequire, sensor.curRPM);
  fuelAmount = fuelAmount * 2.55;
  injectPID.setBangBang(100, 50);
  injectPID.setTimeStep(100);
  // Read injectionpump travel
  injectPID.run();
  analogWrite(injectionPin, injectPWM);
  if (debugEnabled)
  {
    Serial.print("Fueling quantity with load/lambda: ")
        Serial.print(sensor.curLoad);
    Serial.print("/");
    Serial.print(sensor.curLambda);
    Serial.print(" is ");
    Serial.print(fuelAmount);
  }
#endif
}

void radioControl()
{
  static byte readData;
  static byte pwrCounter = 1;

  if (Serial1.available() > 0)
  {
    readData = Serial1.read();

    if (readData == 100 && !shiftPending && gear < 5)
    {
      lastShift = millis();
      gearUp();
      readData = 0;
    }
    else if (readData == 100 && shiftPending)
    {
      readData = 0;
      Serial.println("Steering: shift pending");
    }
    else if (readData == 200 && !shiftPending && gear > 1)
    {
      lastShift = millis();
      gearDown();
      readData = 0;
    }
    else if (readData == 200 && shiftPending)
    {
      readData = 0;
      Serial.println("Steering: shift pending");
    }
    else if (readData == 55)
    {
      hornOn();
      readData = 0;
    }
    else if (readData == 101)
    {
      /*  tpsConfigMode = true;
      tpsInitPhase1, tpsInitPhase2 = false;*/
      if (boostOverride < 300)
      {
        boostOverride = boostOverride + 50;
        infoBoost = false;
      }
    }
    else if (readData == 201)
    {
      //tpsConfigMode = false;
      if (boostOverride > 0)
      {
        boostOverride = boostOverride - 50;
        infoBoost = false;
      }
    }
    else if (readData == 150)
    {
      if (page < 7)
      {
        page++;
      }
      else if (page > 7)
      {
        page = 1;
      }
      readData = 0;
    }
    else if (readData == 151)
    {
      if (page > 1)
      {
        page--;
      }
      else if (page < 1)
      {
        page = 7;
      }
      readData = 0;
    }
    else if (readData == 249)
    {
      if (millis() - lastInput > 1000)
      {
        pwrCounter = pwrCounter + 1;
        lastInput = millis();
      }

      if (truePower && pwrCounter > 5)
      {
        truePower = false;
        pwrCounter = 1;
      }
      else if (!truePower && pwrCounter > 5)
      {
        truePower = true;
        pwrCounter = 1;
      }
    }
  }
}

void keypadWatch(Task *me)
{
  int keypadValue = analogRead(keypadPin);
  if ((millis() - lastPress > 500))
  {
    if (keypadValue > 850 && keypadValue < 870)
    {
      lastPress = millis();
      keypadControl(100); //Gear UP
    }
    if (keypadValue > 230 && keypadValue < 250)
    {
      lastPress = millis();
      keypadControl(101); //BOOST Override UP
    }
    if (keypadValue > 115 && keypadValue < 140)
    {
      lastPress = millis();
      keypadControl(200); //Gear DOWN
    }
    if (keypadValue > 485 && keypadValue < 510)
    {
      lastPress = millis();
      keypadControl(201); //BOOST Override DOWN
    }
    if (keypadValue > 725 && keypadValue < 750)
    {
      lastPress = millis();
      keypadControl(249); //PWR Counter
    }
    if (keypadValue > 370 && keypadValue < 400)
    {
      lastPress = millis();
      keypadControl(150); //Previous page
    }
    if (keypadValue > 580 && keypadValue < 620)
    {
      lastPress = millis();
      keypadControl(151); //Next page
    }
    if (keypadValue > 950 && keypadValue < 1024)
    {
      lastPress = millis();
      keypadControl(160);
    }
    if (keypadValue > 900 && keypadValue < 901)
    {
      lastPress = millis();
      keypadControl(161);
    }
    if (keypadValue < 3000 || keypadValue < 3000)
    {
      keypadControl(55); //Horn
    }
  }
}

void keypadControl(int command)
{
  static byte readData;
  static byte pwrCounter = 1;

  if (command > 0)
  {
    readData = command;

    if (readData == 100 && !shiftPending && gear < 5)
    {
      lastShift = millis();
      gearUp();
      readData = 0;
    }
    else if (readData == 100 && shiftPending)
    {
      readData = 0;
      Serial.println("Steering: shift pending");
    }
    else if (readData == 200 && !shiftPending && gear > 1)
    {
      lastShift = millis();
      gearDown();
      readData = 0;
    }
    else if (readData == 200 && shiftPending)
    {
      readData = 0;
      Serial.println("Steering: shift pending");
    }
    else if (readData == 55)
    {
      hornOn();
      readData = 0;
    }
    else if (readData == 101)
    {
      /*  tpsConfigMode = true;
      tpsInitPhase1, tpsInitPhase2 = false;*/
      if (boostOverride < 300)
      {
        boostOverride = boostOverride + 50;
        infoBoost = false;
      }
    }
    else if (readData == 201)
    {
      //tpsConfigMode = false;
      if (boostOverride > 0)
      {
        boostOverride = boostOverride - 50;
        infoBoost = false;
      }
    }
    else if (readData == 150)
    {
      if (page < 7)
      {
        page++;
      }
      else if (page > 7)
      {
        page = 1;
      }
      readData = 0;
    }
    else if (readData == 151)
    {
      if (page > 1)
      {
        page--;
      }
      else if (page < 1)
      {
        page = 7;
      }
      readData = 0;
    }
    else if (readData == 249)
    {
      if (millis() - lastInput > 1000)
      {
        pwrCounter = pwrCounter + 1;
        lastInput = millis();
      }

      if (truePower && pwrCounter > 5)
      {
        truePower = false;
        pwrCounter = 1;
      }
      else if (!truePower && pwrCounter > 5)
      {
        truePower = true;
        pwrCounter = 1;
      }
    }
  }
}