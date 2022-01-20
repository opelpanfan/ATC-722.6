/*
    722.6 transmission controller
    Copyright (C) 2018 Markus Kovero <mui@mui.fi>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

    Big thanks to 
    Tuomas Kantola for maps and related math
    Tommi Otsavaara for guiding in electronics
    Mikko Kovero and Pete for mechanical side of things
    Jami Karvanen for datalogging and frontend stuff
    Liia Ahola for pcb tracing
    Joosep Vahar for testing
    Toni Lassila and Jan Blonski for ideas
    Krzysztof Dymianiuk for hardware design, coding, Nextion support and ideas
    Benas Brazdziunas for coding and helping with Nextion support
*/

#include <Arduino.h>
#include "include/globals.h"

STM32_CAN Can0 (_CAN1, DEF);
static CAN_message_t outMsg;
static CAN_message_t inMsg;

#define DISPLAYTYPE1 // Can be DISPLAYTYPE2 also.

#ifdef ECU
Task pollInjectionControl(100, injectionControl);
#endif

#define NEXTION

#ifdef NEXTION // if nextion screen is used
#include "include/nextion.h" // include nection library
Nextion screen = Nextion(); // create nextion class object

void fastRefreshScreen(); //define screen refresh function
//Task pollNextionFast(100, fastRefreshScreen); //define refresh task and set frequency

void averageRefreshScreen(); //define screen refresh function
//Task pollNextionAverage(500, averageRefreshScreen); //define refresh task and set frequency

void slowRefreshScreen(); //define screen refresh function
//Task pollNextionSlow(1000, slowRefreshScreen); //define refresh task and set frequency

int realRPM = 0;
int realATF = 0;
int realLOAD = 0;
int realTPS = 0;
int realRPM2 = 0;
int realATF2 = 0;
int realtps_Bar = 0;
int realload_Bar = 0;


void fastRefreshScreen() //screen refresh function all display data goes here
{

  struct SensorVals sensor = readSensors(); //read current sensor data
  
// RPM value + gauge display on Nextion
  screen.setText("rpmValue", (String)sensor.curRPM);
  realRPM = screen.mapInt(sensor.curRPM, 0, 7000, 0, 253);
  realRPM = realRPM < 0 ? 360 + realRPM : realRPM;
  screen.setVal("rpmGauge", (String)realRPM);
// ATF value + gauge display on Nextion
  screen.setText("atfValue", (String)sensor.curAtfTemp);
  realATF = screen.mapInt(sensor.curAtfTemp, 0, 140, 0, 253);
  realATF = realATF < 0 ? 360 + realATF : realATF;
  screen.setVal("atfGauge", (String)realATF);
// LOAD value + gauge display on Nextion
  screen.setText("loadValue", (String)sensor.curLoad);
  realLOAD = screen.mapInt(sensor.curLoad, 0, 100, 0, 277);
  realLOAD = realLOAD < 0 ? 360 + realLOAD : realLOAD;
  screen.setVal("loadGauge", (String)realLOAD);
// TPS value + gauge display on Nextion
  screen.setText("tpsValue", (String)sensor.curTps);
  realTPS = screen.mapInt(sensor.curTps, 0, 100, 0, 277);
  realTPS = realTPS < 0 ? 360 + realTPS : realTPS;
  screen.setVal("tpsGauge", (String)realTPS);
//RATIO display on Nextion
  screen.setText("ratioValue", String(config.transferRatio));
  //screen.setText("ratioValue", String(sensor.curRatio));
//n2 display on Nextion
  screen.setText("n2Value", String(n2Speed));
//n3 display on Nextion
  screen.setText("n3Value", String(n3Speed));
//RPM2 (main window) display
  realRPM2 = screen.mapInt(sensor.curRPM, 0, 7000, 0, 252);
  realRPM2 = realRPM2 < 0 ? 360 + realRPM2 : realRPM2;
  screen.setVal("rpmGauge2", (String)realRPM2);
// ATF2 (main window)Nextion
  realATF2 = screen.mapInt(sensor.curAtfTemp, 140, 0, 90, 270);
  realATF2 = realATF2 < 0 ? 360 + realATF2 : realATF2;
  screen.setVal("atfGauge2", (String)realATF2);
// //PRND graphics display on Nextion
//   //wantedGear 6 = N
//   //wantedGear 7 = R
//   //wantedGear 8 = P
//   screen.setPic("gear", wantedGear == 8 ? 3 : wantedGear == 7 ? 4 : (wantedGear < 6 &&  wantedGear > 0) ? 6 : 3);
// //Current Gear graphics display
//   screen.setPic("curGear", gear == 5 ? 11 : gear == 4 ? 10 : gear == 3 ? 9 : gear == 2 ? 8 : gear == 1 ? 7 : 8);
// //Gear display on Nextion
//   screen.setText("gear_number", String(gear));
// //Speed display on Nextion
//   screen.setText("speed_val", String(sensor.curSpeed));
// //1to2
//   screen.setText("p1to2_val", String(config.oneTotwo));
// //2to3
//   screen.setText("p2to3_val", String(config.twoTothree));
// //3to4
//   screen.setText("p3to4_val", String(config.threeTofour));
// //4to5
//   screen.setText("p4to5_val", String(config.fourTofive));
// //5to4
//   screen.setText("p5to4_val", String(config.fiveTofour));
// //4to3
//   screen.setText("p4to3_val", String(config.fourTothree));
// //3to2
//   screen.setText("p3to2_val", String(config.threeTotwo));
// //2to1
//   screen.setText("p2to1_val", String(config.twoToone));
// //TPS bar
// screen.setText("tps_Bar", sensor.curTps);
// int realtps_Bar = screen.mapInt(sensor.curTps, 0, 100, 0, 100);
// screen.setVal("tps_Bar", realtps_Bar);
// //Load bar
// screen.setText("load_Bar", sensor.curLoad);
// int realload_Bar = screen.mapInt(sensor.curLoad, 0, 100, 0, 100);
// screen.setVal("load_Bar", realload_Bar);

// //  screen.setText("atf_bar", sensor.curAtfTemp);
// //  int realATF = screen.mapInt(sensor.curAtfTemp, -40, 130, 0, 100);
// //  screen.setVal("atf_bar", realATF);

// //ATF bar + value display on Nextion
// //  screen.setText("atf_value", sensor.curAtfTemp);
// //  screen.setText("atf_bar", sensor.curAtfTemp);
// //  int realATF = screen.mapInt(sensor.curAtfTemp, -40, 130, 0, 100);
// //  screen.setVal("atf_bar", realATF);
// //PRND value display on Nextion
// //screen.setText("PRND", wantedGear == 8 ? "P" : wantedGear == 7 ? "R" : wantedGear == 6 ? "N" : fullAuto < 6 ? "D" : String(wantedGear));

// //Battery bar + value display on Nextion

// screen.setText("bat_value", String((int)(sensor.curBattery / 1000)) + "." + String((int)(sensor.curBattery % 1000) / 10));
//   int realBatt = screen.mapInt(sensor.curBattery, 11000, 16000, 0, 100);
//   screen.setVal("bat_value", realBatt);

}

void averageRefreshScreen() //screen refresh function all display data goes here
{

  struct SensorVals sensor = readSensors(); //read current sensor data
  
// // RPM value + gauge display on Nextion
//   screen.setText("rpmValue", sensor.curRPM);
//   realRPM = screen.mapInt(sensor.curRPM, 0, 7000, 0, 253);
//   realRPM = realRPM < 0 ? 360 + realRPM : realRPM;
//   screen.setVal("rpmGauge", realRPM);
// // ATF value + gauge display on Nextion
//   screen.setText("atfValue", sensor.curAtfTemp);
//   realATF = screen.mapInt(sensor.curAtfTemp, 0, 140, 0, 253);
//   realATF = realATF < 0 ? 360 + realATF : realATF;
//   screen.setVal("atfGauge", realATF);
// // LOAD value + gauge display on Nextion
//   screen.setText("loadValue", sensor.curLoad);
//   realLOAD = screen.mapInt(sensor.curLoad, 0, 100, 0, 277);
//   realLOAD = realLOAD < 0 ? 360 + realLOAD : realLOAD;
//   screen.setVal("loadGauge", realLOAD);
// // TPS value + gauge display on Nextion
//   screen.setText("tpsValue", sensor.curTps);
//   realTPS = screen.mapInt(sensor.curTps, 0, 100, 0, 277);
//   realTPS = realTPS < 0 ? 360 + realTPS : realTPS;
//   screen.setVal("tpsGauge", realTPS);
// //RATIO display on Nextion
//   screen.setText("ratioValue", String(config.transferRatio));
//   //screen.setText("ratioValue", String(sensor.curRatio));
// //n2 display on Nextion
//   screen.setText("n2Value", String(n2Speed));
// //n3 display on Nextion
//   screen.setText("n3Value", String(n3Speed));
// //RPM2 (main window) display
//   realRPM2 = screen.mapInt(sensor.curRPM, 0, 7000, 0, 252);
//   realRPM2 = realRPM2 < 0 ? 360 + realRPM2 : realRPM2;
//   screen.setVal("rpmGauge2", realRPM2);
// // ATF2 (main window)Nextion
//   realATF2 = screen.mapInt(sensor.curAtfTemp, 140, 0, 90, 270);
//   realATF2 = realATF2 < 0 ? 360 + realATF2 : realATF2;
//   screen.setVal("atfGauge2", realATF2);
//PRND graphics display on Nextion
  //wantedGear 6 = N
  //wantedGear 7 = R
  //wantedGear 8 = P
  screen.setPic("gear", (String)(wantedGear == 8 ? 3 : wantedGear == 7 ? 4 : (wantedGear < 6 &&  wantedGear > 0) ? 6 : 3));
//Current Gear graphics display
  screen.setPic("curGear", (String)(gear == 5 ? 11 : gear == 4 ? 10 : gear == 3 ? 9 : gear == 2 ? 8 : gear == 1 ? 7 : 8));
//Gear display on Nextion
  screen.setText("gear_number", String(gear));
//Speed display on Nextion
  screen.setText("speed_val", String(sensor.curSpeed));
//1to2
  screen.setText("p1to2_val", String(config.oneTotwo));
//2to3
  screen.setText("p2to3_val", String(config.twoTothree));
//3to4
  screen.setText("p3to4_val", String(config.threeTofour));
//4to5
  screen.setText("p4to5_val", String(config.fourTofive));
//5to4
  screen.setText("p5to4_val", String(config.fiveTofour));
//4to3
  screen.setText("p4to3_val", String(config.fourTothree));
//3to2
  screen.setText("p3to2_val", String(config.threeTotwo));
//2to1
  screen.setText("p2to1_val", String(config.twoToone));
//TPS bar
screen.setText("tps_Bar", (String)sensor.curTps);
int realtps_Bar = screen.mapInt(sensor.curTps, 0, 100, 0, 100);
screen.setVal("tps_Bar", (String)realtps_Bar);
//Load bar
screen.setText("load_Bar", (String)sensor.curLoad);
int realload_Bar = screen.mapInt(sensor.curLoad, 0, 100, 0, 100);
screen.setVal("load_Bar", (String)realload_Bar);

//  screen.setText("atf_bar", sensor.curAtfTemp);
//  int realATF = screen.mapInt(sensor.curAtfTemp, -40, 130, 0, 100);
//  screen.setVal("atf_bar", realATF);

//ATF bar + value display on Nextion
//  screen.setText("atf_value", sensor.curAtfTemp);
//  screen.setText("atf_bar", sensor.curAtfTemp);
//  int realATF = screen.mapInt(sensor.curAtfTemp, -40, 130, 0, 100);
//  screen.setVal("atf_bar", realATF);
//PRND value display on Nextion
//screen.setText("PRND", wantedGear == 8 ? "P" : wantedGear == 7 ? "R" : wantedGear == 6 ? "N" : fullAuto < 6 ? "D" : String(wantedGear));

//Battery bar + value display on Nextion

// screen.setText("bat_value", String((int)(sensor.curBattery / 1000)) + "." + String((int)(sensor.curBattery % 1000) / 10));
//   int realBatt = screen.mapInt(sensor.curBattery, 11000, 16000, 0, 100);
//   screen.setVal("bat_value", realBatt);

}

void slowRefreshScreen() //screen refresh function all display data goes here
{
  struct SensorVals sensor = readSensors(); //read current sensor data
  
  screen.setText("bat_value", String((int)(sensor.curBattery / 1000)) + "." + String((int)(sensor.curBattery % 1000) / 10));
  int realBatt = screen.mapInt(sensor.curBattery, 11000, 16000, 0, 100);
  screen.setVal("bat_value", (String)realBatt);

}

#endif

void setup()
{
  initialiseTimers();
  //initConfig();

  // MPC and SPC should have frequency of 1000hz
  // TCC should have frequency of 100hz
  // Lower the duty cycle, higher the pressures.

  // analogWriteFrequency(spc, 1000);     // 1khz for spc
  // analogWriteFrequency(mpc, 1000);     // and mpc
  // analogWriteFrequency(boostCtrl, 30); // 30hz for boost controller
  // analogWriteFrequency(rpmMeter, 50);  // 50hz for w124 rpm meter

  Serial.begin(115200);

  if (radioEnabled)
  {
    Serial1.begin(9600);
    if (debugEnabled && !datalogger)
    {
      Serial.println("Radio initialized.");
    }
  }
  
  pinMode(Running_LED, OUTPUT);
  pinMode(Warning_LED, OUTPUT);
  pinMode(Alert_LED, OUTPUT);
  pinMode(Comms_LED, OUTPUT);

  // Solenoid outputs
  pinMode(y3, OUTPUT);  // 1-2/4-5 solenoid
  pinMode(y4, OUTPUT);  // 2-3
  pinMode(y5, OUTPUT);  // 3-4
  pinMode(spc, OUTPUT); // shift pressure
  pinMode(mpc, OUTPUT); // modulation pressure
  pinMode(tcc, OUTPUT); // lock
  pinMode(rpmMeter, OUTPUT);
  pinMode(boostCtrl, OUTPUT);
  pinMode(speedoCtrl, OUTPUT);
  pinMode(fuelPumpCtrl, OUTPUT);
  pinMode(hornPin, OUTPUT);
  pinMode(reversePin, OUTPUT);

  // Sensor input
  pinMode(boostPin, INPUT);     // boost sensor
  pinMode(tpsPin, INPUT);       // throttle position sensor
  pinMode(oilPin, INPUT);       // engine coolant sensor
  pinMode(atfPin, INPUT);       // ATF temp
  pinMode(n2pin, INPUT_PULLUP); // N2 sensor
  pinMode(n3pin, INPUT_PULLUP); // N3 sensor
  pinMode(speedPin, INPUT);     // vehicle speed
  pinMode(rpmPin, INPUT);
  pinMode(batteryPin, INPUT);
  pinMode(keypadPin, INPUT);  //keypad analog pin
  pinMode(lowGearPin, INPUT);  //keypad analog pin

  //For manual control
  pinMode(autoSwitch, INPUT);

  if (!resistiveStick)
  {
    pinMode(gupSwitch, INPUT);   // gear up
    pinMode(gdownSwitch, INPUT); // gear down
  }
  else
  {
    pinMode(gupSwitchalt, INPUT_PULLUP); // gear up
    pinMode(gdownSwitch, INPUT_PULLUP);  // gear down
  }

  pinMode(fuelInPin, INPUT); // Fuel flow meter in

  //For stick control
  pinMode(whitepin, INPUT);
  pinMode(bluepin, INPUT);
  pinMode(greenpin, INPUT);
  pinMode(yellowpin, INPUT);

  pinMode(exhaustPresPin, INPUT);
  pinMode(exhaustTempPin, INPUT);

  // Make sure solenoids are all off.
  analogWrite(y3, 255); // 1-2/4-5 Solenoid is pulsed during ignition crank.
  analogWrite(y4, 0);
  analogWrite(y5, 0);
  analogWrite(spc, 0);
  analogWrite(mpc, 0);
  analogWrite(tcc, 0);
  analogWrite(speedoCtrl, 0); // Wake up speedometer motor so it wont stick

  if (rpmSpeed && fuelPumpControl)
  {
    analogWrite(fuelPumpCtrl, 255); // Wake up fuel pumps
  }

  digitalWrite(rpmPin, HIGH); // pull-up

  attachInterrupt(digitalPinToInterrupt(n2pin), N2SpeedInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(n3pin), N3SpeedInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(speedPin), vehicleSpeedInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(rpmPin), rpmInterrupt, RISING);


  if (debugEnabled && !datalogger)
  {
    Serial.println(F("Started."));
  }

#ifdef ECU
  // SoftTimer.add(&pollInjectionControl);
#endif

#ifdef NEXTION // nextion display implementation
  Serial1.begin(115200); //begin serial communication
  screen.setupScreen(Serial1); // begin screen communication using serial port defned above
  screen.setPage("0"); // set screen page
  screen.setDim(100);  // set screen brightness
#endif
}

void loop()
{
  mainLoopCount++;
  LOOP_TIMER = TIMER_mask;

if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_1KHZ))
  {
    BIT_CLEAR(TIMER_mask, BIT_TIMER_1KHZ);
  }
  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_200HZ))
  {
    BIT_CLEAR(TIMER_mask, BIT_TIMER_200HZ);
  }
  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_100HZ))
  {
    BIT_CLEAR(TIMER_mask, BIT_TIMER_100HZ);
  }
  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_50HZ))
  {
    BIT_CLEAR(TIMER_mask, BIT_TIMER_50HZ);
    faultMon();
    digitalToggle(Comms_LED);
  }
  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_30HZ))
  {
    BIT_CLEAR(TIMER_mask, BIT_TIMER_30HZ);   
    polltrans(); 
  }

  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_15HZ))
  {
    BIT_CLEAR(TIMER_mask, BIT_TIMER_15HZ); 
    decideGear();
    digitalToggle(Warning_LED);
  }

  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ))
  {
    BIT_CLEAR(TIMER_mask, BIT_TIMER_10HZ);
    updateDisplay();
    pollstick();
    pollsensors();
    boostControl();
    serialWatch();
    keypadWatch();
    digitalToggle(Running_LED);
  }

  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_4HZ))
  {
    BIT_CLEAR(TIMER_mask, BIT_TIMER_4HZ);
    datalog();
  }

  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_1HZ))
  {
    BIT_CLEAR(TIMER_mask, BIT_TIMER_1HZ);
    fuelControl();
    digitalToggle(Alert_LED);
  }
}
