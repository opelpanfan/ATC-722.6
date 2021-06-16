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
#include "include/pins.h"
#include "include/sensors.h"
#include "include/core.h"
#include "include/input.h"
#include "include/ui.h"
#include "include/serial_config.h"
#include "include/config.h"

#include <EEPROM.h>
#include <SoftTimer.h>
#include <AutoPID.h>

// "Protothreading", we have time slots for different functions to be run.
//Task pollDisplay(200, updateDisplay);     // 500ms to update display*/
Task pollData(33, datalog);               // 200ms to update datalogging
Task pollStick(100, pollstick);           // 100ms for checking stick position*
Task pollGear(200, decideGear);           // 200ms for deciding new gear*/
Task pollSensors(80, pollsensors);        // 100ms to update sensor values*/
Task pollTrans(50, polltrans);            // 50ms to check transmission state (this needs to be faster than stick.)
Task pollSerialWatch(50, serialWatch);
Task keypadPressWatch(100, keypadWatch);


#define NEXTION

#ifdef NEXTION // if nextion screen is used
#include "include/nextion.h" // include nection library
Nextion screen = Nextion(); // create nextion class object

void fastRefreshScreen(Task *me); //define screen refresh function
Task pollNextionFast(100, fastRefreshScreen); //define refresh task and set frequency

void averageRefreshScreen(Task *me); //define screen refresh function
Task pollNextionAverage(500, averageRefreshScreen); //define refresh task and set frequency

void slowRefreshScreen(Task *me); //define screen refresh function
Task pollNextionSlow(1000, slowRefreshScreen); //define refresh task and set frequency

int realRPM = 0;
int realATF = 0;
int realLOAD = 0;
int realTPS = 0;
int realRPM2 = 0;
int realATF2 = 0;
int realtps_Bar = 0;
int realload_Bar = 0;
int realAtf_Bar = 0;


void fastRefreshScreen(Task *me) //screen refresh function all display data goes here
{

struct SensorVals sensor = readSensors(); //read current sensor data

//SPEED display on Nextion
  screen.setText("speedValue", String(sensor.curSpeed)); 
// RPM value + gauge display on Nextion
  screen.setText("rpmValue", sensor.curRPM);
  realRPM = screen.mapInt(sensor.curRPM, 0, 7000, 0, 253);
  realRPM = realRPM < 0 ? 360 + realRPM : realRPM;
  screen.setVal("rpmGauge", realRPM);
// ATF value + gauge display on Nextion
  screen.setText("atfValue", sensor.curAtfTemp);
  realATF = screen.mapInt(sensor.curAtfTemp, 0, 140, 0, 253);
  realATF = realATF < 0 ? 360 + realATF : realATF;
  screen.setVal("atfGauge", realATF);
// LOAD value + gauge display on Nextion
  screen.setText("loadValue", sensor.curLoad);
  realLOAD = screen.mapInt(sensor.curLoad, 0, 100, 0, 277);
  realLOAD = realLOAD < 0 ? 360 + realLOAD : realLOAD;
  screen.setVal("loadGauge", realLOAD);
// TPS value + gauge display on Nextion
  screen.setText("tpsValue", sensor.curTps);
  realTPS = screen.mapInt(sensor.curTps, 0, 100, 0, 277);
  realTPS = realTPS < 0 ? 360 + realTPS : realTPS;
  screen.setVal("tpsGauge", realTPS);
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
  screen.setVal("rpmGauge2", realRPM2);
//LOAD value
  screen.setText("cltValue", canCoolant);
}

void averageRefreshScreen(Task *me) //screen refresh function all display data goes here
{
struct SensorVals sensor = readSensors(); //read current sensor data

//PRND graphics display on Nextion
  //wantedGear 6 = N
  //wantedGear 7 = R
  //wantedGear 8 = P
  screen.setPic("gear", wantedGear == 8 ? 2 : wantedGear == 7 ? 3 : (wantedGear < 6 &&  wantedGear > 0) ? 5 : 4);
//Current Gear graphics display
  screen.setPic("curGear", gear == 5 ? 10 : gear == 4 ? 9 : gear == 3 ? 8 : gear == 2 ? 7 : gear == 1 ? 6 : 7);
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
// TPS bar
screen.setText("tps_Bar", sensor.curTps);
int realtps_Bar = screen.mapInt(sensor.curTps, 0, 100, 0, 100);
screen.setVal("tps_Bar", realtps_Bar);
// Load bar
screen.setText("load_Bar", sensor.curLoad);
int realload_Bar = screen.mapInt(sensor.curLoad, 0, 100, 0, 100);
screen.setVal("load_Bar", realload_Bar);
// ATF bar
screen.setText("atf_Bar", sensor.curAtfTemp);
int realAtf_Bar = screen.mapInt(sensor.curAtfTemp, 0, 100, 0, 100);
screen.setVal("atf_Bar", realAtf_Bar);
// Manual / Automatic mode
int autoState = digitalRead(autoSwitch);
screen.setPic("mode", autoState == HIGH ? 12 : autoState == LOW ? 13 : 13);
}

void slowRefreshScreen(Task *me) //screen refresh function all display data goes here
{
  struct SensorVals sensor = readSensors(); //read current sensor data
// BATTERY display  
  screen.setText("bat_value", String((int)(sensor.curBattery / 1000)) + "." + String((int)(sensor.curBattery % 1000) / 10));
  int realBatt = screen.mapInt(sensor.curBattery, 11000, 16000, 0, 100);
  screen.setVal("bat_value", realBatt);
//ABS teeth display
  screen.setText("absValue", String(config.rearDiffTeeth));
//DIFF ratio display
  screen.setText("diffValue", String(config.diffRatio));
//TPS Agresivness ratio display
  screen.setText("tpsaValue", String(config.tpsAgre));
//Crankshaft trigger teeth ratio display
  screen.setText("crankValue", String(config.triggerWheelTeeth));
//LowRange ratio display
  screen.setText("lowrangeValue", String(config.transferRatio));
}

#endif

void setup()
{
  Serial.begin(115200);
  delay(5000);
  initConfig();
  
  pinMode(LED_BUILTIN, OUTPUT);  // 1-2/4-5 solenoid

  // MPC and SPC should have frequency of 1000hz
  // TCC should have frequency of 100hz
  // Lower the duty cycle, higher the pressures.

  analogWriteFrequency(spc, 1000);     // 1khz for spc
  analogWriteFrequency(mpc, 1000);     // and mpc
  analogWriteFrequency(boostCtrl, 30); // 30hz for boost controller
  analogWriteFrequency(rpmMeter, 50);  // 50hz for w124 rpm meter

  // Solenoid outputs
  pinMode(y3, OUTPUT);  // 1-2/4-5 solenoid
  pinMode(y4, OUTPUT);  // 2-3
  pinMode(y5, OUTPUT);  // 3-4
  pinMode(spc, OUTPUT); // shift pressure
  pinMode(mpc, OUTPUT); // modulation pressure
  pinMode(tcc, OUTPUT); // lock

  // Other LOW-Side outputs
  pinMode(rpmMeter, OUTPUT); 
  pinMode(speedoCtrl, OUTPUT);
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


  //*portConfigRegister(boostCtrl) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  //  *portConfigRegister(tpsPin) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  //*portConfigRegister(atfPin) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  //*portConfigRegister(n2pin) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  //*portConfigRegister(n3pin) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  //*portConfigRegister(speedPin) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  //*portConfigRegister(rpmPin) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  //*portConfigRegister(hornPin) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  //*portConfigRegister(reversePin) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  
  //For manual control
  pinMode(autoSwitch, INPUT);  // manual-auto mode
  pinMode(gupSwitch, INPUT);   // gear up
  pinMode(gdownSwitch, INPUT); // gear down

  //pinMode(fuelInPin, INPUT); // Fuel flow meter in
  // pinMode(fuelOutPin, INPUT); // Fuel flow meter out
  //*portConfigRegister(fuelInPin) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  // *portConfigRegister(fuelOutPin) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  //#endif

  //*portConfigRegister(autoSwitch) = PORT_PCR_MUX(1) | PORT_PCR_PE;

  //For stick control
  pinMode(whitepin, INPUT);
  pinMode(bluepin, INPUT);
  pinMode(greenpin, INPUT);
  pinMode(yellowpin, INPUT);

  //*portConfigRegister(whitepin) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  //*portConfigRegister(bluepin) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  //*portConfigRegister(greenpin) = PORT_PCR_MUX(1) | PORT_PCR_PE;
  //*portConfigRegister(yellowpin) = PORT_PCR_MUX(1) | PORT_PCR_PE;

  // Make sure solenoids are all off.
  analogWrite(y3, 255); // 1-2/4-5 Solenoid is pulsed during ignition crank.
  analogWrite(y4, 0);
  analogWrite(y5, 0);
  analogWrite(spc, 0);
  analogWrite(mpc, 0);
  analogWrite(tcc, 0);
  analogWrite(speedoCtrl, 0); // Wake up speedometer motor so it wont stick

  digitalWrite(rpmPin, HIGH); // pull-up

  attachInterrupt(digitalPinToInterrupt(n2pin), N2SpeedInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(n3pin), N3SpeedInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(speedPin), vehicleSpeedInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(rpmPin), rpmInterrupt, RISING);

  /* This is for erasing EEPROM on start.
  for (int i = 0; i < EEPROM.length(); i++) {
     EEPROM.write(i, 0);
  }
*/

  if (debugEnabled && !datalogger)
  {
    Serial.println(F("Started."));
  }

  // initialize timers
  SoftTimer.add(&pollData);
  SoftTimer.add(&pollStick);
  SoftTimer.add(&pollGear);
  SoftTimer.add(&pollSensors);
  SoftTimer.add(&pollTrans);
  SoftTimer.add(&pollSerialWatch);
  SoftTimer.add(&keypadPressWatch);

#ifdef NEXTION // nextion display implementation

  Serial2.begin(115200); //begin serial communication
  screen.setupScreen(Serial2); // begin screen communication using serial port defned above
  screen.setPage("0"); // set screen page
  screen.setDim(100);  // set screen brightness

  SoftTimer.add(&pollNextionFast); // start screen refresh task
  SoftTimer.add(&pollNextionAverage); // start screen refresh task
  SoftTimer.add(&pollNextionSlow); // start screen refresh task

#endif
}
