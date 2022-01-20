#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <EEPROM.h>
#include <STM32_CAN.h>
#include <Filters.h>
#include "Stream.h"

#include "include/calc.h"
#include "include/maps.h"
#include "include/sensors.h"
#include "include/eeprom.h"
#include "include/maps.h"
#include "include/ui.h"
#include "include/config.h"
#include "include/core.h"
#include "include/serial_config.h"
#include "include/input.h"
#include "include/timers.h"

#define BOARD_MAX_IO_PINS 110

#define STM // we're running this on TEENSY

//Handy bitsetting macros
#define BIT_SET(a,b) ((a) |= (1U<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1U<<(b)))
#define BIT_CHECK(var,pos) !!((var) & (1U<<(pos)))

#define interruptSafe(c) (noInterrupts(); {c} interrupts();) //Wraps any code between nointerrupt and interrupt calls

#define MS_IN_MINUTE 60000
#define US_IN_MINUTE 60000000

#define BIT_TIMER_1HZ             0
#define BIT_TIMER_4HZ             1
#define BIT_TIMER_10HZ            2
#define BIT_TIMER_15HZ            3
#define BIT_TIMER_30HZ            4
#define BIT_TIMER_50HZ            5
#define BIT_TIMER_100HZ            6
#define BIT_TIMER_200HZ           7
#define BIT_TIMER_1KHZ           8

volatile unsigned long ms_counter = 0;
volatile uint16_t mainLoopCount;

volatile uint8_t TIMER_mask;
volatile uint8_t LOOP_TIMER;

HardwareTimer Timer1(TIM1);
HardwareTimer Timer2(TIM2);
HardwareTimer Timer3(TIM3);
HardwareTimer Timer4(TIM4);
HardwareTimer Timer6(TIM6);
HardwareTimer Timer11(TIM11);
HardwareTimer Timer14(TIM14);



#define Running_LED PG9
#define Warning_LED PG10
#define Alert_LED PG11
#define Comms_LED PG12


#define y3 BOARD_MAX_IO_PINS - 1 // FMT3, orange<->brown/red // DOUT3, I need to use 16 as testing instead of 36.
#define y4 BOARD_MAX_IO_PINS - 1 // orange <-> brown/grey // DOUT2
#define y5 BOARD_MAX_IO_PINS - 1 //ex 14, orange <-> brown/black // DOUT1
#define mpc BOARD_MAX_IO_PINS - 1 // red <-> brown/pink // DOUT5 // FMT3
#define spc BOARD_MAX_IO_PINS - 1 // red <-> brown/yellow // DOUT4 // FMT3
#define tcc BOARD_MAX_IO_PINS - 1 // pink <-> brown/yellow/white // DOUT6
#define speedoCtrl BOARD_MAX_IO_PINS - 1 // orig 7, blue <-> blue/green // DOUT7, 
#define rpmMeter BOARD_MAX_IO_PINS - 1 // FMT2 missing // DOUT10
#define boostCtrl BOARD_MAX_IO_PINS - 1 // FMT1, green <-> green/white/yellow // DOUT8, 3?
#define fuelPumpCtrl BOARD_MAX_IO_PINS - 1 // missing // DOUT9, 13?
#define hornPin BOARD_MAX_IO_PINS - 1 // Horn

#define reversePin BOARD_MAX_IO_PINS - 1 // Reverse pin, test only, original "speedoCtrl"

// END OUTPUT PINS
#define injectionPin BOARD_MAX_IO_PINS - 1 // should be 16.


// INPUT PINS
// Stick input
#define whitepin BOARD_MAX_IO_PINS - 1 // orig pin 27 // 0.5kohm <-> yellow <-> grey-yellow-grey // DIN2 <-> blue
#define bluepin BOARD_MAX_IO_PINS - 1 // orig pin 34 // /0.5kohm <-> yellow <-> grey-green-grey // DIN4 <-> green
#define greenpin BOARD_MAX_IO_PINS - 1 // 0.5kohm <-> yellow <-> grey-white-grey // DIN1 <-> whiteblue
#define yellowpin BOARD_MAX_IO_PINS - 1  // orig pin 28 //0.5kohm <-> yellow <-> grey-black-grey // DIN3 <-> whiteorange

// Switches
#define autoSwitch BOARD_MAX_IO_PINS - 1 // ex. 22, 0.5kohm <-> yellow <-> grey-pink-grey // DIN5

// KeyPad
#define keypadPin  BOARD_MAX_IO_PINS - 1 // test pin only, original "bluepin"

//Low Gear Switch
#define lowGearPin  BOARD_MAX_IO_PINS - 1 // test pin only, original "yellowpin"


#define gdownSwitch BOARD_MAX_IO_PINS - 1 // ex. 23 <-> NC // DIN6
#define gupSwitch BOARD_MAX_IO_PINS - 1 // <-> NC // DIN7
#define gupSwitchalt BOARD_MAX_IO_PINS - 1
#define fuelInPin BOARD_MAX_IO_PINS - 1

#define exhaustPresPin BOARD_MAX_IO_PINS - 1 // A12, A9 used in coupe.
#define exhaustTempPin BOARD_MAX_IO_PINS - 1

// Car sensor input pins, black
#define tpsPin BOARD_MAX_IO_PINS - 1 // voltage div 5/3 <-> black <-> blue-black-blue = 1kohm/1.8kohm div // ANAIN3, boost? A3->A11
#define atfPin BOARD_MAX_IO_PINS - 1 // voltage div 5/3 <-> black <-> pink = 1kohm/1.8kohm div // ANAIN2
#define boostPin BOARD_MAX_IO_PINS - 1 // voltage div 5/3 <-> black <-> blue-brown-blue = 1kohm/1.8kohm div // ANAIN4, tps? A2->A10
#define oilPin BOARD_MAX_IO_PINS - 1 // voltage div 12/3 <-> black <-> white-pink-white = 1kohm/380ohm div // ANAIN1
#define n2pin BOARD_MAX_IO_PINS - 1 // voltage div 5/3 <-> black <-> whiteredwhite = 1kohm/1.8kohm div // DIN14
#define n3pin BOARD_MAX_IO_PINS - 1 // voltage div 5/3 <-> black <-> brownredwhite = 1kohm/1.8kohm div // DIN15
#define speedPin BOARD_MAX_IO_PINS - 1 // voltage div 12/3 <-> black <-> blueyellowblue = 1kohm/380ohm div // DIN10
#define rpmPin BOARD_MAX_IO_PINS - 1 // voltage div 12/3 <-> black <-> whitebluewhite = 1kohm/380ohm div // DIN12
#define batteryPin BOARD_MAX_IO_PINS - 1 // car battery monitor
#define refPin BOARD_MAX_IO_PINS - 1 // sensor voltage ref ?
//#define refPin A13 // using this in exhaustTempPin for now.

#define exhaustTemperatureCS BOARD_MAX_IO_PINS - 1
#define displayCS BOARD_MAX_IO_PINS - 1

#endif // GLOBALS_H
