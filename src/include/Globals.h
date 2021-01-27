#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

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
#define BIT_TIMER_60HZ            6

volatile unsigned long ms_counter = 0;
volatile uint16_t mainLoopCount;

volatile uint8_t TIMER_mask;
volatile uint8_t LOOP_TIMER;


#endif // GLOBALS_H
