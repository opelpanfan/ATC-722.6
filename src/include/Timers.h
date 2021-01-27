#ifndef Timers_H
#define Timers_H
#include <Arduino.h>


volatile byte loop10ms;
volatile byte loop20ms;
volatile byte loop33ms;
volatile byte loop66ms;
volatile byte loop100ms;
volatile byte loop250ms;
volatile int loopSec;

void oneMSInterval();
void initialiseTimers();

#endif
