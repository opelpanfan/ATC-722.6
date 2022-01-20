#ifndef Timers_H
#define Timers_H

#include "globals.h"

volatile byte loop1ms;
volatile byte loop5ms;
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
