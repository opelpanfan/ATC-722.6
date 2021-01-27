#include "include/timers.h"
#include "include/globals.h"


void initialiseTimers()
{
  loop10ms = 0;
  loop20ms = 0;
  loop33ms = 0;
  loop66ms = 0;
  loop100ms = 0;
  loop250ms = 0;
  loopSec = 0;
}

  void oneMSInterval(){
  ms_counter++;

  //Increment Loop Counters
  loop10ms++;
  loop20ms++;
  loop33ms++;
  loop66ms++;
  loop100ms++;
  loop250ms++;
  loopSec++;

  //60Hz loop
  if (loop10ms == 10)
  {
    loop10ms = 0;
    BIT_SET(TIMER_mask, BIT_TIMER_60HZ);
  }
  //50Hz loop
  if (loop20ms == 20)
  {
    loop20ms = 0;
    BIT_SET(TIMER_mask, BIT_TIMER_50HZ);
  }
  //30Hz loop
  if (loop33ms == 33)
  {
    loop33ms = 0;
    BIT_SET(TIMER_mask, BIT_TIMER_30HZ);
  }

  //15Hz loop
  if (loop66ms == 66)
  {
    loop66ms = 0;
    BIT_SET(TIMER_mask, BIT_TIMER_15HZ);
  }

  if (loop100ms == 100)
  {
    loop100ms = 0; //Reset counter
    BIT_SET(TIMER_mask, BIT_TIMER_10HZ);
  }
  if (loop250ms == 250)
  {
    loop250ms = 0; //Reset Counter
    BIT_SET(TIMER_mask, BIT_TIMER_4HZ);
  }

  if (loopSec == 1000)
  {
    loopSec = 0;
    BIT_SET(TIMER_mask, BIT_TIMER_1HZ);
    mainLoopCount = 0;
  }
}
