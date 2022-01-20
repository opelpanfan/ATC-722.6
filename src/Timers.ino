#include "include/timers.h"
	
void initialiseTimers()
{
  loop1ms = 0;
  loop5ms = 0;
  loop10ms = 0;
  loop20ms = 0;
  loop33ms = 0;
  loop66ms = 0;
  loop100ms = 0;
  loop250ms = 0;
  loopSec = 0;

 Timer11.setOverflow(1000, MICROSEC_FORMAT);  // Set up period
      #if ( STM32_CORE_VERSION_MAJOR < 2 )
      Timer11.setMode(1, TIMER_OUTPUT_COMPARE);
      Timer11.attachInterrupt(1, oneMSInterval);
      #else
      Timer11.attachInterrupt(oneMSInterval);
      #endif
      Timer11.resume(); //Start Timer
}

void oneMSInterval()
{
  ms_counter++;

  //Increment Loop Counters
  loop1ms++;  //1000Hz
  loop5ms++;  //200Hz
  loop10ms++; //100Hz
  loop20ms++; //50Hz
  loop33ms++; //30Hz
  loop66ms++; //15Hz
  loop100ms++;//10Hz
  loop250ms++;//4Hz
  loopSec++;  //1Hz

  //1KHz loop
  if (loop1ms == 1)
  {
    loop1ms = 0;
    BIT_SET(TIMER_mask, BIT_TIMER_1KHZ);
  }
  //200Hz loop
  if (loop5ms == 5)
  {
    loop5ms = 0;
    BIT_SET(TIMER_mask, BIT_TIMER_200HZ);
  }
  //100Hz loop
  if (loop10ms == 10)
  {
    loop10ms = 0;
    BIT_SET(TIMER_mask, BIT_TIMER_100HZ);
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
