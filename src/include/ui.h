#include <Arduino.h>

void draw(int wantedGear, int loopTime);
void rpmMeterUpdate();
void updateDisplay();
void datalog();
void updateSpeedo();

extern byte gear;         
extern bool debugEnabled;
extern bool fullAuto;
extern bool datalogger;
extern bool w124speedo;
extern bool w124rpm;
extern bool infoBoost;
extern byte wantedGear,page;
extern int lastMapVal;
extern bool shiftPending,truePower;
extern float ratio;
extern int n2Speed, n3Speed;
extern float gearSlip;
extern unsigned long fuelIn, fuelOut;
#ifdef __arm__
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__