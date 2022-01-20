#include "include/globals.h"

void switchGearStart(int cSolenoid, int spcVal, int mpcVal);
void switchGearStop();
void gearchangeUp(int newGear);
void gearchangeDown(int newGear);
void decideGear();
int evaluateGear();
float ratioFromGear(int inputGear);
int gearFromRatio(float inputRatio);
float getGearSlip();
void doPreShift();
void doShift();
void doPostShift();
void faultMon();

extern byte gear;         
extern int cSolenoid;  
extern int spcSetVal;
extern int spcPercentVal,mpcPercentVal;
extern unsigned long int shiftStartTime;
extern unsigned long int shiftDuration;
extern int cSolenoidEnabled;
extern int lastMapVal,lastXval,lastYval;
extern bool trans;
extern bool sensors;
extern bool shiftBlocker;
extern bool debugEnabled;
extern byte wantedGear;
extern byte newGear;
extern byte pendingGear;
extern bool shiftPending;
extern int vehicleSpeedRevs;
extern bool speedFault;
extern bool adaptive;
extern bool evalGear;
extern int shiftLoad, shiftAtfTemp, wrongGearPoint;
extern bool shiftConfirmed, preShift, postShift, preShiftDone, postShiftDone, shiftDone, batteryFault, slipFault, carRunning;