#include "include/globals.h"

void pollstick();
void gearUp();
void gearDown();
void polltrans();
void boostControl();
void fuelControl();
int adaptSPC(int mapId, int xVal, int yVal);
void adaptSPCup();
void adaptSPCdown();
void radioControl();
void onReleased(unsigned long pressTimespanMs);

void keypadWatch();
void keypadControl(int command);

extern int spcPercentVal;       
extern unsigned long int shiftStartTime;
extern unsigned long int shiftDuration;
extern bool trans;
extern bool boostSensor;
extern bool shiftBlocker;
extern bool debugEnabled;
extern bool boostLimit;
extern int n2Speed;
extern int n3Speed;
extern bool fullAuto;
extern bool fuelPumpControl;
extern bool shiftPending;
extern bool ignition;
extern bool fuelPumps;
extern bool radioEnabled;
extern bool stickCtrl;
extern bool horn;
extern bool manual;
extern bool truePower;
extern bool tccLock;
extern bool stick;
extern bool autoGear;
extern bool tpsConfigMode;
extern double lastShiftPoint;
extern bool tpsInitPhase1, tpsInitPhase2;
extern byte page;
extern double boostPWM;
extern bool boostLimitShift;
extern int boostOverride;
extern double lastPress;

extern double canbusTPS;
extern double canbusRPM;
extern double canbusCoolant;
extern double canbusSpeed;