
#include <Arduino.h>

struct SensorVals
{
    int curSpeed, curOilTemp, curExTemp, curBoost, curExPres, curAtfTemp, curRPM, curBoostLim, curEvalGear, curBattery, fuelUsed, fuelUsedAvg, curLambda;
    byte curTps, curLoad;
    float curSlip, curRatio, curPresDiff;
};

void N2SpeedInterrupt();
void N3SpeedInterrupt();
void vehicleSpeedInterrupt();
void rpmInterrupt();
void fuelInInterrupt();
void fuelOutInterrupt();
void pollsensors();
int speedRead();
int tpsRead();
void tpsInit(int action);
int rpmRead();
int lambdaRead();
int boostRead();
int exhaustPressureRead();
int exhaustTempRead();
int boostLimitRead(int oilTemp, int tps);
int loadRead(int curTps, int curBoost, int curBoostLim, int curRPM);
int atfRead();
int oilRead();
int batteryRead();
struct SensorVals readSensors();

extern bool trans;
extern bool sensors;
extern bool tpsSensor;
extern bool boostSensor;
extern bool debugEnabled;
extern bool boostLimit;
extern bool drive;
extern bool diffSpeed;
extern bool rpmSpeed;
extern bool useCanSpeed;
extern bool batteryMonitor, exhaustPresSensor, carRunning;
extern int evalGearVal, speedoRPM;