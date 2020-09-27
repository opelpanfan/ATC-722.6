#include <Arduino.h>
#include <SoftTimer.h>

extern boolean boostSensor;
extern boolean debugEnabled;
extern boolean boostLimit;
extern boolean fullAuto;
extern boolean fuelPumpControl;
extern boolean radioEnabled;
extern boolean stickCtrl;
extern boolean horn;
extern boolean manual;
extern boolean truePower;
extern boolean tccLock;
extern boolean stick;
extern boolean autoGear;
extern boolean adaptive;
extern boolean evalGear;
extern boolean tpsSensor;
extern boolean boostSensor;
extern boolean debugEnabled;
extern boolean boostLimit;
extern boolean diffSpeed;
extern boolean rpmSpeed;
extern boolean batteryMonitor;
extern boolean exhaustPresSensor;
extern boolean datalogger;
extern boolean w124speedo;
extern boolean w124rpm;
extern boolean exhaustTempSensor;
extern boolean boostLimit;
extern boolean boostLimitShift;
extern boolean resistiveStick;
extern struct ConfigParam config;

extern boolean useCanSensors;

void initConfig();
void pollConfigMode();
void getFeatures();
void getGears();
void setFeatures(int asset, int value);
void getConfig();
void setConfig(int asset, int value);
void setUpGear(int asset, int value);
void setDownGear(int asset, int value);
void setConfigFloat(int asset, float fvalue);
void serialConfig();
void serialWatch(Task* me);
