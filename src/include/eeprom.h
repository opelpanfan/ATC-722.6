#include <Arduino.h>

int readEEPROM(int mapId, int xVal, int yVal);
void writeEEPROM(int mapId, int xVal, int yVal, int modVal);
void resetEEPROM();
extern bool debugEnabled;