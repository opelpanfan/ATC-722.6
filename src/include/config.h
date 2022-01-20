#include "include/globals.h"
struct ConfigParam
{
    int boostMax, boostDrop, boostSpring, fuelMaxRPM, maxRPM, tireWidth, tireProfile, tireInches, rearDiffTeeth, nextShiftDelay, stallSpeed, batteryLimit, firstTccGear, triggerWheelTeeth, tpsAgre, highRPMshiftLimit, lowRPMshiftLimit;
    float diffRatio, transferRatio, maxSlip, transSloppy;
    int oneTotwo, twoTothree, threeTofour, fourTofive, fiveTofour, fourTothree, threeTotwo, twoToone;
};

extern struct ConfigParam config;