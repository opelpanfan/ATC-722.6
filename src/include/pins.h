#define TEENSY // we're running this on TEENSY

// Outputs pins
#define y3 4
#define y4 3
#define y5 4
#define mpc 6
#define spc 5
#define tcc 9
#define speedoCtrl 10 
#define rpmMeter 26
#define hornPin 31
#define reversePin 22 //reverse light digital pin

// Shifter input pins
#define whitepin 29
#define bluepin 27
#define greenpin 30
#define yellowpin 28

// Switches
#define autoSwitch 12 // auto-manual switch W/S
#define gdownSwitch 17 //gear down paddle manual
#define gupSwitch 16 //gear up paddle manual
#define lowGearPin  18 // TransferCase lowrange switch pin, +12V

// Resistive keypad
#define keypadPin  19 //54 // Resistive keys pin, analog pin

// Car sensor input pins
#define tpsPin A1
#define atfPin A0
#define oilPin A9
#define n2pin 24
#define n3pin 25
#define speedPin 33
#define rpmPin 32
#define refPin A6

// Unused pins with Teensy 4.0
     
      #define batteryPin A6 //A7 // car battery monitor
      //#define refPin A13 // using this in exhaustTempPin for now.
      #define boostPin 46 //A11 // voltage div 5/3 <-> black <-> blue-brown-blue = 1kohm/1.8kohm div // ANAIN4, tps? A2->A10
      #define exhaustPresPin 47 // A12, A9 used in coupe.
      #define exhaustTemperatureCS 48
      #define boostCtrl 49 // FMT1, green <-> green/white/yellow // DOUT8, 3?
      #define fuelPumpCtrl 50 // missing // DOUT9, 13?
      #define displayCS 51
      #define fuelInPin 52
      #define injectionPin 53 // should be 16.
      #define exhaustTempPin 54
      #define gupSwitchalt 55
