#ifndef GLOBALS_H
#define GLOBALS_H

const int EEPROM_VER = 11;  // eeprom data tracking

// custom characters for LCD
const byte delta[8] = {
  B00000,
  B00000,
  B00000,
  B00100,
  B01110,
  B11111,
  B11111,
  B00000
};

const uint8_t rightArrow[8] = {
  B11000,
  B10100,
  B10010,
  B10001,
  B10010,
  B10100,
  B11000,
  B00000
};

const byte disc[8] = {
  B01110,
  B11111,
  B11111,
  B11111,
  B01110,
  B00000,
  B00000,
  B00000
};

const byte dot[8] = {
  B00000,
  B00000,
  B00100,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000
};

const byte circle[8] = {
  B01110,
  B10001,
  B10001,
  B10001,
  B01110,
  B00000,
  B00000,
  B00000
};

const byte inverted[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};

const byte degc[8] = {
  B01000,
  B10100,
  B01000,
  B00011,
  B00100,
  B00100,
  B00011,
  B00000
};

const byte degf[8] = {
  B01000,
  B10100,
  B01000,
  B00011,
  B00100,
  B00111,
  B00100,
  B00000
};

struct profileStep {  // struct to encapsulate temperature and duration for fermentation profiles
  double temp;
  double duration;
  profileStep() : temp(0), duration(0) {}
};

// arduino pin declarations:
const byte encoderPinA = 3;   // rotary encoder A channel **interrupt pin**
const byte encoderPinB = 2;   // rotary encoder B channel **interrupt pin**
const byte chipSelect = 10;   // data logging shield
const byte pushButton = A0;   // rotary encoder pushbutton
const byte onewireData = 2;  // one-wire data
const byte relay1 = 8;       // relay 1 (fridge compressor)
const byte relay2 = 7;       // relay 2 (heating element)

volatile char encoderPos;    // a counter for the rotary encoder dial
volatile byte encoderState;  // 3 bit-flag encoder state (A Channel)(B Channel)(is rotating)
#define CHAN_A   0b100
#define CHAN_B   0b010
#define DEBOUNCE 0b001

OneWire onewire(onewireData);  // declare instance of the OneWire class to communicate with onewire sensors
probe beer(&onewire), fridge(&onewire);

byte programState;  // 6 bit-flag program state -- (mainPID manual/auto)(heatPID manual/auto)(temp C/F)(fermentation profile on/off)(data capture on/off)(file operations) = 0b000000
#define MAIN_PID_MODE 0b100000
#define HEAT_PID_MODE 0b010000
#define DISPLAY_UNIT  0b001000
#define TEMP_PROFILE  0b000100
#define DATA_LOGGING  0b000010
#define FILE_OPS      0b000001

double Input, Setpoint, Output;  // SP, PV, CO, tuning params for main PID
// double Ki = 2 ;
// double Kp = 2 ;
// double Kd = 2 ;
// double Ki = 1 ;
// double Kp = 1 ;
// double Kd = 1 ;
 double Ki = 3 ;
 double Kp = 1 ;
 double Kd = 1 ;
double heatInput, heatOutput, heatSetpoint, heatKp, heatKi, heatKd;  // SP, PV, CO tuning params for HEAT PID
PID mainPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);  // main PID instance for beer temp control (DIRECT: beer temperature ~ fridge(air) temperature)
PID heatPID(&heatInput, &heatOutput, &heatSetpoint, heatKp, heatKi, heatKd, DIRECT);   // create instance of PID class for cascading HEAT control (HEATing is a DIRECT process)

LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
//QueueList <profileStep> profile;   // dynamic queue (FIFO) linked list; contains steps for temperature profile

#endif
