
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <Wire.h>
#include <SPI.h>
#include "PID_v1.h"
#include "probe.h"
#include "fridge.h"
#include "globals.h"
#define DEBUG true  // debug flag for including debugging code

void mainUpdate();  // update sensors, PID output, fridge state, write to log, run profiles
void initDisplay();    // print static characters to LCD (lables, scrollbar, page names, etc)
void updateDisplay();  // print dynamic characters to LCD (PID/temp values, etc)
void tempUnit();     // temperature display units C/F

#if DEBUG == true
  int freeRAM();  // approximate free SRAM for debugging
#endif

void setup() {

  pinMode(relay1, OUTPUT);  // configure relay pins and write default HIGH (relay open)
    digitalWrite(relay1, HIGH);
  pinMode(relay2, OUTPUT);
    digitalWrite(relay2, HIGH);

  encoderPos = 0;
  encoderState = 0b000;


  #if DEBUG == true  //start serial at 9600 baud for debuging
    Serial.begin(9600);
  #endif

  Wire.begin();              // initialize rtc communication
  lcd.createChar(0, (uint8_t*)delta);  // create custom characters for LCD (slots 0-7)
  lcd.createChar(1, (uint8_t*)rightArrow);
  lcd.createChar(2, (uint8_t*)disc);
  lcd.createChar(3, (uint8_t*)circle);
  lcd.createChar(4, (uint8_t*)dot);
  lcd.createChar(5, (uint8_t*)inverted);
  lcd.createChar(6, (uint8_t*)degc);
  lcd.createChar(7, (uint8_t*)degf);

  lcd.begin(16,2);


  delay(1500);

  fridge.init();
  beer.init();

  mainPID.SetTunings(Kp, Ki, Kd);    // set tuning params
  mainPID.SetSampleTime(1000);       // (ms) matches sample rate (1 hz)
  //mainPID.SetOutputLimits(0.3, 38);  // deg C (~32.5 - ~100 deg F)
  mainPID.SetOutputLimits(10, 30);  // deg C (~32.5 - ~100 deg F)
  //if (programState & MAIN_PID_MODE) mainPID.SetMode(AUTOMATIC);  // set man/auto
  //  else mainPID.SetMode(MANUAL);
  mainPID.SetMode(AUTOMATIC);
  mainPID.setOutputType(FILTERED);
  mainPID.setFilterConstant(10);
  mainPID.initHistory();

  heatPID.SetTunings(heatKp, heatKi, heatKd);
  heatPID.SetSampleTime(heatWindow);       // sampletime = time proportioning window length
  heatPID.SetOutputLimits(0, heatWindow);  // heatPID output = duty time per window
  //if (programState & HEAT_PID_MODE) heatPID.SetMode(AUTOMATIC);
  //  else heatPID.SetMode(MANUAL);
  heatPID.SetMode(AUTOMATIC);
  heatPID.initHistory();

  encoderPos = 0;  // zero rotary encoder position for main loop

  Serial.print(Setpoint);
  Serial.println(F(" Grados"));



  #if DEBUG == true
    Serial.print(F("init complete. "));
    Serial.print(millis());
    Serial.print(F("ms elapsed. "));
  wdt_enable(WDTO_8S);  // enable watchdog timer with 8 second timeout (max setting)
                        // wdt will reset the arduino if there is an infinite loop or other hangup; this is a failsafe device
  initDisplay();

  Setpoint = 18.00 ;

    Serial.print(F("Temp Objetivo SetPoint: "));
    Serial.print(freeRAM());
    Serial.println(F(" bytes free SRAM remaining"));
  #endif
}

void loop() {
  wdt_reset();                      // reset the watchdog timer (once timer is set/reset, next reset pulse must be sent before timeout or arduino reset will occur)
  static char listSize = 4;         // for constraining rotary encoder position
  static char lastReportedPos = 1;  // default to 1 to force display initialze on first iteration
  encoderState |= DEBOUNCE;         // reset rotary debouncer

  updateDisplay();                       // update display data
  mainUpdate();                            // subroutines manage their own timings, call every loop

}

void mainUpdate() { // call all update subroutines
  probe::startConv();                            // start conversion for all sensors
  if (probe::isReady()) {                        // update sensors when conversion complete
    fridge.update();
    beer.update();
    Input = beer.getFilter();
  }
  if (programState ) ;  // update main Setpoint if fermentation profile active
  mainPID.Compute();                             // update main PID
  updateFridge();                                // update fridge status
}

void initDisplay() {
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print(F("Tf="));
  lcd.setCursor(8, 1);
  lcd.print(F("Tb="));
  lcd.setCursor(0, 0);
  lcd.print(F("SP="));
}

void updateDisplay() {
  lcd.setCursor(9, 0);
  if (programState & DISPLAY_UNIT) lcd.write((byte)7);
    else lcd.write((byte)6);
  if (programState & TEMP_PROFILE) lcd.print(F(" PGM "));
  else {
    if (programState & MAIN_PID_MODE) lcd.print(F("A"));
      else lcd.print(F("M"));
    if (programState & HEAT_PID_MODE) lcd.print(F("A"));
      else lcd.print(F("M"));
  }
  if (getFridgeState(0) == IDLE) lcd.print(F("I"));
  if (getFridgeState(0) == HEAT) lcd.print(F("H"));
  if (getFridgeState(0) == COOL) lcd.print(F("C"));

  // temperature units = deg C
        lcd.setCursor(3, 1);
        lcd.print(fridge.getTemp());
        lcd.setCursor(11, 1);
        lcd.print(beer.getTemp());
        lcd.setCursor(3, 0);
        lcd.print(Setpoint);
}

void tempUnit() {
  char listSize = 2;
  encoderPos = (programState & DISPLAY_UNIT) >> 3;
  char lastReportedPos = encoderPos + 1;
  lcd.setCursor(0, 2);
  lcd.print(F("                    "));
  lcd.setCursor(2, 2);
  lcd.write((byte)1);
  lcd.print(F(" \337"));
  do {
    wdt_reset();
    mainUpdate();
    encoderState |= DEBOUNCE;
    if (lastReportedPos != encoderPos) {
      encoderPos = (encoderPos + listSize) % listSize;
      lcd.setCursor(5, 2);
      if (encoderPos) lcd.print(F("F"));
        else lcd.print(F("C"));
      lastReportedPos = encoderPos;
    }
  } while (digitalRead(pushButton));
  if (encoderPos) programState |= DISPLAY_UNIT;
    else programState &= ~DISPLAY_UNIT;

  #if DEBUG == true
    if (encoderPos) Serial.print(F("Units set to deg F."));
      else Serial.print(F("Units set to deg C."));
    Serial.print(freeRAM());
    Serial.println(F(" bytes free SRAM remaining"));
  #endif
}


#if DEBUG == true
int freeRAM() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
#endif
