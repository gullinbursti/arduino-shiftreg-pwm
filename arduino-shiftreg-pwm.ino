

//#define ENCODER_DO_NOT_USE_INTERRUPTS


#include <Encoder.h>

#include "SevSegShift.h"
#include "ShiftRegisterPWM.h"
//#include ".h"

#define PWM_DATA_PIN  2
#define PWM_CLOCK_PIN 3
#define PWM_LATCH_PIN 4

#define SEG_DATA_PIN  5
#define SEG_CLOCK_PIN 6
#define SEG_LATCH_PIN 7

#define ROTOR_TURN_PIN_1 8
#define ROTOR_TURN_PIN_2 9
#define ROTOR_BTN_PIN A2

#define ORTHODOX_PWM 8.00
#define ORTHODOX_POS 248


bool btnUp = true;
int curPos = ORTHODOX_POS;

float seqInterval = ORTHODOX_PWM;

Encoder digiPot(ROTOR_TURN_PIN_1, ROTOR_TURN_PIN_2);
ShiftRegisterPWM sr(1, 24);
SevSegShift sevseg(SEG_DATA_PIN, SEG_CLOCK_PIN, SEG_LATCH_PIN);



int clampInt(int val, int lower, int upper) {
  if (val < lower) {
    return (lower);

  } else if (val > upper) {
    return (upper);

  } else {
    return (val);
  }
}

void displayDigits(int val) {
  sevseg.setNumber(val, 4);
  sevseg.refreshDisplay();
}


void setup() {
  Serial.begin(9600);
  
  pinMode(ROTOR_BTN_PIN, INPUT_PULLUP);
//  pinMode(ROTOR_BTN_PIN, INPUT);

  pinMode(PWM_DATA_PIN, OUTPUT);
  pinMode(PWM_CLOCK_PIN, OUTPUT);
  pinMode(PWM_LATCH_PIN, OUTPUT);

  pinMode(SEG_DATA_PIN, OUTPUT);
  pinMode(SEG_CLOCK_PIN, OUTPUT);
  pinMode(SEG_LATCH_PIN, OUTPUT);

//  sr.interrupt(ShiftRegisterPWM::UpdateFrequency::VerySlow);
//  sr.interrupt(ShiftRegisterPWM::UpdateFrequency::Slow);
  sr.interrupt(ShiftRegisterPWM::UpdateFrequency::Medium);
//  sr.interrupt(ShiftRegisterPWM::UpdateFrequency::Fast);
//  sr.interrupt(ShiftRegisterPWM::UpdateFrequency::SuperFast);


  digiPot.write(ORTHODOX_POS);
  byte numDigits = 4;
  byte digitPins[] = {8 + 2, 8 + 5, 8 + 6, 2};
  byte segmentPins[] = {8 + 3, 8 + 7, 4, 6, 7, 8 + 4, 3, 5};
  bool resistorsOnSegments = true;
  byte hardwareConfig = COMMON_CATHODE;
  bool updateWithDelays = false;
  bool leadingZeros = true;
  bool disableDecPoint = false;

  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments, updateWithDelays, leadingZeros, disableDecPoint);
  sevseg.setBrightness(80);
  sevseg.blank();

//  Serial.print("curPos [");
//  Serial.print(curPos);
//  Serial.print("]");
//  Serial.print(" =-= ");
//  Serial.print("seqInterval [");
//  Serial.print(seqInterval);
//  Serial.print("]");
//  Serial.println();
}

void loop() {

//  Serial.print(F("maxBrite [");
//  Serial.print(maxBrite);
//  Serial.print(F("]");
//  Serial.print(F(" =-= ");
//  Serial.println();
//
//  Serial.println(digitalRead(ROTOR_BTN_PIN));

  digiPot.write(clampInt(digiPot.read(), 0, 320 * 4));
//  long updPos = clampInt(digiPot.read(), 0, 1280);
  long updPos = digiPot.read();
//  if (curPos != updPos && (curPos % 4 == 0 || updPos % 4 == 0)) {
  if (curPos != updPos && updPos % 4 == 0) {
    int dir = float((abs(curPos) - abs(updPos)) * 0.25) * -1;

//    Serial.print("curPos [");
//    Serial.print(curPos);
//    Serial.print("]");
//    Serial.print(" =-= ");
//    Serial.print("updPos [");
//    Serial.print(updPos);
//    Serial.print("]");
//    Serial.print(" =-= ");

//    if (abs(diff) == 3) {
    if (dir != 0) {
      float inc = float(dir * 0.125);// + float(updPos * 0.25);

//      Serial.print("dir [");
//      Serial.print(dir);
//      Serial.print("]");
//      Serial.print(" =-= ");
//      Serial.print("inc [");
//      Serial.print(inc);
//      Serial.print("]");
//      Serial.print(" =-= ");
      
//      seqInterval += ((diff > 0) ? 0.125 : -0.125);// + ((-320 - updPos) * 0.1);
      seqInterval = min(max(0.125, seqInterval + inc), 40.00);
//      seqInterval = min(max(0.125, seqInterval + (dir * float(updPos * 0.0625))), 180.00);

//      Serial.print("seqInterval [");
//      Serial.print(seqInterval);
//      Serial.print("]");
//      Serial.print(" =-= ");
    }

    curPos = updPos;

//    int seqPercent = (float(curPos * 0.25) / 320) * 100;
    
//    Serial.print("seqPercent [");
//    Serial.print(seqPercent);
//    Serial.print("]");
//    Serial.println();
  }
  
  
  if (digitalRead(ROTOR_BTN_PIN) == LOW && btnUp) {
    Serial.println(F("BUTTON DN"));
    btnUp = false;
  }

  if (digitalRead(ROTOR_BTN_PIN) == HIGH && !btnUp) {
    Serial.println(F("BUTTON UP"));

    curPos = ORTHODOX_POS;
    digiPot.write(curPos);
    seqInterval = ORTHODOX_PWM;
    
    btnUp = true;
  }

  for (uint8_t i=0; i<8; i++) {
//    uint8_t val = (uint8_t)(((float)sin(millis() / 150.0 + i / 4.0 * 2.0 * PI) + 1) * 128);
    uint8_t val = (uint8_t)(((float)sin(millis() / 150.0 + i / seqInterval * 2.0 * PI) + 1) * 128);
    sr.set(i, val);
  }

//  displayDigits(int(millis() * float(0.025)) % 10000);
//  displayDigits(int(((320 - curPos) / 320) * 100));
//   displayDigits(int(float(curPos / (320 * 4)) * 100) * 10);
    displayDigits(float(curPos * 0.25));
}
