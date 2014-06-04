#include "encoder.h"

Encoder enc(A2, A1, A0);
unsigned int INA = 9;
unsigned int INB = 8;
unsigned int PW = 10;
unsigned int CS = 3;

unsigned int KP = 15;
signed int setpoint = 0;

signed int CSoffset = -10;
//PWMLimit = 255;

signed int effort;
signed int error;

void setup() {
  effort = 0;
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(PW, OUTPUT);
  
  // Set PWM properties
  TCCR1B = TCCR1B & 0b11111000 | 0x01;
  TCCR2B = TCCR2B & 0b11111000 | 0x01;
}

void loop() {
  unsigned int encread = enc.read();
  unsigned int csread = (analogRead(A3) - 512 - CSoffset);
  error = csread - setpoint;
  
  if (encread > 1800 || encread < 200) {
    setEffort(0);
  }
  else {
    setEffort(error * KP);
  }
  delayMicroseconds(100);
}

void setEffort(signed int effortSetting)
{
  effort = effortSetting;
  effort = min(255, max(-255, effort));
  
  if (effortSetting >= 0) {
    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
    analogWrite(PW, min(255, max(-255, effort)));
  }
  else {
    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
    analogWrite(PW, min(255, max(-255, -effort)));
  }
}

signed int getEffort()
{
  return effort;
}
