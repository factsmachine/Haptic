#include "encoder.h"

Encoder enc(A2, A1, A0);
unsigned int INA = 9;
unsigned int INB = 8;
unsigned int PW = 10;

unsigned int KP = 1;
signed int effort;

unsigned int setpoint = 1000;
signed int error;

void setup() {
  effort = 0;
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(PW, OUTPUT);
  
  // Set PWM properties
  //TCCR1B = TCCR1B & 0b11111000 | 0x01;
  //TCCR2B = TCCR2B & 0b11111000 | 0x01;
}

void loop() {
  unsigned int encread = enc.read();
  error = encread - setpoint;
  
  if (encread > 1800 || encread < 200) {
    setEffort(0);
  }
  else {
    setEffort(error * KP / 30);
  }
  delayMicroseconds(100);
}

void setEffort(signed int effortSetting)
{
  effort = effortSetting;
  effort = min(50, max(-50, effort));
  if (effortSetting >= 0) {
    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
    analogWrite(PW, effort);
  }
  else {
    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
    analogWrite(PW, -effort);
  }
}

signed int getEffort()
{
  return effort;
}
