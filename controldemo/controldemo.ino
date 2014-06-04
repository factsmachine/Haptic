#include <Servo.h>
#include "encoder.h"
Servo serv;
Encoder enc(A2, A1, A0);

unsigned int KP = 8;
signed int setpoint = 20;
signed int error;

signed int reading = 0;

void setup()
{
  Serial.begin(115200);
  serv.attach(10, 1000, 2000);
}
 
void loop()
{
  unsigned int encread = enc.read();
  if (encread > 1500) {
    setpoint = 0;
  }
  else {
    setpoint = 20;
  }
  
  int scale = 1;
  reading = scale * (analogRead(3) - 512);
  
  error = reading - setpoint;
  serv.writeMicroseconds(1500 + KP * error);

  Serial.println(reading);
  delayMicroseconds(100); // Tcs waiting for another read in
}
