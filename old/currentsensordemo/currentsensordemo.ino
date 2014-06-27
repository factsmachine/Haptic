#include <Servo.h>
Servo serv;

signed int reading = 0;

void setup()
{
  Serial.begin(115200);
  serv.attach(10, 1000, 2000);
  serv.writeMicroseconds(1800);
}
 
void loop()
{
  int scale = 1;
  reading = scale * (analogRead(3) - 512);
  Serial.println(reading);
  delay(100); // Tcs waiting for another read in
}
