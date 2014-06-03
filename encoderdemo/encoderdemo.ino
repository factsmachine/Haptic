#include "encoder.h"

unsigned int reading = 0;
Encoder enc(A2, A1, A0);

void setup()
{
  Serial.begin(115200);
}
 
void loop()
{
  reading = enc.read();
  Serial.println(reading);
  delay(100); // Tcs waiting for another read in
}
