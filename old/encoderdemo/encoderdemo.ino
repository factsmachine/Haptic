#include "encoder.h"

unsigned int reading = 0;
Encoder* enc; 

void setup()
{
  enc = new Encoder(51, 52, 53);
  Serial.begin(115200);
}
 
void loop()
{
  reading = enc->read();
  Serial.println(reading);
  delay(100); // Tcs waiting for another read in
}
