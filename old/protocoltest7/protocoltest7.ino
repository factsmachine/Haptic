#include <encoder.h>

// Dead-zone limits
const int LOWPOS = 1110;
const int HIGHPOS = 3160;

// Encoder
const int CSn = 51; // Encoder chip select pin
const int CLK = 52; // Encoder clock signal pin
const int DO = 53;  // Encoder digital output pin

//short unsigned int theNum;
unsigned int encread;
byte translated;



Encoder* enc;

void setup() {
  Serial.begin(19200);
  enc = new Encoder(CSn, CLK, DO);
  encread = 0;
  translated = 0;
}

void loop() {
  encread = enc->read();
  
  if (encread > HIGHPOS) {
    encread = HIGHPOS;
  } else if (encread < LOWPOS) {
    encread = LOWPOS;
  }
  
  translated = (byte)(255 * (encread - LOWPOS) / (HIGHPOS - LOWPOS));
  
  //Serial.println(translated, DEC);
  Serial.write(translated);
  
  delay(10);
  //delay(200);
  //delay(5);
  //if (Serial.available())
  //{
  //  theNum = Serial.read();
    //Serial.println(theChar);
  //  Serial.write(theNum);
  //}
}
