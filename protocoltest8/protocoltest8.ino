#include <encoder.h>

// Dead-zone limits
const int LOWPOS = 1110;
const int HIGHPOS = 3160;
const int MAXVAL = 65535;
// Encoder
const int CSn = 51; // Encoder chip select pin
const int CLK = 52; // Encoder clock signal pin
const int DO = 53;  // Encoder digital output pin

const char UNITY_WANTS_POSITION = 'p';
const char UNITY_HAS_POSITION = 'P';

//short unsigned int theNum;
unsigned int encread;
unsigned short int translated;
char serialChar;
unsigned short int u_position;

Encoder* enc;

void setup() {
  Serial.begin(19200);
  enc = new Encoder(CSn, CLK, DO);
  encread = 0;
  translated = 0;
  u_position = 0;
}

void loop() {
  encread = enc->read();
  
  if (encread > HIGHPOS) {
    encread = HIGHPOS;
  } else if (encread < LOWPOS) {
    encread = LOWPOS;
  }
  
  translated = (unsigned short int)(MAXVAL * (encread - LOWPOS) / (HIGHPOS - LOWPOS));
  
  if (Serial.available())
  {
    serialChar = Serial.read();
    
    switch(serialChar) {
      case UNITY_WANTS_POSITION:
        Serial.write(highByte(translated));
        Serial.write(lowByte(translated));
        break;
      //case UNITY_HAS_POSITION:
      //  u_position = Serial.read
      //  break;
    }
  }
  
  //Serial.write(UNITY_POSITION_REQUEST);
  
  //Serial.println(translated, DEC);
  /*Serial.write('p');
  Serial.write(highByte(translated));
  Serial.write(lowByte(translated));
  Serial.write('\r');
  Serial.write('\n');*/
  
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


