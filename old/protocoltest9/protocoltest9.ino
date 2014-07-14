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
const char UNITY_HAS_POSITION = 'r';

const char LED_PIN = 13;

int STATE;
int left_to_read;

//short unsigned int theNum;
unsigned int encread;
unsigned short int translated;
char serialChar;
unsigned short int u_position;
unsigned short int temp_u_position;
bool done;

Encoder* enc;

void setup() {
  Serial.begin(115200);
  enc = new Encoder(CSn, CLK, DO);
  encread = 0;
  translated = 0;
  u_position = 0;
  STATE = 0;
  left_to_read = 0;
  done = false;
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  encread = enc->read();
  
  // Maintain encoder reading within bounds
  if (encread > HIGHPOS) {
    encread = HIGHPOS;
  } else if (encread < LOWPOS) {
    encread = LOWPOS;
  }
  
  // If we're beyond the avatar...
  if (encread > u_position)
  {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
  
  // If there's a character to processs
  if (Serial.available())
  {
    serialChar = Serial.read();
    
    // PROCESS THAT CHARACTER!!
    if (STATE == 1) // Interpreting as new position announcement
    {
      left_to_read--;
      temp_u_position += (serialChar << (8 * left_to_read));
      
      if (left_to_read <= 0)
      {
        u_position = unityToEncoder(temp_u_position);
        temp_u_position = 0; // Redundant
        STATE = 0;
        //Serial.println();
        //Serial.print(u_position);
      }
    }
    
    else {    
      switch(serialChar) {
        case UNITY_WANTS_POSITION:
          translated = encoderToUnity(encread);
          Serial.write(highByte(translated));
          Serial.write(lowByte(translated));
          break;
        case UNITY_HAS_POSITION:
          STATE = 1;
          temp_u_position = 0;
          left_to_read = 2;
          break;
      }
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

unsigned short int encoderToUnity(int enc_reading)
{
  return (unsigned short int)(MAXVAL * (enc_reading - LOWPOS) / (HIGHPOS - LOWPOS));
}

int unityToEncoder(unsigned short int unity_reading)
{
  return LOWPOS + ((HIGHPOS - LOWPOS) * unity_reading / MAXVAL);
}
