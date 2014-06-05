#include "encoder.h"
#include "PID_lite.h"
#include <math.h> 


// Good KP: 0.4
// Good KI: 0.1
// Good target: 330

// Command defines
const char COMGO = 'a';
const char COMSTOP = 'b';
const char COMRAMP = 'c';
const char SETKP = 'p';
const char SETKI = 'i';
const char SETKD = 'd';
const char SETTARGET = 't';
const char SESAT = 's';
// Pin assignments
const unsigned int INA = 9;
const unsigned int INB = 8;
const unsigned int PW = 10;
const unsigned int CS = A3;
const unsigned int SG = A4;
const unsigned int LC = A5;

// Torque ramp 
const int LOWPOS = 200;
const int HIGHPOS = 2000;
const int HIGHFORCE = 363;
const int LOWFORCE = 300;

// Averaging variables
long lcsum;
long sgsum;
long cursum;
int curno;
int lcno;
int sgno;

// Signal variables
long encread;
long csread;
long sgread;

// Encoder object
Encoder enc(6, 7, 12);

int yolo = 3600;
// PID parameters
float kp = 0;
float ki = 0;
float kd = 0;
long input, output, target;

// PID object - note that the interval is in microseconds
PID_lite myPID(&input,&output,&target, kp, ki, kd, 1000);

const signed int CSoffset = 0;

char theChar;
bool goflag = false;
unsigned int modulation = 0;

signed int effort;
signed int error;
bool rampingUp;

void setup() {
  // Initialize motor paramters
  effort = 0;
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(PW, OUTPUT);

  // Set PWM properties- 32kHz, 
  TCCR1B = TCCR1B & 0b11111000 | 0x01;
  TCCR2B = TCCR2B & 0b11111000 | 0x01;
  Serial.begin(9600);
  myPID.setSaturation(-50,50);
  rampingUp = true;
}

void loop() {
  // Step one: check out the serial port
  checkSerial();
  // Step two: check sensors
  encread = enc.read();
//  csread = (analogRead(A3) - 512 - CSoffset);
  sgread = analogRead(SG);
  input = sgread;

  if(goflag)
  {
    myPID.activate();
    controllIt();
  }
  else
  {
    setEffort(0);
    myPID.deactivate();
  }
  
  
  // simplest control
  int strainoffset = 318; // Lower numbers will bias motor clockwise
  target = strainoffset;
  
  
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

void checkSerial()
{
  if(Serial.available())
  {
    theChar = Serial.read();
    switch (theChar){
    case COMGO:
      goflag = true;
      cursum = 0;
      lcsum = 0;
      sgsum = 0;
      curno = 0;
      lcno = 0;
      sgno = 0;
      Serial.println("GO!");

      break;
    case COMSTOP:
      goflag = false;
      modulation = 0; // NOTE: THIS HAS BEEN ADDED
      Serial.println("STOP");
      break;
    case COMRAMP:
      modulation += 5;
      Serial.println(modulation);
      break;
    case SETKP:
      kp =  parseInput();
      myPID.setParameters(kp,ki,kd);
      Serial.println(kp,4);
      break;
    case SETKI:
      ki =  parseInput();
      myPID.setParameters(kp,ki,kd);
      Serial.println(ki,4);
      break;
    case SETKD:
      kd =  parseInput();
      myPID.setParameters(kp,ki,kd);
      Serial.println(kd,4);
      break;
    case SETTARGET:
      target = parseInput();
      Serial.println(target);
      break;
    case SESAT:
    int saturday = parseInput();
    Serial.println(saturday);
     myPID.setSaturation(-saturday,saturday);
     break;
    }
  }

}

void controllIt()
{
  //encread < 200||
 if (encread >= LOWPOS && encread <= HIGHPOS)
  {
    //target = map(encread, HIGHPOS,LOWPOS,LOWFORCE,HIGHFORCE);
   //Serial.println(input);
  }
  if (encread > HIGHPOS|| encread < LOWPOS) {
    setEffort(0);
    myPID.deactivate();
  }
  else
  {
    if(myPID.compute(micros()))
    {
      setEffort(output);
    }
  }
}

float parseInput()
{
  delay(50);
  char theChar = '0';
  float theFloat = 0;
  int thePlace = 1;
  bool decPoint = false;
  while (Serial.available())
  {
    theChar = Serial.read();
    if(theChar == '.')
    {
     decPoint = true; 
     
    }
    else if(decPoint)
    {
      theFloat = theFloat + ((float)theChar-48)/(pow(10, thePlace));
      //Serial.println(input);
      thePlace++;
    }
    else
    {
    theFloat = 10*theFloat + (theChar-48);
    }
    delay(50);
  }
  //  Serial.println(theFloat);    //le debug
  return theFloat;
}






