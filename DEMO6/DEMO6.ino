#include "encoder.h"
#include "MotorDriver.h"
#include "PID_lite.h"
#include <math.h> 
#define PI 3.14159265

// Good KP: 0.4
// Good KI: 0.1
// Good target: 330

// Command defines
const char COMGO = 'a';
const char COMSTOP = 'b';
const char SETKP = 'p';
const char SETKI = 'i';
const char SETKD = 'd';
const char SETTARGET = 't';
const char SESAT = 's';
const char CALIB = 'r';
const char BRAKE = 'k';
const char UNBRAKE = 'u';

// Pin assignments
const unsigned int INA = 9; // For motor driver
const unsigned int INB = 8; // For motor driver
const unsigned int PW = 10; // PWM for motor driver
const unsigned int CS = A3;
const unsigned int SG = A4;
const unsigned int LC = A5;

// Dead-zone limits
const int LOWPOS = 275;
const int HIGHPOS = 2100;

// Sensor variables
long encread;
long csread;
long sgread;

// State machine variable
unsigned int STATE = 0;

// Calibration variables
unsigned int calibcount = 0;
signed int sgoffset;
bool calibflag = false; // whether a switch from control state to calibration state called
const unsigned int CALIB_WAIT = 5000;
const unsigned int CALIB_MID = 1200;
const unsigned int CALIB_TOL = 75;
const signed int CALIB_WEIRD_OFFSET = 2; // Seems to be about 4 or less.

// Error checking variables
long lastencread;
long lasttimestamp = 0;
const long VELOCITY_TIME_GAP = 10000;  // In microseconds
const unsigned int MAX_SPEED = 85;  // Encoder counts per VELOCITY_TIME_GAP to trigger speed error

// Other constants
const float GRAVITY_STRAIN = -5.0f; // Measured at -8.0, but values closer to 0 are more stable

// PID parameters
float kp = 0.4;
float ki = 0.1;
float kd = 0;
long input, output, target;

// Objects 
Encoder enc(6, 7, 12);
MotorDriver mot(INA, INB, PW);
PID_lite myPID(&input,&output,&target, kp, ki, kd, 1000); // note that the interval is in microseconds

// Other global variables
char theChar;
bool goflag = false;


//================================================================================================


void setup() {
  // Set PWM properties- 32kHz, 
  TCCR1B = TCCR1B & 0b11111000 | 0x01;
  TCCR2B = TCCR2B & 0b11111000 | 0x01;
  Serial.begin(9600);
  myPID.setSaturation(-50,50);
  calibcount = 0;
  Serial.println("Booted. Awaiting gauge calibration...");
  STATE = 1;
}

void loop() {
  // NOTE: INCLUDE DEFAULT STATE FOR SAFETY
  switch (STATE) {
    
    // STATE 1: Strain gauge calibration.
    case 1:
      encread = enc.read();
      //sgoffset = analogRead(SG); // Would only do this here to counter weird 3-4 count offset
      if (encread >= CALIB_MID-CALIB_TOL && encread <= CALIB_MID+CALIB_TOL) {
        calibcount++;
      }
      else {
        calibcount = 0;
      }
      
      if (calibcount >= CALIB_WAIT) {
        sgoffset = analogRead(SG) - CALIB_WEIRD_OFFSET;
        calibcount = 0; // So that a recalibration won't happen instantaneously
        Serial.println("Gauge calibrated. Awaiting dead-zone entry...");
        Serial.println(sgoffset);
        //target = sgoffset;
        calibflag = false;
        STATE = 2;
      }
    break;
    
    // STATE 2: Waiting for position to enter dead-zone so control can start
    case 2:
      encread = enc.read();
      if (isInDeadZone(encread)) {
        Serial.println("Dead-zone entered.");
        Serial.println("Active!");
        goflag = true;
        lastencread = encread;
        STATE = 3;
      }
    break;
    
    // STATE 3: Actual control algorithm
    case 3:
      // Check for and process new serial data
      checkSerial();
      // Update sensor readings
      encread = enc.read();
      sgread = analogRead(SG);
      input = sgread;
    
      // Check for overspeed. Note that this entire routine only runs once every VELOCITY_TIME_GAP.
      // Performance can be improved by constantly checking "isGoingTooFast()" if desired.
      if (micros() - lasttimestamp > VELOCITY_TIME_GAP) {
        lasttimestamp = micros();
        if (isGoingTooFast(encread, lastencread)) {
          goflag = false;
        }
        lastencread = encread;
      }
    
      // Compensate for gravity
      target = sgoffset + (long)(GRAVITY_STRAIN * sin(PI * (encread - CALIB_MID) / 2048.0f));
    
      // Change additional control variables here if necessary
    
      // Perform actual control (if allowed by goflag)
      if (goflag) {
        myPID.activate();
        controllIt();
      }
      else {
        mot.setEffort(0);
        myPID.deactivate();
        STATE = 4; // Note: deactivation above is technically redundant, but safer
        break;
      }
      
      // Check for calibration request
      if (calibflag) {
        mot.setEffort(0);
        myPID.deactivate();
        STATE = 1;
        break;
      }
    break;
    
    // STATE 4: Deactivate motors and controller and print stop message
    default:
    case 4:
      mot.setEffort(0);
      myPID.deactivate();
      Serial.println("Halted.");
      STATE = 5;
    break;
    
    // STATE 5: Check serial until we reactivate
    case 5:
      checkSerial();
      if (goflag) {
        Serial.println("Active!");
        lastencread = enc.read(); // So speed kill isn't triggered
        STATE = 3;
      }
    break;
  }
}

void checkSerial()
{
  if(Serial.available())
  {
    theChar = Serial.read();
    switch (theChar){
    case COMGO:
      goflag = true;
      //Serial.println("GO!");
      break;
    case COMSTOP:
      goflag = false;
      //Serial.println("STOP");
      break;
    case CALIB:
      goflag = false;
      calibflag = true;
      //Serial.println("STOP");
      Serial.println("Recalibration requested.");
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
    case BRAKE:
      mot.brake();
      Serial.println("Braked.");
      break;
    case UNBRAKE:
      mot.unbrake();
      Serial.println("Unbraked.");
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
  if (isInDeadZone(encread)) {
    mot.setEffort(0);
    myPID.deactivate();
  }
  else
  {
    if(myPID.compute(micros())) {
      mot.setEffort(output);
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

bool isInDeadZone(unsigned int encodervalue)
{
  return (encodervalue >= HIGHPOS|| encodervalue <= LOWPOS);
}

bool isGoingTooFast(int cur, int last)
{
  if (cur > last) {
    return (cur - last > MAX_SPEED);
  }
  else {
    return (last - cur > MAX_SPEED);
  }
}
