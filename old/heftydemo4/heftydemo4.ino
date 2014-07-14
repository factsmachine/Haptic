#include <osmc.h>
#include <HerkBrake.h>
#include <encoder.h>

// OSMC portions based on Chris B Stones' code.
const int AHI     = 8;     // normal I/O pin
const int BHI     = 7;     // normal I/O pin
const int ALI     = 6;     // must be on a PWM I/O pin
const int BLI     = 9;     // must be on a PWM I/O pin   // CHANGED FROM PIN 5 FOR PWM01 LIBRARY
const int DIS = 4; // normal I/O pin
const int CSn = 51; // Chip select pin
const int CLK = 52; // Clock signal pin
const int DO = 53;  // Digital output pin
const int BUTTON_PIN = 22;
const int EFFORT_ADJUSTMENT = 1000;
const long EFFORT_LIMIT_LOW_DEFAULT = -10000;
const long EFFORT_LIMIT_HIGH_DEFAULT = 10000;

// Dead-zone limits
const int LOWPOS = 824;
const int HIGHPOS = 1962;

const char COM_GO = 'a';
const char COM_STOP = 'b';
const char COM_CW_MORE = 'p';
const char COM_CCW_MORE = 'q';
const char COM_CW_SET = 'o';
const char COM_CCW_SET = 'w';
const char COM_CW_LIMIT_SET = 'm';
const char COM_CCW_LIMIT_SET = 'z';

int STATE = 0;

const int STATE_SG_NEUTRAL = 1;
const int STATE_SG_DEADZONE = 2;
const int STATE_NORMAL_CONTROL = 3;
const int STATE_PAST_LIMITS_INIT = 4;
const int STATE_PAST_LIMITS_WAIT = 5;
const int STATE_EMERGENCY = 99;

const unsigned int CALIB_MID = 1848;
const unsigned int CALIB_TOL = 50;
const unsigned int CALIB_WAIT = 10000;

// Button variables
bool pressedInNow = false;
bool pressedInLast = false;
bool go_flag = true;
long effort = 0;
char theChar;
long effort_limit_low = EFFORT_LIMIT_LOW_DEFAULT;
long effort_limit_high = EFFORT_LIMIT_HIGH_DEFAULT;
unsigned int encread;
unsigned long calibcount;
bool calibflag = false;

OSMC* osmc;
HerkBrake* herk_brake;
Encoder* enc;

void setup() {  
  Serial.begin(115200);
  Serial.println("Beginning!"); 
  //herk_brake = new HerkBrake(0xfd, 1, -72.0f);
  //herk_brake = new HerkBrake(0xfd, 1, -36.0f);
  analogReadResolution(12); // For best strain gauge functionality
  herk_brake = new HerkBrake(0xfd, 1, 118.3f, 41.6f);
  osmc = new OSMC(AHI, BHI, ALI, BLI, DIS, effort_limit_low, effort_limit_high);
  enc = new Encoder(CSn, CLK, DO);
  pinMode(22, INPUT); // For push buttton
  STATE = 1;
  calibcount = 0;
}

void loop() {
  switch (STATE) {
    case STATE_SG_NEUTRAL:
      encread = enc.read();
      if (encread >= CALIB_MID-CALIB_TOL && encread <= CALIB_MID+CALIB_TOL) {
        calibcount++;
      }
      else {
        calibcount = 0;
      }
      
      if (calibcount >= CALIB_WAIT) {
        sgoffset = analogRead(SG);
        calibcount = 0; // So that a recalibration won't happen instantaneously
        Serial.println("Gauge calibrated. Awaiting dead-zone entry...");
        Serial.println(sgoffset);
        calibflag = false;
        STATE = STATE_SG_DEADZONE;
      }
    break;
    
    case STATE_SG_DEADZONE:
      encread = enc.read();
      if (isInDeadZone(encread)) {
        Serial.println("Dead-zone entered.");
        Serial.println("Active!");
        go_flag = true;
        lastencread = encread;
        STATE = STATE_NORMAL_CONTROL;
      }
    break;

    case STATE_NORMAL_CONTROL: // General GO state
      // Update relevant data
      encread = enc->read();
      checkSerial();
      osmc->setEffort(effort);
      
      // Check for transition conditions
      if (digitalRead(BUTTON_PIN) == LOW) {
        osmc->brake();
        herk_brake->brake();
        go_flag = false;
        osmc->setEffort(0);
        STATE = STATE_EMERGENCY;
      }
      else if (isInDeadZone(encread) || !go_flag) {
        osmc->brake();
        herk_brake->brake();
        //go_flag = false;
        osmc->setEffort(0);
        STATE = STATE_PAST_LIMITS_INIT;
      }
    break;
    
    case STATE_PAST_LIMITS_INIT: // Quick state
      delay(500);
      herk_brake->unbrake();
      STATE = STATE_PAST_LIMITS_WAIT;
    break;
    
    case 3: // Temporary brake state
      encread = enc->read();
      checkSerial();
      if (go_flag && !isInDeadZone(encread)) {
        STATE = STATE_NORMAL_CONTROL;
      }
    break;
    
    default:
    case STATE_EMERGENCY: // EMERGENCY STOP STATE
      osmc->brake();
      herk_brake->brake();
      go_flag = false;
      while(true) {} // Enter into infinite loop (waits for board reset)
    break;
  }
}

////////////////////////////////////////////////////////////////////////
//SUPPPORTING METHODS
void checkSerial()
{
  if(Serial.available())
  {
    theChar = Serial.read();
    switch (theChar){
    case COM_GO:
      go_flag = true;
      Serial.println("GO!");
      break;
    case COM_STOP:
      go_flag = false;
      Serial.println("STOP");
      break;
    case COM_CW_MORE:
      effort += EFFORT_ADJUSTMENT;
      Serial.println("New commanded effort:");
      Serial.println(effort);
      break;
    case COM_CCW_MORE:
      effort -= EFFORT_ADJUSTMENT;
      Serial.println("New commanded effort:");
      Serial.println(effort);
      break;
    case COM_CW_SET:
      effort = (long)(parseInput());
      Serial.println("New commanded effort:");
      Serial.println(effort);
      break;
    case COM_CCW_SET:
      effort = -(long)(parseInput());
      Serial.println("New commanded effort:");
      Serial.println(effort);
      break;
    case COM_CW_LIMIT_SET:
      effort_limit_high = (long)(parseInput());
      osmc->setEffortLimitHigh(effort_limit_high);
      Serial.println("New high effort limit:");
      Serial.println(osmc->getEffortLimitHigh());
      break;
    case COM_CCW_LIMIT_SET:
      effort_limit_low = -(long)(parseInput());
      osmc->setEffortLimitLow(effort_limit_low);
      Serial.println("New low effort limit:");
      Serial.println(osmc->getEffortLimitLow());
      break;
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
      thePlace++;
    }
    else
    {
      theFloat = 10*theFloat + (theChar-48);
    }
    delay(50);
  }
  return theFloat;
}

// Auxiallary functions
bool isInDeadZone(unsigned int encodervalue)
{
  return (encodervalue >= HIGHPOS || encodervalue <= LOWPOS);
}
