#include <osmc.h>
#include <HerkBrake.h>
#include <encoder.h>
#include <PID_lite.h>

//===================================PIN ASSIGNMENTS
// OSMC
const int AHI = 8;     // (digital)
const int BHI = 7;     // (digital)
const int ALI = 6;     // (PWM)
const int BLI = 9;     // (PWM), originally pin 5 in pwm01 library.
const int DIS = 4;     // Diable pin (digital)
// Encoder
const int CSn = 51; // Encoder chip select pin
const int CLK = 52; // Encoder clock signal pin
const int DO = 53;  // Encoder digital output pin
// Emergency stop button
const int BUTTON_PIN = 22;
// Strain gauge
const int SG = A0;
// Current sensor
const int CURRENT_SENSE_PIN = A1;


const int SG_OFFSET_AT_CALIBRATION = 124; // SG at calib minus streaming SG reading
const int EFFORT_ADJUSTMENT = 1000;
const long EFFORT_LIMIT_LOW_DEFAULT = -20000;
const long EFFORT_LIMIT_HIGH_DEFAULT = 20000;

// Dead-zone limits
const int LOWPOS = 824;
const int HIGHPOS = 1962;

const char COM_GO = 'a';
const char COM_STOP = 'b';
const char COM_CW_MORE = ']';
const char COM_CCW_MORE = '[';
const char COM_CW_SET = '}';
const char COM_CCW_SET = '{';
const char COM_CW_LIMIT_SET = 'm';
const char COM_CCW_LIMIT_SET = 'z';
const char COM_SET_KP = 'p';
const char COM_SET_KI = 'i';
const char COM_SET_KD = 'd';

int STATE = 0;

const int STATE_SG_NEUTRAL = 1;
const int STATE_SG_DEADZONE = 2;
const int STATE_NORMAL_CONTROL = 3;
const int STATE_PAST_LIMITS_INIT = 4;
const int STATE_PAST_LIMITS_WAIT = 5;
const int STATE_HALTED = 6;
const int STATE_WAIT_FOR_SERIAL = 7;
const int STATE_EMERGENCY = 99;

const unsigned int CALIB_MID = 1848;
const unsigned int CALIB_TOL = 50;
const unsigned int CALIB_WAIT = 10000;

//const float GRAVITY_STRAIN = -70.0f; // Measured at -70.0, but values closer to 0 are more stable
const float GRAVITY_STRAIN = 0.0f;

// PID parameters
float kp = 0;
float ki = 0;
float kd = 0;
long input, output, target;

// Button variables
bool pressedInNow = false;
bool pressedInLast = false;
bool go_flag = true;
long effort = 0;
char theChar;
long effort_limit_low = EFFORT_LIMIT_LOW_DEFAULT;
long effort_limit_high = EFFORT_LIMIT_HIGH_DEFAULT;
long encread;
long csread;
long lastencread;
unsigned long calibcount;
bool calibflag = false;
int sgoffset;
long sgread;

OSMC* osmc;
HerkBrake* herk_brake;
Encoder* enc;
//PID_lite myPID(&input,&output,&target, kp, ki, kd, 1000); // note that the interval is in microseconds
PID_lite* myPID;

long start_time;
long elapsed;

void setup() {  
  Serial.begin(19200);
  Serial.println("Beginning. Awaiting gauge calibration...");
  //herk_brake = new HerkBrake(0xfd, 1, -72.0f);
  //herk_brake = new HerkBrake(0xfd, 1, -36.0f);
  analogReadResolution(12); // For best strain gauge functionality
  //herk_brake = new HerkBrake(0xfd, 1, 118.3f, 41.6f);
  herk_brake = new HerkBrake(0xfd, 1, 0.0f); // Until brake is fixed
  osmc = new OSMC(AHI, BHI, ALI, BLI, DIS, effort_limit_low, effort_limit_high);
  enc = new Encoder(CSn, CLK, DO);
  myPID = new PID_lite(&input, &output, &target, kp, ki, kd, 1000);
  pinMode(BUTTON_PIN, INPUT); // For push buttton
  STATE = 1;
  calibcount = 0;
  myPID->setSaturation(EFFORT_LIMIT_LOW_DEFAULT, EFFORT_LIMIT_HIGH_DEFAULT);
}

void loop() {
  switch (STATE) {
    case STATE_SG_NEUTRAL:
      encread = enc->read();
      if (encread >= CALIB_MID-CALIB_TOL && encread <= CALIB_MID+CALIB_TOL) {
        calibcount++;
      }
      else {
        calibcount = 0;
      }
      
      // State transitions
      if (emergency()) {emergencyProcedure();}
      else if (calibcount >= CALIB_WAIT) {
        int temp_offset = analogRead(SG);
        sgoffset = temp_offset - SG_OFFSET_AT_CALIBRATION;
        calibcount = 0; // So that a recalibration won't happen instantaneously
        Serial.println("Gauge calibrated. Awaiting dead-zone entry...");
        Serial.print(temp_offset);
        Serial.print(" --> ");
        Serial.println(sgoffset);
        calibflag = false;
        STATE = STATE_SG_DEADZONE;
      }
    break;
    
    case STATE_SG_DEADZONE:
      encread = enc->read();
      
      // State transitions
      if (emergency()) {emergencyProcedure();}
      else if (isInDeadZone(encread)) {
        Serial.println("Dead-zone entered.");
        Serial.println("Active!");
        go_flag = true;
        lastencread = encread;
        STATE = STATE_NORMAL_CONTROL;
      }
    break;

    case STATE_NORMAL_CONTROL: // General GO state
      start_time = micros();
      //delayMicroseconds(869);
    
      // Update relevant data
      encread = enc->read();
      csread = analogRead(CURRENT_SENSE_PIN);
      sgread = analogRead(SG);
      checkSerial();
      //osmc->setEffort(effort);
      
      // Check for overspeed. Note that this entire routine only runs once every VELOCITY_TIME_GAP.
      // Performance can be improved by constantly checking "isGoingTooFast()" if desired.
      //if (micros() - lasttimestamp > VELOCITY_TIME_GAP) {
      //  lasttimestamp = micros();
      //  if (isGoingTooFast(encread, lastencread)) {
      //    goflag = false;
      //  }
      //  lastencread = encread;
      //}
      
      input = sgread;
      target = sgoffset; // No gravity compensation for now...
      //target = sgoffset + (long)(GRAVITY_STRAIN * sin(PI * (encread - CALIB_MID) / 2048.0f));
      //Serial.println(" ");
      //Serial.println(input);
      //Serial.println(target);
      
      if (go_flag) {
        myPID->activate();
        controllIt();
      }
      else {
        osmc->setEffort(0);
        myPID->deactivate();
        STATE = STATE_HALTED; // Note: deactivation above is technically redundant, but safer
        break;
      }
      
      // Check for transition conditions
      if (emergency()) {emergencyProcedure();}
      else if (isInDeadZone(encread) || !go_flag) {
        osmc->brake();
        herk_brake->brake();
        //go_flag = false;
        osmc->setEffort(0);
        STATE = STATE_PAST_LIMITS_INIT;
      }
      else {
        // For debug
        elapsed = micros() - start_time;
        //Serial.println(elapsed);
      }
    break;
    
    case STATE_PAST_LIMITS_INIT: // Quick state
      delay(500);
      
      // State transitions
      if (emergency()) {emergencyProcedure();}
      else {
        herk_brake->unbrake();
        STATE = STATE_PAST_LIMITS_WAIT;
      }
    break;
    
    case STATE_PAST_LIMITS_WAIT: // Temporary brake state
      encread = enc->read();
      checkSerial();
      
      // State transitions
      if (emergency()) {emergencyProcedure();}
      else if (go_flag && !isInDeadZone(encread)) {
        STATE = STATE_NORMAL_CONTROL;
      }
    break;
    
    case STATE_HALTED:
      osmc->setEffort(0);
      myPID->deactivate();
      Serial.println("Halted.");
      
      // State transitions
      if (emergency()) {emergencyProcedure();}
      else {
        STATE = STATE_WAIT_FOR_SERIAL;
      }
    break;
    
    case STATE_WAIT_FOR_SERIAL:
      checkSerial();
      if (emergency()) {emergencyProcedure();}
      else if (go_flag) {
        Serial.println("Reactivated!");
        //lastencread = enc.read(); // So speed kill isn't triggered
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
    // The effort commands might not be relevant anymore...
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
    case COM_SET_KP:
      kp = parseInput();
      myPID->setParameters(kp, ki, kd);
      Serial.println(kp, 4);
      break;
    case COM_SET_KI:
      ki =  parseInput();
      myPID->setParameters(kp, ki, kd);
      Serial.println(ki, 4);
      break;
    case COM_SET_KD:
      kd =  parseInput();
      myPID->setParameters(kp, ki, kd);
      Serial.println(kd, 4);
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

//bool isGoingTooFast(int cur, int last)
//{
//  if (cur > last) {
//    return (cur - last > MAX_SPEED);
//  }
//  else {
//    return (last - cur > MAX_SPEED);
//  }
//}

void controllIt()
{
  if (isInDeadZone(encread)) {
    osmc->setEffort(0);
    myPID->deactivate();
  }
  else
  {
    osmc->setEffort((long)(kp * (input - target))); // Put this here so we don't wait for Marc's code to work
    
    if (false) {
      Serial.print("SG");
      Serial.print(input);
      Serial.print(" TGT");
      Serial.print(target);
      Serial.print(" ERR");
      Serial.print(input - target);
      Serial.print(" EFF");
      Serial.print(osmc->getEffort());
      Serial.println();
    }
    
    
    //if(myPID->compute(micros())) {
      //osmc->setEffort( (input - target) * 5);
      //osmc->setEffort(output);
      // DEBUGGG
      //osmc->setEffort(-output);
      //DEBUG
      
      /*
      Serial.print("SG");
      Serial.print(input);
      Serial.print(" TGT");
      Serial.print(target);
      Serial.print(" ERR");
      Serial.print(input - target);
      Serial.print(" EFF");
      Serial.print(osmc->getEffort());
      Serial.println();
      */
    //}
  }
}

bool emergency()
{
  return (digitalRead(BUTTON_PIN) == LOW);
}

void emergencyProcedure()
{
  osmc->brake();
  herk_brake->brake();
  go_flag = false;
  osmc->setEffort(0);
  STATE = STATE_EMERGENCY;
}
