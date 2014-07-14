#include <osmc.h>
#include <HerkBrake.h>

// OSMC portions based on Chris B Stones' code.
const int AHI     = 8;     // normal I/O pin
const int BHI     = 7;     // normal I/O pin
const int ALI     = 6;     // must be on a PWM I/O pin
const int BLI     = 9;     // must be on a PWM I/O pin   // CHANGED FROM PIN 5 FOR PWM01 LIBRARY
const int DIS = 4; // normal I/O pin
const int BUTTON_PIN = 22;
const int EFFORT_ADJUSTMENT = 1000;
const long EFFORT_LIMIT_LOW_DEFAULT = -10000;
const long EFFORT_LIMIT_HIGH_DEFAULT = 10000;

const char COM_GO = 'a';
const char COM_STOP = 'b';
const char COM_CW_MORE = 'p';
const char COM_CCW_MORE = 'q';
const char COM_CW_SET = 'o';
const char COM_CCW_SET = 'w';
const char COM_CW_LIMIT_SET = 'm';
const char COM_CCW_LIMIT_SET = 'z';

int STATE = 0;

// Button variables
bool pressedInNow = false;
bool pressedInLast = false;
bool go_flag = true;
long effort = 0;
char theChar;
long effort_limit_low = EFFORT_LIMIT_LOW_DEFAULT;
long effort_limit_high = EFFORT_LIMIT_HIGH_DEFAULT;

OSMC* osmc;
HerkBrake* herk_brake;

void setup() {  
  Serial.begin(115200);
  Serial.println("Beginning!"); 
  herk_brake = new HerkBrake(0xfd, 1, -72.0f);
  osmc = new OSMC(AHI, BHI, ALI, BLI, DIS, effort_limit_low, effort_limit_high);
  pinMode(22, INPUT); // For push buttton
  STATE = 1;
}

void loop() {
  switch (STATE) {
    case 1:
      herk_brake->unbrake();
      osmc->setEffort(effort);
    break;
    
    default:
    case 2:
      osmc->brake();
      herk_brake->brake();
      go_flag = false;
    break;
  }
  
  if (digitalRead(BUTTON_PIN) == LOW) {
    osmc->brake();  // These are technically redundant rel. to state 2
    herk_brake->brake();
    go_flag = false;
    STATE = 2;
  }
  
  checkSerial();
  
  if (go_flag) {
    STATE = 1;
  } else {
    STATE = 2;
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
