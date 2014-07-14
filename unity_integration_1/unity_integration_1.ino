#include <encoder.h>
#include <osmc.h>
#include <PID_v1.h>

// Dead-zone limits
const int LOWPOS = 1110;
const int HIGHPOS = 3160;
const int MAXVAL = 65535;

const int MAX_EFFORT = 35000;

// Encoder
const int CSn = 51; // Encoder chip select pin
const int CLK = 52; // Encoder clock signal pin
const int DO = 53;  // Encoder digital output pin
// OSMC
const int AHI = 8;     // (digital)
const int BHI = 7;     // (digital)
const int ALI = 6;     // (PWM)
const int BLI = 9;     // (PWM), originally pin 5 in pwm01 library.
const int DIS = 4;     // Diable pin (digital)

const char UNITY_WANTS_POSITION = 'p';
const char UNITY_HAS_POSITION = 'r';

const char KP_UP = '7';
const char KP_DOWN = '1';
const char KI_UP = '8';
const char KI_DOWN = '2';
const char KD_UP = '9';
const char KD_DOWN = '3';

//const char CONTROL_ON = 'a';
const char CONTROL_LEFT = 'a';
const char CONTROL_RIGHT = 'z';
const char CONTROL_OFF = 'b';

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
OSMC* osmc;
PID* pos_ctrl;

double kp = 160.0f;
double ki = 0.0f;
double kd = 1.6f;

double input, output, target;

bool controlOn = true;

void setup() {
  Serial.begin(115200);
  enc = new Encoder(CSn, CLK, DO);
  osmc = new OSMC(AHI, BHI, ALI, BLI, DIS, -MAX_EFFORT, MAX_EFFORT);
  pos_ctrl = new PID(&input, &output, &target, kp, ki, kd, 0);
  pos_ctrl->SetOutputLimits(-MAX_EFFORT, MAX_EFFORT);
  pos_ctrl->SetSampleTime(1); // milliseconds
  pos_ctrl->SetMode(1); // required to set "auto" mode for some reason
  encread = 0;
  translated = 0;
  u_position = enc->read(); // First target is where we already are
  left_to_read = 0;
  done = false;
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  STATE = 0;
}

void loop() {  
  encread = enc->read();
  
  // Maintain encoder reading within bounds
  if (encread > HIGHPOS) {
    encread = HIGHPOS;
    osmc->brake();
  } else if (encread < LOWPOS) {
    encread = LOWPOS;
    osmc->brake();
  } else if (!controlOn) {
    osmc->brake();
  } else {
    // CONTROL ALGORITHM
    input = encread;
    target = u_position;
    if (pos_ctrl->Compute())
    {
      osmc->setEffort(output);
    }
  }
  
  // If there's a character to process
  if (Serial.available())
  {
    serialChar = Serial.read();
    
    // PROCESS THAT CHARACTER!!
    if (STATE == 1) // Interpreting as new position announcement
    {
      left_to_read--;
      //temp_u_position += (serialChar << (8 * left_to_read));
      temp_u_position += (serialChar << (8 * (1 - left_to_read))); // TEMP WORKAROUND
      
      if (left_to_read <= 0)
      {
        u_position = unityToEncoder(temp_u_position);
        
        /*if (temp_u_position > 10000)
        {
          digitalWrite(LED_PIN, HIGH);
        } else {
          digitalWrite(LED_PIN, LOW);
        }*/
        
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
        case CONTROL_OFF:
          controlOn = false;
          pos_ctrl->clearIntegration();
          break;
        case CONTROL_LEFT:
          controlOn = true;
          u_position = enc->read();
          pos_ctrl->SetOutputLimits(0, MAX_EFFORT);
          break;
        case CONTROL_RIGHT:
          controlOn = true;
          u_position = enc->read();
          pos_ctrl->SetOutputLimits(-MAX_EFFORT, 0);
        /*case KP_UP:
          kp += 5;
          pos_ctrl->SetTunings(kp, ki, kd);
          //Serial.println(kp);
          break;
        case KP_DOWN:
          kp -= 5;
          pos_ctrl->SetTunings(kp, ki, kd);
          //Serial.println(kp);
          break;
        case KI_UP:
          ki += 5;
          pos_ctrl->SetTunings(kp, ki, kd);
          //Serial.println(ki);
          break;
        case KI_DOWN:
          ki -= 5;
          pos_ctrl->SetTunings(kp, ki, kd);
          //Serial.println(ki);
          break;
        case KD_UP:
          kd += 0.01;
          pos_ctrl->SetTunings(kp, ki, kd);
          //Serial.println(kd);
          break;
        case KD_DOWN:
          kd -= 0.01;
          pos_ctrl->SetTunings(kp, ki, kd);
          //Serial.println(kd);
          break;*/
      }
    }
  }
  
  //delay(10);
}

unsigned short int encoderToUnity(int enc_reading)
{
  return (unsigned short int)(MAXVAL * (enc_reading - LOWPOS) / (HIGHPOS - LOWPOS));
}

int unityToEncoder(unsigned short int unity_reading)
{
  return LOWPOS + ((HIGHPOS - LOWPOS) * unity_reading / MAXVAL);
}


