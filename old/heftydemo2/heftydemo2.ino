#include <osmc.h>
#include <HerkBrake.h>
#include <pwm01.h>
//#define TC1_CMR ((volatile unsigned int *) 0x40080004);
//#define TC1_CMR ((volatile unsigned int *) 0x40080004);

//const int TC1_CMR = 0x40080004; // this is the address of a register...
//#define TC1_CMR ((volatile unsigned int *) 0xFFFE0044);

// Based on Chris B Stones' code.
/* When wiring up an OSMC Board this values correspond to the HIP4081A chip specs. 
Refer to that chips data sheet to figure out which pins to wire to. 
Pin 2 on the HIP Chip is BHI
Pin 3 on the HIP Chip is DIS
Pin 5 on the HIP Chip is BLI
Pin 6 on the HIP Chip is ALI
Pin 7 on the HIP Chip is AHI
etc..
*/ 

/* With this Very Basic library you need 5 pins per
   OSMC board. They all are I/O pins but 2 of them have to 
   be PWM capable 
*/
const int AHI     = 8;     // normal I/O pin
const int BHI     = 7;     // normal I/O pin
const int ALI     = 6;     // must be on a PWM I/O pin
const int BLI     = 5;     // must be on a PWM I/O pin
const int DIS = 4; // normal I/O pin
const int BUTTON_PIN = 22;
const int EFFORT_ADJUSTMENT = 50;
const char COM_GO = 'a';
const char COM_STOP = 'b';
const char COM_CW_MORE = 'p';
const char COM_CCW_MORE = 'q';
const char COM_CW_SET = 'o';
const char COM_CCW_SET = 'w';

int STATE = 0;

// Button variables
bool pressedInNow = false;
bool pressedInLast = false;
bool go_flag = true;
int effort = 0;
char theChar;

OSMC* osmc;
HerkBrake* herk_brake;

void setup() {
  //REG_PWM_CLK = REG_PWM_CLK & 0b11111000 | 0x01; // Ought to set PWM frequency
  //PWM->PWM_CH_NUM[0].PWM_CMR = 0x1904; // Divides 21 MHz... 2904 should mean 2 KHz...
  //PWM->PWM_CH_NUM[1].PWM_CMR = 0x1904; // Divides 21 MHz... 2904 should mean 2 KHz...
  //PWM->PWM_CH_NUM[2].PWM_CMR = 0x1904; // Divides 21 MHz... 2904 should mean 2 KHz...
  //PWM->PWM_CH_NUM[3].PWM_CMR = 0x1904; // Divides 21 MHz... 2904 should mean 2 KHz...
  //PWM->PWM_CH_NUM[4].PWM_CMR = 0x1904; // Divides 21 MHz... 2904 should mean 2 KHz...
  //REG_PWM_CLK = 0;                // choose clock reference, 0 -> full MCLK as reference 84MHz, determines the possible range of PWM freq
  //REG_PWM_CPRD0 = 8400;            // initialize gray code PWM slope period -> T = value/84MHz (2<value<16bit), value=840 -> 100kHz slope rate
  //REG_PWM_CDTY0 = 1;              // initialize duty cycle (not sure why this would be required, but w/e)
  //pmc_enable_periph_clk(PWM_INTERFACE_ID);
  //PWM->PWM_CLK = 0x.......;
  //PWMC_ConfigureClocks(16000 * 255, 16000 * 255, VARIANT_MCK);
  //PWMC_ConfigureClocks(16000 * 255 , 0, VARIANT_MCK);
  
  // CODE COPIED
  uint32_t  pwm_duty = 32767;
    uint32_t  pwm_freq1 = 5000;  
    //uint32_t  pwm_freq2 = 5000;

    // Set PWM Resolution
    pwm_set_resolution(16);  

    // Setup PWM Once (Up to two unique frequencies allowed
    //-----------------------------------------------------    
    pwm_setup( 6, pwm_freq1, 1);  // Pin 6 freq set to "pwm_freq1" on clock A
    //pwm_setup( 7, pwm_freq2, 2);  // Pin 7 freq set to "pwm_freq2" on clock B
    //pwm_setup( 8, pwm_freq2, 2);  // Pin 8 freq set to "pwm_freq2" on clock B
    //pwm_setup( 9, pwm_freq2, 2);  // Pin 9 freq set to "pwm_freq2" on clock B
      
    // Write PWM Duty Cycle Anytime After PWM Setup
    //-----------------------------------------------------    
    pwm_write_duty( 6, pwm_duty );  // 50% duty cycle on Pin 6
    //pwm_write_duty( 7, pwm_duty );  // 50% duty cycle on Pin 7
    //pwm_write_duty( 8, pwm_duty );  // 50% duty cycle on Pin 8
    //pwm_write_duty( 9, pwm_duty );  // 50% duty cycle on Pin 9

    delay(1000);
    pwm_stop(6);

    //delay(30000);  // 30sec Delay; PWM signal will still stream
        
    // Force PWM Stop On All Pins
    //-----------------------------    
    //pwm_stop( 6 );
    //pwm_stop( 7 );
    //pwm_stop( 8 );
    //pwm_stop( 9 );
  // END CODE COPIED
  
  
  Serial.begin(115200);
  Serial.println("Beginning!"); 
  osmc = new OSMC(AHI, BHI, ALI, BLI, DIS);
  herk_brake = new HerkBrake(0xfd, 1, -72.0f);
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

// SUPPPORTING METHODS
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
      Serial.println(effort);
      break;
    case COM_CCW_MORE:
      effort -= EFFORT_ADJUSTMENT;
      Serial.println(effort);
      break;
    case COM_CW_SET:
      effort = (int)(parseInput());
      Serial.println(effort);
      break;
    case COM_CCW_SET:
      effort = -(int)(parseInput());
      Serial.println(effort);
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
