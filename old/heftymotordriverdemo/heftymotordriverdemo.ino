#include <osmc.h>
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

int STATE = 0;

// Button variables
bool pressedInNow = false;
bool pressedInLast = false;

OSMC* osmc;

void setup() {
  // INCREASE PWM FREQUENCY!!!
  
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
      osmc->setEffort(20);
    break;
    
    case 2:
      osmc->brake();
      
  
  
  Serial.println("ADVANCE!"); 
  osmc->setEffort(20);
  Serial.println(osmc->getEffort());
  delay(2000);
  Serial.println("RETREAT!");
  osmc->setEffort(-20);
  Serial.println(osmc->getEffort());
  delay(2000);
}
