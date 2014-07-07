//#include <HerkulexEdit.h>
#include <HerkBrake.h>
//int n=0xfd; //motor ID - verify your ID !!!!
//HerkBrake herk_brake(n, 1, 20.0f);
HerkBrake* herk_brake;
bool pressedInNow = false;
bool pressedInLast = false;

const int BUTTON_PIN = 22;

void setup()  
{
  delay(2000);  //a delay to have time for serial monitor opening
  Serial.begin(115200);    // Open serial communications
  Serial.println("Begin");
  delay(200);
  //herk_brake = new HerkBrake(n, 1, 20.0f);
  herk_brake = new HerkBrake(0xfd, 1, -72.0f);
  
  pinMode(22, INPUT);
}

void loop(){
  delay(200);
  Serial.println(herk_brake->getAngle());
  
  pressedInNow = (digitalRead(BUTTON_PIN) == LOW);
  if (pressedInNow && !pressedInLast) {
    Serial.println("Braking!");
    herk_brake->brake();
  } else if (!pressedInNow && pressedInLast) {
    Serial.println("Unbraking!");
    herk_brake->unbrake();
  }
  pressedInLast = pressedInNow;
}
