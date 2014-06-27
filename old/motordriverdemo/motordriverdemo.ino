#include <Servo.h>

Servo serv;

void setup() {
  serv.attach(10, 1000, 2000);
  serv.writeMicroseconds(1500);
}

void loop() {
  // put your main code here, to run repeatedly:

}
