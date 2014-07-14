char theChar;

void setup() {
  Serial.begin(19200);
}

void loop() {
  //delay(5);
  if (Serial.available())
  {
    theChar = Serial.read();
    //Serial.println(theChar);
    Serial.write(theChar);
  }
}
