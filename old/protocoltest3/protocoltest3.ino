char theChar;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);
  theChar = 'a';
}

void loop() {
  if (Serial.available())
  {
    theChar = Serial.read() + 1;
    delay(25);
    //Serial.println(theChar);
    Serial.write(theChar);
    //Serial.write('\r');
    //Serial.write('\n');
    //Serial.flush();
  }
  Serial.flush();  // Not really sure what effect this has, if any.
}
