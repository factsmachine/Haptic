void setup() {
  // put your setup code here, to run once:
  delay(1000);
  Serial.begin(1115200);
}

void loop() {
  delay(100);
  Serial.println(analogRead(0));
}
