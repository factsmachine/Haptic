#include <encoder.h>

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

Encoder* enc;
int encread;
int sgread;
int csread;
bool eread;

void setup() {
  enc = new Encoder(CSn, CLK, DO);
  analogReadResolution(12);
  pinMode(BUTTON_PIN, INPUT);
  Serial.begin(9600);
  Serial.println("Beginning data stream.");
}

void loop() {
  // put your main code here, to run repeatedly:
  encread = enc->read();
  sgread = analogRead(SG);
  csread = analogRead(CURRENT_SENSE_PIN);
  eread = digitalRead(BUTTON_PIN) == LOW;
  
  Serial.print("ENC: ");
  Serial.print(encread);
  Serial.print(" SG: ");
  Serial.print(sgread);
  Serial.print(" CS: ");
  Serial.print(csread);
  Serial.print(" EMGCY: ");
  Serial.println(eread);
  
  delay(100);
}
