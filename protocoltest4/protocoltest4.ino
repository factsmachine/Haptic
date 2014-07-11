const int BUTTON_PIN = 22;

//char theChar;
bool pushed = false;
bool pushedlast = false;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);
  //theChar = 'a';
  pinMode(BUTTON_PIN, INPUT);
}

void loop() {
  pushedlast = pushed;
  pushed = digitalRead(BUTTON_PIN) == LOW;
  
  if (pushed && !pushedlast)
  {
    Serial.println('Pushed');
  } 
  else if (!pushed && pushedlast)
  {
    Serial.println('Released');
  }
}
