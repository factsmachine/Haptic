const int BUTTON_PIN = 22;
const int LED_PIN = 13;

//char theChar;
bool pushed = false;
bool pushedlast = false;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);
  //theChar = 'a';
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  Serial.println('H');
  delay(10);
  
  if (Serial.available())
  {
    if (Serial.read() == 'a')
    {
      digitalWrite(LED_PIN, HIGH);
    }
  }

  //delay(9);
  
  
  //pushedlast = pushed;
  //pushed = digitalRead(BUTTON_PIN) == LOW;
  
  //if (pushed && !pushedlast)
  //{
  //  Serial.println('Pushed');
  //} 
  //else if (!pushed && pushedlast)
  //{
  //  Serial.println('Released');
  //}
}
