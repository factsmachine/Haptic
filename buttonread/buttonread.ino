/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
 
int ON_TIME_MAX = 200;
int OFF_TIME_MAX = 200;
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;
int extLED = 11;
int button = 2;
int state;
int lastState;
int onTime = ON_TIME_MAX;
int offTime = OFF_TIME_MAX;
int onCount = 0;
int offCount = 0;
int BUTTON_PUSHED = LOW;
bool ledOn = true;
int accelCounter = 0;
int ACCEL_MAX = 20;
int pulseCount = 0;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);    
  pinMode(button, INPUT);
  pinMode(extLED, OUTPUT);
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  state = digitalRead(button);
  
  if (wasStateChange())
  {
    Serial.println(state);
  }
  
  if (state == BUTTON_PUSHED)
  {
    digitalWrite(extLED, 0);
    if (ledOn)
    {
      digitalWrite(led, HIGH);
      onCount++;
      if (onCount >= onTime)
      {
        ledOn = false;
        onCount = 0;
        offCount = 0;
      }
    }
    else
    {
      digitalWrite(led, LOW);
      offCount++;
      if (offCount >= offTime)
      {
        ledOn = true;
        offCount = 0;
        onCount = 0;
      }
    }
    
    accelCounter++;
    if (accelCounter >= ACCEL_MAX)
    {
      decTime();
      accelCounter = 0;
    }    
  }
  else
  {
    digitalWrite(led, LOW);
    ledOn = true;
    onTime = ON_TIME_MAX;
    offTime = OFF_TIME_MAX;
    onCount = 0;
    offCount = 0;
    accelCounter = 0;
    
    
    pulseCount++;
    if (pulseCount/4 >= 256)
      pulseCount = 0;
    analogWrite(extLED, pulseCount/4);
  }
  
  lastState = state;
  delay(1);
}

void decTime()
{
  if (onTime > 1)
    {
      onTime--;
    }
    if (offTime > 1)
    {
      offTime--;
    }
}

bool wasStateChange()
{
  return (lastState != state);
}
