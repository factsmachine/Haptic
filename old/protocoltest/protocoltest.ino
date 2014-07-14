byte commandout[3];
byte commandin[3];
char pread;
char kread;
char nread;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);
  
  char p = 33;
  char k = 34;
  char n = 0;
  
  commandout[0] = p;
  commandout[1] = k;
  commandout[2] = n;
  
  Serial.print("Command from Unity to Arduino: ");
  
  for (int i=0; i<3; i++)
  {
    Serial.write(commandout[i]);
  }
  Serial.println();
  
  //Serial.print(p_char);
  //Serial.print(k_char);
  //Serial.println(n_char);
}

void loop() {
  if (Serial.available() == 3)
  {
    for (int i=0; i<3; i++)
    {
      commandin[i] = Serial.read();
    }
    
    Serial.print("Angle is ");
    Serial.println(byteToAngle(commandin[0]));
    Serial.print("Stiffness is ");
    Serial.println(byteToStiffness(commandin[1]));
    Serial.print("Normal is ");
    Serial.println(byteToNormal(commandin[2]));
  }

}

int byteToAngle(char in)
{
  return in;
}

int byteToStiffness(char in)
{
  return in;
}

bool byteToNormal(char in)
{
  return (in != 0);
}
