#include <stdlib.h>     // needed for atoi
char buffer[4];
int received;
int ledPin = 9;
void setup()
{
    Serial.begin(9600);
    received = 0;
    buffer[received] = '\0';
}
void loop()
{
    if (Serial.available())
    {
        buffer[received++] = Serial.read();
        buffer[received] = '\0';
        if (received >= (sizeof(buffer)-1))
        {
            Serial.print(buffer);
            int myInt = atoi(buffer);
            //analogWrite(ledPin, myInt);
            Serial.println(myInt);
            received = 0;
        }
    }
}
