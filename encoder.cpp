#include "Arduino.h"
#include "encoder.h"

Encoder::Encoder(int CS_set, int CLK_set, int DO_set)
{
	CS = CS_set;
	CLK = CLK_set;
	DO = DO_set;

	pinMode(CS, OUTPUT);
	pinMode(CLK, OUTPUT);
	pinMode(DO, INPUT);

	digitalWrite(CS, HIGH);
	digitalWrite(CLK, HIGH);
}

unsigned int Encoder::read()
{
	unsigned int dataOut = 0;

	digitalWrite(CS, LOW);
	delayMicroseconds(1); //Waiting for Tclkfe

	//Passing 12 times, from 0 to 11
	for (int x = 0; x<12; x++)
	{
		digitalWrite(CLK, LOW);
		delayMicroseconds(1); // Tclk/2
		digitalWrite(CLK, HIGH);
		delayMicroseconds(1); // Tdo,valid (like Tclk/2)
		dataOut = (dataOut << 1) | digitalRead(DO); // Shift data to left and read bits as appropriate. Left is MSB.
	}

	digitalWrite(CS, HIGH);
	return dataOut;
}