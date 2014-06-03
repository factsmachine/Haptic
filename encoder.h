#ifndef Encoder_h
#define Encoder_h
#include "Arduino.h"

class Encoder
{
	public:
		Encoder(int CS_set, int CLK_set, int DO_set);
		unsigned int read();
	private:
		int CS;
		int CLK;
		int DO;
};

#endif