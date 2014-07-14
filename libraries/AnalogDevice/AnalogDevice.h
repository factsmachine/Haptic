#ifndef AnalogDevice_h
#define AnalogDevice_h
#include "Arduino.h"

class AnalogDevice
{
public:
	AnalogDevice(unsigned short int pin_set);
	unsigned int read();

	private:
		unsigned short int pin;
};

#endif