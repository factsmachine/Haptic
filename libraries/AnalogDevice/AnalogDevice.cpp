#include "Arduino.h"
#include "AnalogDevice.h"

AnalogDevice::AnalogDevice(short unsigned int pin_set)
{
	pin = pin_set;
	pinMode(pin, INPUT);
}

unsigned int AnalogDevice::read()
{
	return analogRead(pin);
}