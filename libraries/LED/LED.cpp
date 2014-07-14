#include "Arduino.h"
#include "LED.h"

LED::LED(short unsigned int pin_set)
{
	pin = pin_set;
	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
	on = false;
}

LED::LED(short unsigned int pin_set, bool on_set)
{
	pin = pin_set;
	pinMode(pin, OUTPUT);
	on = on_set;

	if (on) {
		digitalWrite(pin, HIGH);
	} else {
		digitalWrite(pin, LOW);
	}
}

void LED::turnOn()
{
	digitalWrite(pin, HIGH);
	on = true;
}

void LED::turnOff()
{
	digitalWrite(pin, LOW);
	on = false;
}

void LED::toggle()
{
	if (on)	{
		turnOff();
	} else {
		turnOn();
	}
}

bool LED::isOn()
{
	return on;
}