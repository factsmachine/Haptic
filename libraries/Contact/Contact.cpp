#include "Arduino.h"
#include "Button.h"

Button::Button(short unsigned int pin_set, bool active_high_set)
{
	pin = pin_set;
	active_high = active_high_set;
	pinMode(pin, INPUT);
}

bool Button::isPushed()
{
	return !active_high ^ (digitalRead(pin) == HIGH);
}