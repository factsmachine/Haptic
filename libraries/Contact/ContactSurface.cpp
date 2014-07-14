#include "Arduino.h"
#include "Contact.h"

Contact::Contact(float* position, int )
{
	pin = pin_set;
	active_high = active_high_set;
	pinMode(pin, INPUT);
}

bool Button::isPushed()
{
	return !active_high ^ (digitalRead(pin) == HIGH);
}