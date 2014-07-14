#ifndef Button_h
#define Button_h
#include "Arduino.h"

class Button
{
public:
	Button(unsigned short int pin_set, bool active_high_set);
	bool isPushed();

	private:
		unsigned short int pin;
		bool active_high;
};

#endif