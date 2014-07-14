#ifndef LED_h
#define LED_h
#include "Arduino.h"

class LED
{
	public:
		LED(unsigned short int pin_set);
		LED(unsigned short int pin_set, bool on_set);
		void turnOn();
		void turnOff();
		void toggle();
		bool isOn();
	private:
		unsigned short int pin;
		bool on;
};

#endif