#ifndef HerkBrake_h
#define HerkBrake_h
#include "Arduino.h"
#include "../Herkulex/HerkulexEdit.h"

class HerkBrake
{
public:
	HerkBrake(int motor_ID_set, int serial_port_set, float brake_travel_set);
	void brake();
	void unbrake();
	bool isBraked();
	float getBrakeTravel();
	void setBrakeTravel(float travel_setting);
private:
	bool braked;
	float brake_travel;
	float zero_pos;
	int serial_port;
	int motor_ID;
};

#endif