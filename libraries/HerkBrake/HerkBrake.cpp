#include "Arduino.h"
#include "HerkBrake.h"
#include "../Herkulex/HerkulexEdit.cpp"

const int BRAKE_TIME = 0; // Minimum duration of servo motion. Set to 0 for max speed.

HerkBrake::HerkBrake(int motor_ID_set, int serial_port_set, float brake_travel_set)
{
	brake_travel = brake_travel_set;
	braked = false;
	serial_port = serial_port_set;
	motor_ID = motor_ID_set;

	switch (serial_port) {
	case 1:
		Herkulex.beginSerial1(115200);
		break;
	case 2:
		Herkulex.beginSerial2(115200);
		break;
	case 3:
		Herkulex.beginSerial3(115200);
		break;
	}

	Herkulex.reboot(motor_ID);
	delay(500);					// Delays given in example code
	Herkulex.initialize();
	delay(200);
	zero_pos = Herkulex.getAngle(motor_ID);
	Herkulex.setLed(motor_ID, 1);
}

void HerkBrake::brake()
{
	braked = true;
	Herkulex.moveOneAngle(motor_ID, fmod(zero_pos + brake_travel, 360.0f), BRAKE_TIME, 2);
}

void HerkBrake::unbrake()
{
	braked = false;
	Herkulex.moveOneAngle(motor_ID, zero_pos, BRAKE_TIME, 3);
}

bool HerkBrake::isBraked()
{
	return braked;
}

void HerkBrake::setBrakeTravel(float travel_setting)
{
	brake_travel = travel_setting;
}

float HerkBrake::getBrakeTravel()
{
	return brake_travel;
}