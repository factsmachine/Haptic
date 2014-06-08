#include "Arduino.h"
#include "MotorDriver.h"

MotorDriver::MotorDriver(int INA_set, int INB_set, int PW_set)
{
	INA = INA_set;
	INB = INB_set;
	PW = PW_set;

	pinMode(INA, OUTPUT);
	pinMode(INB, OUTPUT);
	pinMode(PW, OUTPUT);

	digitalWrite(INA, LOW);
	digitalWrite(INB, LOW);
	digitalWrite(PW, LOW);

	effort = 0;
	braked = false;
}

void MotorDriver::setEffort(int effortSetting)
{
	effort = effortSetting;
	effort = min(255, max(-255, effort));

	if (!braked)
	{
		if (effortSetting >= 0) {
			digitalWrite(INA, LOW);
			digitalWrite(INB, HIGH);
			analogWrite(PW, min(255, max(-255, effort)));
		}
		else {
			digitalWrite(INA, HIGH);
			digitalWrite(INB, LOW);
			analogWrite(PW, min(255, max(-255, -effort)));
		}
	}
}

int MotorDriver::getEffort()
{
	return effort;
}

void MotorDriver::brake()
{
	digitalWrite(INA, HIGH);
	digitalWrite(INB, HIGH);
	braked = true;
}

void MotorDriver::unbrake()
{
	braked = false;
	setEffort(effort);
}

bool MotorDriver::isBraked()
{
	return braked;
}