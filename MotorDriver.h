#ifndef MotorDriver_h
#define MotorDriver_h
#include "Arduino.h"

class MotorDriver
{
public:
	MotorDriver(int INA_set, int INB_set, int PW_set);
	void setEffort(int effortSetting);
	int getEffort();
	void brake();
	void unbrake();
	bool isBraked();
private:
	int effort;
	bool braked;
	int INA;
	int INB;
	int PW;
};

#endif