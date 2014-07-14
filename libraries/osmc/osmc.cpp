/*
Copyright (c) 2009 Chris B Stones (welcometochrisworld.com)

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
*/

#include "osmc.h"
#include "../pwm01/pwm01.h"

//const long EFFORT_LIMIT_HIGH = 65535;
//const long EFFORT_LIMIT_LOW = -65535;
const int BASE_PWM_FREQUENCY = 5000; // changed from 10000

OSMC::OSMC(int AHI_set,int BHI_set,int ALI_set,int BLI_set, int DIS_set, long effort_limit_low_set, long effort_limit_high_set) {
	AHI = AHI_set;
	BHI = BHI_set;
	ALI = ALI_set;
	BLI = BLI_set;
	DIS = DIS_set;
	effort_limit_low = effort_limit_low_set;
	effort_limit_high = effort_limit_high_set;
	braked = false;
	disabled = false;
	effort = 0;

	pinMode(AHI, OUTPUT);
	pinMode(BHI, OUTPUT);
	pinMode(ALI, OUTPUT);
	pinMode(BLI, OUTPUT);
	pinMode(DIS, OUTPUT);
	digitalWrite(AHI, HIGH);
	digitalWrite(BHI, HIGH);
	digitalWrite(ALI, LOW);
	digitalWrite(BLI, LOW);
	digitalWrite(DIS, LOW);

	pwm_set_resolution(16);
	pwm_setup(ALI, BASE_PWM_FREQUENCY, 1);
	pwm_setup(BLI, BASE_PWM_FREQUENCY, 2);
	pwm_write_duty(ALI, 0);
	pwm_write_duty(BLI, 0);
}

void OSMC::forward(long effort_setting) {
  digitalWrite(AHI, HIGH);
  digitalWrite(BHI, HIGH);
  pwm_write_duty(ALI, 0);
  pwm_write_duty(BLI, effort_setting);
  digitalWrite(DIS, LOW);
}

void OSMC::reverse(long effort_setting) {
  digitalWrite(AHI, HIGH);
  digitalWrite(BHI, HIGH);
  pwm_write_duty(ALI, effort_setting);
  pwm_write_duty(BLI, 0);
  digitalWrite(DIS, LOW);
}

void OSMC::brake() {
  digitalWrite(AHI, HIGH);
  digitalWrite(BHI, HIGH);
  pwm_write_duty(ALI, 0);
  pwm_write_duty(BLI, 0);
  digitalWrite(DIS, LOW);
  braked = true;
}

void OSMC::disable() {
	digitalWrite(DIS, HIGH);
	disabled = true;
}

void OSMC::enable() {
	digitalWrite(DIS, LOW);
	disabled = false;
}

void OSMC::setEffort(long effort_setting) {
	int temp_effort = effort_setting;

	if (temp_effort > effort_limit_high) {
		temp_effort = effort_limit_high;
	}
	else if (temp_effort < effort_limit_low) {
		temp_effort = effort_limit_low;
	}

	if (temp_effort >= 0) {
		this->forward(temp_effort);
	}
	
	if (temp_effort < 0) {
		this->reverse(-1 * temp_effort);
	}

	effort = temp_effort;
	braked = false;
}

long OSMC::getEffort() {
	return effort;
}

bool OSMC::isBraked() {
	return braked;
}

bool OSMC::isDisabled() {
	return disabled;
}

void OSMC::setEffortLimitLow(long effort_limit_low_set) {
	effort_limit_low = effort_limit_low_set;
	
	// WARNING: WHAT IF LOW LIMIT IS SET HIGHER THAN HIGH LIMIT???
	if (effort < effort_limit_low)
	{
		if (!braked) {
			setEffort(effort_limit_low);
		}
		else {
			effort = effort_limit_low;
		}
	}
}

void OSMC::setEffortLimitHigh(long effort_limit_high_set) {
	effort_limit_high = effort_limit_high_set;

	// WARNING: WHAT IF HIGH LIMIT IS SET LOWER THAN LOW LIMIT???
	if (effort > effort_limit_high)
	{
		if (!braked) {
			setEffort(effort_limit_high);
		}
		else {
			effort = effort_limit_high;
		}
	}
}

long OSMC::getEffortLimitLow() {
	return effort_limit_low;
}

long OSMC::getEffortLimitHigh() {
	return effort_limit_high;
}

void OSMC::setEffortLimits(long effort_limit_low_set, long effort_limit_high_set) {
	setEffortLimitLow(effort_limit_low_set);
	setEffortLimitHigh(effort_limit_high_set);
}