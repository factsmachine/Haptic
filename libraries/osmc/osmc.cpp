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

const int EFFORT_LIMIT_HIGH = 4095;
const int EFFORT_LIMIT_LOW = -4095;
const int BASE_PWM_FREQUENCY = 2222;

OSMC::OSMC(int AHI_set,int BHI_set,int ALI_set,int BLI_set, int DIS_set) {
	AHI = AHI_set;
	BHI = BHI_set;
	ALI = ALI_set;
	BLI = BLI_set;
	DIS = DIS_set;
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

	pwm_set_resolution(12);
	pwm_setup(ALI, BASE_PWM_FREQUENCY, 1);
	pwm_setup(BLI, BASE_PWM_FREQUENCY, 2);
	pwm_write_duty(ALI, 0);
	pwm_write_duty(BLI, 0);
}

void OSMC::forward(int effort_setting) {
  digitalWrite(AHI, HIGH);
  digitalWrite(BHI, HIGH);
  pwm_write_duty(ALI, 0);
  pwm_write_duty(BLI, effort_setting);
  digitalWrite(DIS, LOW);
}

void OSMC::reverse(int effort_setting) {
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

void OSMC::setEffort(int effort_setting) {
	int temp_effort = effort_setting;

	if (temp_effort > EFFORT_LIMIT_HIGH) {
		temp_effort = EFFORT_LIMIT_HIGH;
	}
	else if (temp_effort < EFFORT_LIMIT_LOW) {
		temp_effort = EFFORT_LIMIT_LOW;
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

int OSMC::getEffort() {
	return effort;
}

bool OSMC::isBraked() {
	return braked;
}

bool OSMC::isDisabled() {
	return disabled;
}