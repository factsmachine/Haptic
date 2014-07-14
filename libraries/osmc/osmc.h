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

#ifndef OSMC_H
#define OSMC_H

//#include <WProgram.h>
#include <Arduino.h>

class OSMC {
public:
	OSMC(int AHI_set, int BHI_set, int ALI_set, int BLI_set, int DIS_set, long effort_limit_low_set, long effort_limit_high_set);
	void brake();
	//void unbrake();
	void disable();
	void enable();
	void setEffort(long effort_setting);
	long getEffort();
	bool isBraked();
	bool isDisabled();
	void setEffortLimitLow(long effort_limit_low_set);
	void setEffortLimitHigh(long effort_limit_high_set);
	void setEffortLimits(long effort_limit_low_set, long effort_limit_high_set);
	long getEffortLimitLow();
	long getEffortLimitHigh();

	//void forward(int effort_setting);
	//void reverse(int effort_setting);

private:
	int AHI, BHI, ALI, BLI, DIS;
	long effort, effort_limit_low, effort_limit_high;
	bool disabled, braked;

	void forward(long effort_setting);
	void reverse(long effort_setting);
};

#endif
