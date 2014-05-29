/*--------------------------------------------------------------------//
 //----------------------------- PID LITE ----------------------------//
 This is a control library with basic PID functionality. This includes
 proportional, integral and derivative action, as well as setting gains
 and activating/deactivating the controller. Additionally, users can set
 a saturation point, this includes anti-windup integrator dumping at that
 saturation point.
 
 NOTE: what makes this a "light" PID controller is that it does not use
 floats within the main compute() cycle. KI, KP and KD are set as floats,
 but the class converts these into scaled integers.
 
 LICENSE: This software is free to use, modify and distribute with a
two resitrictions: 
	1) 	Any modification should be documented in the change log below. 
	2) 	This code may be used in commercial devices, but only as part of
		a combined physical device and control system.
		
CHANGE LOG:
	2-25-13	added contstructor, main variables and methods.
			PID_lite currently supports parameter tuning, 
			saturation limits and activation/deactivation.
			-----MG
	3-19-13 recognized saturation issues, changed from dumping cumerr
			on output saturation to simply flagging it to skip.
			Arithmetic overflow has been considered an issue, currently
			building a saturated arithmetic library to cope
	3-19-13 progress! 
			
VERSION:
	0.80
 */
#include <long_sat.h>

#ifndef PID_lite_h

#define PID_lite_h

class PID_lite
{

public:

//Constructor- pointers to the long integers for input, output and target, as well as all three gains
//The last call is the interval in milliseconds of the compute() execution. 
PID_lite();
PID_lite(long*, long*, long*, float, float, float, int );

//Computing should have a millisecond value for the point in the execution cycle fed to it- for arduino
//operation, simply call it as myPID.compute(millis()). Regardless, the PID algorithm proper only runs if
//enough time has elapsed and the controller has been enabled by activate()
bool compute(long);

//Set parameters changes all of the gains, feed it floats.  
void setParameters(float , float , float);

//Set saturation, long integers for the minimum and maximum, in that order
void setSaturation(long,long);

//Calling activate() enables the PID algorithm
void activate();

//deactivate() deactivates the algorithm, as well as dumping cumulative error. 
void deactivate();

//setInterval changes the interval of the controller
void setInterval(long);

//Getting the integer gains
long getKP();

long getKD();

long getKI();

void dump();

long getCumerr();

bool getSaturateFlag();



private:
#define THAT_SCALE 4096
/*
const long THAT_SCALE = 4096;	//this is the offset to get floating point behavior out of
								//integers. Should be a power of 2.
								*/
long Kp, Ki, Kd;
long error;          //offset between target and actual
long cumerr;         //cumulative error
long dererr;		//derivative component of error
long last_input;	//previous sensor reading
long* p_input;         //actual sensor reading
long* p_target;         //target sensor reading
long* p_output;      //output output

long upper_output_limit, lower_output_limit;  //the ends of the output

long interval;         //interval of computation
long internal_timer;   //this is the millisecond value of the runtime

bool active;           //flag checked to see if the controller should run
bool saturated;

};
#endif