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
 */
#include <PID_lite.h>
#include <long_sat.h>
PID_lite::PID_lite(){
}
PID_lite::PID_lite(long* INPUT, long* OUTPUT, long* TARGET, float KP, float KI, float KD, int INTERVAL) ///inlcude something for the reading, something for the stuff
{
 
  //First, all floats are transferred into long integers- this is accomplished by multiplying them by scalign factor
  //Note that this means that any gain less than 1/THAT_SCALE will be zero!
  PID_lite::setParameters(KP,KI,KD);
  
  //Default output limits, brah- optimized for the arduino style PWM output.
  PID_lite::setSaturation(0,255);
  
//We need to bring the pointers into the newly-created object's variables
  p_input = INPUT;
  p_output = OUTPUT;
  p_target = TARGET;
  interval = INTERVAL;

//internal timer should be initialized to zero
internal_timer = 0;

PID_lite::deactivate();		//Start with the controller off, brah
}

bool PID_lite::compute(long the_timer)
{

  if(active)    //First thing to check is if we should be running at all
  {
    // This is a cooperative multitasking function, so it operates on a 
    // timer. If it's not the time, this is a mighty short function call.
    if(internal_timer < the_timer)
    {
	
	/* SUPER IMPORTANT QUESTION TO ASK: the Ki/Kp values to be used here are fucking wonky- namely, they aren't
	set to a second base- rather, they are on millisecond base (at least Ki and Kd); this is due to the effect
	of dividing/multiplying by the interval size, which is itself in milliseconds. I have compensated for this
	in the setting parameters section by scaling the respective gains. God save us all. */
	
	//A little bit of pointer massaging
	long input = *p_input;
	long target = *p_target;
	
	//Here, we determine the current (proportional), integral and derivative error.
	using namespace long_sat;
	error = sat_addslong(input,-target);
	if(saturated==false)
	{
	//This is a new one for me, so a bit of explanation is due:
	//the long_sat family of functions handles saturating 32bit ints
	//Here, we have a couple of possibilities for overflow with our
	//signed 32 bit ints, so we accounte for that
	//Mathematically, it's just cumerr = Ki*error*interval + cumerr,
	// but it takes care of overflow at each step
	using namespace long_sat;
		cumerr = sat_addslong(sat_mulslong(Ki,sat_mulslong(error,interval)),cumerr);
	}
	using namespace long_sat;
	dererr = sat_mulslong(Kd,(sat_divslong(sat_addslong(input, -last_input),interval)));
	
	last_input = input;	//overwrite the previous reading with the current to prep for the next execution cycle
	
	//output is all the integer gains time their respective errors, divided by the intergerization value
	using namespace long_sat;
    long output = sat_addslong(sat_addslong((Kp*error)/THAT_SCALE, cumerr/THAT_SCALE/1024),dererr/THAT_SCALE);
	//long output = (Kp*error)/THAT_SCALE;
    //saturation is, simply, checking against the saturation limit and stopping at that
 
if(output < lower_output_limit)
      {
        output = lower_output_limit;
        saturated = true;     //This prevents integrator windup. DOPE
      }
else  if(output > upper_output_limit)
    {
		output = upper_output_limit;
        saturated = true;      //This prevents integrator windup. DOPE
    }
	  else
	  {
	  saturated = false;	//gotta leave the saturation off
	  }
	  *p_output = output;	//last step is to write the output to the place pointed to by p_output

      internal_timer += interval;   //last step is to reset the internal timer
	  return true;
    }
	else
	{
	return false;
	}
  }
  else
  {
  return false;
  }
}

void PID_lite::setParameters(float KP, float KI, float KD)
{

//Since we don't want floats when we're doing controller computations, but floats are very nice for 
//modifying the parameters, they are set as floats, but translated into scaled integers.
  KP = KP*THAT_SCALE;
  KD = KD*THAT_SCALE*1000;
  KI = KI*THAT_SCALE;
  Kp = KP;
  Kd = KD;
  Ki = KI;
  //note: the 1000 factor is to compensate for the millisecond/second discrepancy (see compute())
}

void PID_lite::setSaturation(long LOW,long HIGH)
{
//nice and simple
lower_output_limit = LOW;
upper_output_limit = HIGH;
}

void PID_lite::activate()        //Set the active flag on
{
  active = true;
  last_input = *p_input;		//
}


void PID_lite::deactivate()      //Set the active flag off, and most importantly, clear the error counts
{
saturated= false;
  active = false;
  error = 0;
  cumerr = 0;
  dererr = 0;
}

long PID_lite::getKP()
{
return Kp;
}

long PID_lite::getKD()
{
return Kd;
}
long PID_lite::getKI()
{
return Ki;
}

void PID_lite::dump()
{
cumerr = 0;
}

long PID_lite::getCumerr()
{
return cumerr;
}

bool PID_lite::getSaturateFlag()
{
return saturated;
}

void PID_lite::setInterval(long newInterval)
{
interval = newInterval;
}
