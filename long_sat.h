/*--------------------------------------------------------------------//
 //----------------------------- long sat ----------------------------//
 This is a set of utility functions for convenient saturated multiplication
 of 32 bit signed integers. Functions are based entirely on 
 http://locklessinc.com/articles/sat_arithmetic/
 
 Being utility functions, I've opted to put them in a namespace, not a class
 wrapper. This should do an okay job of making sure that they can be accessed by any function
 that includes the appropriate .h file. Does not require construction/initialization
 
 call by saying:
 
 long_sat::relevant_function(arguments)
 
 OR OR OR
 
 using namespace long_sat
 relevant_function(arguments)
 
 
 LICENSE: This software is free to use, modify and distribute with a
two resitrictions: 
	1) 	Any modification should be documented in the change log below. 
	2) 	This code may be used in commercial devices, but only as part of
		a combined physical device and control system.
		
CHANGE LOG:
	3-19-13 Creation! Added a saturated multiplication routine
	3-19-13 Progress! Added the saturated addition, subtraction and division routines
			
VERSION:
	0.80
 */
 #ifndef long_sat_h
 
 #define INT_MAX 2147483647
#define INT_MIN -2147483648
 
#define long_sat_h

namespace long_sat
{
long sat_addslong(long x,long y);
long sat_subslong(long x, long y);
long sat_mulslong(long x,long y);
long sat_divslong(long x,long y);
};
#endif
