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
 
 #include <long_sat.h>
 namespace long_sat
 {
	long sat_addslong(long x,long y)
	{
	unsigned long ux = x;
	unsigned long uy = y;
	unsigned long res = ux + uy;
	
	/* Calculate overflowed result. (Don't change the sign bit of ux) */
	ux = (ux >> 31) + INT_MAX;
	
	/* Force compiler to use cmovns instruction */
	if ((signed long) ((ux ^ uy) | ~(uy ^ res)) >= 0)
	{
		res = ux;
	}
		
	return res;
	};
	long sat_subslong(long x, long y)
	{
	unsigned long ux = x;
	unsigned long uy = y;
	unsigned long res = ux - uy;
	
	ux = (ux >> 31) + INT_MAX;
	
	/* Force compiler to use cmovns instruction */
	if ((signed long)((ux ^ uy) & (ux ^ res)) < 0)
	{
		res = ux;
	}
		
	return res;
	};
	
	long sat_mulslong(long x, long y)
	{
		long long res = (long long) x * (long long) y;
		unsigned long res2 = ((unsigned long) (x ^ y) >> 31) + INT_MAX;
		
		long hi = (res >> 32);
		long lo = res;
	
		if (hi != (lo >> 31)) res = res2;
	
		return res;
	};
	long sat_divslong(long x,long y)
	{
		/* Only one way to overflow, so test for and prevent it. */
	x += !((y + 1) | ((unsigned long) x + INT_MIN));
		
	return x / y;
	};
};