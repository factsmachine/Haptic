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

#include <osmc.h>

/* When wiring up an OSMC Board this values correspond to the HIP4081A chip specs. 
Refer to that chips data sheet to figure out which pins to wire to. 
Pin 2 on the HIP Chip is BHI
Pin 3 on the HIP Chip is DIS
Pin 5 on the HIP Chip is BLI
Pin 6 on the HIP Chip is ALI
Pin 7 on the HIP Chip is AHI
etc..
*/ 

/* With this Very Basic library you need 5 pins per
   OSMC board. They all are I/O pins but 2 of them have to 
   be PWM capable 
*/
int AHI     = 8;     // normal I/O pin
int BHI     = 7;     // normal I/O pin
int ALI     = 6;     // must be on a PWM I/O pin
int BLI     = 5;     // must be on a PWM I/O pin
int disable_pin = 4; // normal I/O pin


OSMC osmc;
void setup() {
  osmc.init(AHI,BHI, ALI, BLI, disable_pin); 
}

// simply show forward and reverse at some speed.
int motor_speed = 200; // a value between 0 and 255
void loop() {
  osmc.forward(motor_speed);
  delay(500);
  osmc.reverse(motor_speed);
  delay(500);
}

// NOTE: I have also designed some specail hardware to 
//       allow control with only 3 pins so that the lib can
//       be called like this //osmc.init(toggle,speed,disable); // specail hardware
//       But my lib does not support that option right now. So if you 
//       really want to control more OSMC boards with fewer pins let me know be leaving
//       comments on my site. welcometochrisworld.com And if there is enough response
//       I might do it. We'll see...
