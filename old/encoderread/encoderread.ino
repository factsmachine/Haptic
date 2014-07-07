// Reads 12-bit encoder. Modified by Ed Foley from RobinL's 10-bit version for Arduino Uno.
 
// Define constants
const int CSn = 51; // Chip select pin
const int CLK = 52; // Clock signal pin
const int DO = 53;  // Digital output pin

unsigned int sensorWaarde = 0;

void setup(){
        // Establishes serial connection
        Serial.begin(115200);
  
	// Declare pins as inputs or outputs and their starting states as necessary
	pinMode(CSn, OUTPUT);
	pinMode(CLK, OUTPUT);
	pinMode(DO, INPUT);
	digitalWrite(CSn, HIGH);
	digitalWrite(CLK, HIGH);
}
 
void loop() {
	sensorWaarde = readSensor();
	delayMicroseconds(1); //Tcs waiting for another read in
}
 
unsigned int readSensor(){
	unsigned int dataOut = 0;
 
	digitalWrite(CSn, LOW);
	delayMicroseconds(1); //Waiting for Tclkfe
 
	//Passing 12 times, from 0 to 11
	for(int x=0; x<12; x++)
        {
		digitalWrite(CLK, LOW); 
		delayMicroseconds(1); //Tclk/2
		digitalWrite(CLK, HIGH);
		delayMicroseconds(1); //Tdo valid, like Tclk/2
		dataOut = (dataOut << 1) | digitalRead(DO); //shift all the entering data to the left and past the pin state to it. 1e bit is MSB
	}
 
	digitalWrite(CSn, HIGH);
        Serial.println(dataOut);
	return dataOut;
}
