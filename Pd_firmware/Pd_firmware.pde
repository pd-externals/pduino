/* Copyright (C) 2006 Hans-Christoph Steiner 
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA  02111-1307  USA
 *
 * -----------------------------
 * Firmata, the Arduino firmware
 * -----------------------------
 * 
 * Firmata turns the Arduino into a Plug-n-Play sensorbox, servo
 * controller, and/or PWM motor/lamp controller.
 *
 * It was originally designed to work with the Pd object [arduino]
 * which is included in Pd-extended.  This firmware is intended to
 * work with any host computer software package.  It can easily be
 * used with other programs like Max/MSP, Processing, or whatever can
 * do serial communications.
 *
 * @authors: Hans-Christoph Steiner <hans@at.or.at>
 *   help with protocol redesign: Jamie Allen <jamie@heavyside.net>
 *   key bugfixes: Georg Holzmann <grh@mur.at>
 *                 Gerda Strobl <gerda.strobl@student.tugraz.at>
 * @date: 2006-05-19
 * @locations: STEIM, Amsterdam, Netherlands
 *             IDMI/Polytechnic University, Brookyn, NY, USA
 *             Electrolobby Ars Electronica, Linz, Austria
 */

/* 
 * TODO: add pulseOut functionality for servos
 * TODO: add software PWM for servos, etc (servo.h or pulse.h)
 * TODO: redesign protocol to accomodate boards with more I/Os
 * TODO: 
 * TODO: add "pinMode all 0/1" command
 * TODO: add cycle markers to mark start of analog, digital, pulseIn, and PWM
 * TODO: convert to MIDI protocol using SysEx for longer messages
 */

/* cvs version: $Id: Pd_firmware.pde,v 1.22 2006-12-06 03:29:06 eighthave Exp $ */

/* Version numbers for the protocol.  The protocol is still changing, so these
 * version numbers are important.  This number can be queried so that host
 * software can test whether it will be compatible with the currently
 * installed firmware. */
#define MAJOR_VERSION 0  // for non-compatible changes
#define MINOR_VERSION 3  // for backwards compatible changes

/* firmata protocol
 * =============== 
 * data: 0-127   
 * control: 128-255
 */
  
/* computer<->Arduino commands
 * -------------------- */
/* 128-129 // UNASSIGNED */
#define SET_PIN_ZERO_TO_IN      130 // set digital pin 0 to INPUT
#define SET_PIN_ONE_TO_IN       131 // set digital pin 1 to INPUT
#define SET_PIN_TWO_TO_IN       132 // set digital pin 2 to INPUT
#define SET_PIN_THREE_TO_IN     133 // set digital pin 3 to INPUT
#define SET_PIN_FOUR_TO_IN      134 // set digital pin 4 to INPUT
#define SET_PIN_FIVE_TO_IN      135 // set digital pin 5 to INPUT
#define SET_PIN_SIX_TO_IN       136 // set digital pin 6 to INPUT
#define SET_PIN_SEVEN_TO_IN     137 // set digital pin 7 to INPUT
#define SET_PIN_EIGHT_TO_IN     138 // set digital pin 8 to INPUT
#define SET_PIN_NINE_TO_IN      139 // set digital pin 9 to INPUT
#define SET_PIN_TEN_TO_IN       140 // set digital pin 10 to INPUT
#define SET_PIN_ELEVEN_TO_IN    141 // set digital pin 11 to INPUT
#define SET_PIN_TWELVE_TO_IN    142 // set digital pin 12 to INPUT
#define SET_PIN_THIRTEEN_TO_IN  143 // set digital pin 13 to INPUT
/* 144-149 // UNASSIGNED */
#define DISABLE_DIGITAL_INPUTS  150 // disable reporting of digital inputs
#define ENABLE_DIGITAL_INPUTS   151 // enable reporting of digital inputs
/* 152-159 // UNASSIGNED */
#define ZERO_ANALOG_INS         160 // disable reporting on all analog ins
#define ONE_ANALOG_IN           161 // enable reporting for 1 analog in (0)
#define TWO_ANALOG_INS          162 // enable reporting for 2 analog ins (0,1)
#define THREE_ANALOG_INS        163 // enable reporting for 3 analog ins (0-2)
#define FOUR_ANALOG_INS         164 // enable reporting for 4 analog ins (0-3)
#define FIVE_ANALOG_INS         165 // enable reporting for 5 analog ins (0-4)
#define SIX_ANALOG_INS          166 // enable reporting for 6 analog ins (0-5)
#define SEVEN_ANALOG_INS        167 // enable reporting for 6 analog ins (0-6)
#define EIGHT_ANALOG_INS        168 // enable reporting for 6 analog ins (0-7)
#define NINE_ANALOG_INS         169 // enable reporting for 6 analog ins (0-8)
/* 170-199 // UNASSIGNED */
#define SET_PIN_ZERO_TO_OUT     200 // set digital pin 0 to OUTPUT
#define SET_PIN_ONE_TO_OUT      201 // set digital pin 1 to OUTPUT
#define SET_PIN_TWO_TO_OUT      202 // set digital pin 2 to OUTPUT
#define SET_PIN_THREE_TO_OUT    203 // set digital pin 3 to OUTPUT
#define SET_PIN_FOUR_TO_OUT     204 // set digital pin 4 to OUTPUT
#define SET_PIN_FIVE_TO_OUT     205 // set digital pin 5 to OUTPUT
#define SET_PIN_SIX_TO_OUT      206 // set digital pin 6 to OUTPUT
#define SET_PIN_SEVEN_TO_OUT    207 // set digital pin 7 to OUTPUT
#define SET_PIN_EIGHT_TO_OUT    208 // set digital pin 8 to OUTPUT
#define SET_PIN_NINE_TO_OUT     209 // set digital pin 9 to OUTPUT
#define SET_PIN_TEN_TO_OUT      210 // set digital pin 10 to OUTPUT
#define SET_PIN_ELEVEN_TO_OUT   211 // set digital pin 11 to OUTPUT
#define SET_PIN_TWELVE_TO_OUT   212 // set digital pin 12 to OUTPUT
#define SET_PIN_THIRTEEN_TO_OUT 213 // set digital pin 13 to OUTPUT
/* 214-228 // UNASSIGNED */
#define OUTPUT_TO_DIGITAL_PINS  229 // next two bytes set digital output data 
/* 230-239 // UNASSIGNED */
#define REPORT_VERSION          240 // return the firmware version
/* 240-248 // UNASSIGNED */
#define SET_PIN_STATE           249 // set a digital pit to INPUT or OUTPUT
#define DISABLE_PWM             250 // next byte sets pin # to disable
#define ENABLE_PWM              251 // next two bytes set pin # and duty cycle
#define RESET                   254 // reset if receive 8 of these bytes
/* 255 // UNASSIGNED */

 
/* two byte digital output data format
 * ----------------------
 * 0  set digital output bytes (229/OUTPUT_TO_DIGITAL_PINS)
 * 1  digitalOut 7-13 bitmask 
 * 2  digitalOut 0-6 bitmask
 */

/* control PWM 
 * ----------------------
 * 0  send digital input bytes (ENABLE_PWM)
 * 1  pin # (0-127)
 * 2  duty cycle expressed as 1 byte (255 = 100%)
 */

/* digital input message format
 * ----------------------
 * 0   digital input marker (255/11111111)
 * 1   digital read from Arduino // 7-13 bitmask
 * 2   digital read from Arduino // 0-6 bitmask
 */

/* analog input message format
 * ----------------------
 * 0   analog input marker (160 + pin number reported)
 * 1   high byte from analog input
 * 2   low byte from analog input
 */

/* version report format
 * Send a single byte 240, Arduino will reply with:
 * ----------------------
 * 0  version report header (240)
 * 1  major version (0-127)
 * 2  minor version (0-127)
 */

/* PROPOSED PROTOCOL ADDITIONS */

/* set digital pin state (249/SET_PIN_STATE)
 * ----------------------
 * 0  set digital pin state
 * 1  pin number (0-127)
 * 2  state (OUTPUT/INPUT, 0/1) */

/* toggle analogIn reporting (249/SET_PIN_STATE)
 * ----------------------
 * 0  analogIn reporting mode
 * 1  pin number (0-127)
 * 2  state (0/1)
 */

/* control PWM 14-bit
 * ----------------------
 * 0  send digital input bytes (ENABLE_PWM)
 * 1  pin # (0-127)
 * 2  duty cycle, high bits (8-13)
 * 3  duty cycle, low bits (0-7)
 */


/* pulseIn (uses 32-bit value)
 * ----------------------
 * 0  pulseIn
 * 1  bits 24-31 (most significant byte)
 * 2  bits 16-23
 * 3  bits 8-15
 * 4  bits 0-7 (least significant byte)
 */

#define TOTAL_DIGITAL_PINS 14

// for comparing along with INPUT and OUTPUT
#define PWM 2

// maximum number of post-command data bytes
#define MAX_DATA_BYTES 2
// this flag says the next serial input will be data
byte waitForData = 0;
byte executeMultiByteCommand = 0; // command to execute after getting multi-byte data
byte storedInputData[MAX_DATA_BYTES] = {0,0}; // multi-byte data

// this flag says the first data byte for the digital outs is next
boolean firstInputByte = false;

/* store the previously sent digital inputs to compare against the current
 * digital inputs.  If there is no change, do not transmit. */ 
byte previousDigitalInputHighByte = 0;
byte previousDigitalInputLowByte = 0;
byte digitalInputHighByte = 0;
byte digitalInputLowByte = 0;

/* this int serves as a bit-wise array to store pin status
 * 0 = INPUT, 1 = OUTPUT  */
int digitalPinStatus = 0;

/* this byte stores the status off whether PWM is on or not
 * bit 9 = PWM0, bit 10 = PWM1, bit 11 = PWM2
 * the rest of the bits are unused and should remain 0  */
int pwmStatus = 0;

boolean digitalInputsEnabled = true;
// TODO: convert this to a bit array int, 1=report, 0=no report
byte analogInputsEnabled = 6;

byte analogPin;
int analogData;

// -------------------------------------------------------------------------
byte transmitDigitalInput(byte startPin) {
	byte i;
	byte digitalPin;
//  byte digitalPinBit;
	byte returnByte = 0;
	byte digitalData;

	for(i=0;i<7;++i) {
		digitalPin = i+startPin;
/*    digitalPinBit = OUTPUT << digitalPin;
// only read the pin if its set to input
if(digitalPinStatus & digitalPinBit) {
digitalData = 0; // pin set to OUTPUT, don't read
}
else if( (digitalPin >= 9) && (pwmStatus & (1 << digitalPin)) ) {
digitalData = 0; // pin set to PWM, don't read
}*/
		if( !(digitalPinStatus & (1 << digitalPin)) ) {
			digitalData = (byte) digitalRead(digitalPin);
			returnByte = returnByte + ((1 << i) * digitalData);
		}
	}
	return(returnByte);
}



// -------------------------------------------------------------------------
/* this function sets the pin mode to the correct state and sets the relevant
 * bits in the two bit-arrays that track Digital I/O and PWM status
 */
void setPinMode(int pin, int mode) {
	if(mode == INPUT) {
		digitalPinStatus = digitalPinStatus &~ (1 << pin);
		pwmStatus = pwmStatus &~ (1 << pin);
		pinMode(pin,INPUT);
	}
	else if(mode == OUTPUT) {
		digitalPinStatus = digitalPinStatus | (1 << pin);
		pwmStatus = pwmStatus &~ (1 << pin);
		pinMode(pin,OUTPUT);
	}
	else if( (mode == PWM) && (pin >= 9) && (pin <= 11) ) {
		digitalPinStatus = digitalPinStatus | (1 << pin);
		pwmStatus = pwmStatus | (1 << pin);
		pinMode(pin,OUTPUT);
	}
// TODO: save status to EEPROM here, if changed
}


/* -------------------------------------------------------------------------
 * this function checks to see if there is data waiting on the serial port 
 * then processes all of the stored data
 */
void checkForInput() {
	if(Serial.available()) {  
		while(Serial.available()) {
			processInput( (byte)Serial.read() );
		}
	}
}

/* -------------------------------------------------------------------------
 * processInput() is called whenever a byte is available on the
 * Arduino's serial port.  This is where the comm1ands are handled.
 */
void processInput(byte inputData) {
	int i;
	int mask;
  
	// a few commands have byte(s) of data following the command
	if( waitForData > 0) {  
		waitForData--;
		storedInputData[waitForData] = inputData;

		if(executeMultiByteCommand && (waitForData==0)) {
			//we got everything
			switch(executeMultiByteCommand) {
			case ENABLE_PWM:
				setPinMode(storedInputData[1],PWM);
				analogWrite(storedInputData[1], storedInputData[0]);
				break;
			case DISABLE_PWM:
				setPinMode(storedInputData[0],INPUT);
				break;
			}
			executeMultiByteCommand = 0;
		}
	}
	else if(inputData < 128) {
		if(firstInputByte) {
			// output data for pins 7-13
			for(i=7; i<TOTAL_DIGITAL_PINS; ++i) {
				mask = 1 << i;
				if( (digitalPinStatus & mask) && !(pwmStatus & mask) ) {
					// inputData is a byte and mask is an int, so align the high part of mask
					digitalWrite(i, inputData & (mask >> 7));
				}        
			}
			firstInputByte = false;
		}
		else { //
			for(i=0; i<7; ++i) {
				mask = 1 << i;
				if( (digitalPinStatus & mask) && !(pwmStatus & mask) ) {
					digitalWrite(i, inputData & mask);
				} 
			}
		}
	}
	else {
		switch (inputData) {
		case REPORT_VERSION:
			Serial.print(REPORT_VERSION, BYTE);
			Serial.print(MAJOR_VERSION, BYTE);
			Serial.print(MINOR_VERSION, BYTE);
			break;
		case ENABLE_PWM:
			waitForData = 2;  // 2 bytes needed (pin#, dutyCycle) 
			executeMultiByteCommand = inputData;
			break;
		case DISABLE_PWM:
			waitForData = 1;  // 1 byte needed (pin#)
			executeMultiByteCommand = inputData;
			break;      
		case OUTPUT_TO_DIGITAL_PINS:   // bytes to send to digital outputs
			firstInputByte = true;
			break;
		case DISABLE_DIGITAL_INPUTS:   // all digital inputs off
			digitalInputsEnabled = false;
			break;
		case ENABLE_DIGITAL_INPUTS:    // all digital inputs on
			digitalInputsEnabled = true;
			break;
		case ZERO_ANALOG_INS:   // analog input off
		case ONE_ANALOG_IN:     // analog 0 on  
		case TWO_ANALOG_INS:    // analog 0,1 on  
		case THREE_ANALOG_INS:  // analog 0-2 on  
		case FOUR_ANALOG_INS:   // analog 0-3 on  
		case FIVE_ANALOG_INS:   // analog 0-4 on  
		case SIX_ANALOG_INS:    // analog 0-5 on  
		case SEVEN_ANALOG_INS:  // analog 0-6 on  
		case EIGHT_ANALOG_INS:  // analog 0-7 on  
		case NINE_ANALOG_INS:   // analog 0-8 on  
			analogInputsEnabled = inputData - ZERO_ANALOG_INS;
			break;
		case SET_PIN_ZERO_TO_IN:       // set digital pins to INPUT
		case SET_PIN_ONE_TO_IN:
		case SET_PIN_TWO_TO_IN:
		case SET_PIN_THREE_TO_IN:
		case SET_PIN_FOUR_TO_IN:
		case SET_PIN_FIVE_TO_IN:
		case SET_PIN_SIX_TO_IN:
		case SET_PIN_SEVEN_TO_IN:
		case SET_PIN_EIGHT_TO_IN:
		case SET_PIN_NINE_TO_IN:
		case SET_PIN_TEN_TO_IN:
		case SET_PIN_ELEVEN_TO_IN:
		case SET_PIN_TWELVE_TO_IN:
		case SET_PIN_THIRTEEN_TO_IN:
			setPinMode(inputData - SET_PIN_ZERO_TO_IN, INPUT);
			break;
		case SET_PIN_ZERO_TO_OUT:      // set digital pins to OUTPUT
		case SET_PIN_ONE_TO_OUT:
		case SET_PIN_TWO_TO_OUT:
		case SET_PIN_THREE_TO_OUT:
		case SET_PIN_FOUR_TO_OUT:
		case SET_PIN_FIVE_TO_OUT:
		case SET_PIN_SIX_TO_OUT:
		case SET_PIN_SEVEN_TO_OUT:
		case SET_PIN_EIGHT_TO_OUT:
		case SET_PIN_NINE_TO_OUT:
		case SET_PIN_TEN_TO_OUT:
		case SET_PIN_ELEVEN_TO_OUT:
		case SET_PIN_TWELVE_TO_OUT:
		case SET_PIN_THIRTEEN_TO_OUT:
			setPinMode(inputData - SET_PIN_ZERO_TO_OUT, OUTPUT);
			break;
		}
	}
}


// =========================================================================

// -------------------------------------------------------------------------
void setup() {
	byte i;

// TODO: load state from EEPROM here

	Serial.begin(115200);	

/* TODO: send digital inputs here, if enabled, to set the initial state on the
 * host computer, since once in the loop(), the Arduino will only send data on
 * change. */

// flash the pin 13 with the protocol minor version (add major once > 0)
	pinMode(13,OUTPUT);
	for(i-0; i<MINOR_VERSION; i++) {
		digitalWrite(13,1);
		delay(100);
		digitalWrite(13,0);
		delay(200);
	}
	for(i=0; i<TOTAL_DIGITAL_PINS; ++i) {
		setPinMode(i,INPUT);
	}
}

// -------------------------------------------------------------------------
void loop() {
	checkForInput();  
  
	// read all digital pins, in enabled
	if(digitalInputsEnabled) {
		digitalInputHighByte = transmitDigitalInput(7);
		checkForInput();  
		digitalInputLowByte = transmitDigitalInput(0);
		checkForInput();  
		// only send data if it has changed
		if( (digitalInputHighByte != previousDigitalInputHighByte) && 
			(digitalInputLowByte != previousDigitalInputLowByte) ) {
			Serial.print(ENABLE_DIGITAL_INPUTS, BYTE);
			Serial.print(digitalInputHighByte, BYTE);
			Serial.print(digitalInputLowByte, BYTE);
			previousDigitalInputHighByte = digitalInputHighByte;
			previousDigitalInputLowByte = digitalInputLowByte;
		}
		checkForInput();
	}

	/* get analog in, for the number enabled */
	for(analogPin=0; analogPin<analogInputsEnabled; ++analogPin) {
		analogData = analogRead(analogPin);
		/* These two bytes get converted back into the whole number on host.
		  Highest bits should be zeroed so the 8th bit doesn't get set */
		Serial.print(ONE_ANALOG_IN + analogPin, BYTE);
		Serial.print(analogData >> 7, BYTE); // shift high bits into output byte
		Serial.print(analogData % 128, BYTE); // mod by 32 for the small byte
		checkForInput();
	}
}
