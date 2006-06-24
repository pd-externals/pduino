/* Pd_firmware aka Pduino
 * ------------------
 * 
 * It was designed to work with the Pd object [arduino]
 * which is included in Pd-extended.  This firmware could 
 * easily be used with other programs like Max/MSP, Processing,
 * or whatever can do serial communications.
 *
 * (copyleft) 2006 Hans-Christoph Steiner <hans@at.or.at>
 * @author: Hans-Christoph Steiner
 * @date: 2006-05-19
 * @location: STEIM, Amsterdam, Netherlands
 */

/* TODO
 *
 * - get digitalInput working
 * - add pulseIn functionality
 * - add software PWM for servos, etc
 * - redesign protocol to accomodate boards with more I/Os
 * - add cycle markers to mark start of analog, digital, pulseIn, and PWM
 */

/*
 * Pduino protocol
 * =============== 
 * data: 0-127   
 * control: 128-255
 * 
 * Pd->Arduino commands
 * --------------------
 * 200-213 - set digital pin 0-13 to INPUT
 * 214-227 - set digital pin 0-13 to OUTPUT
 * 228 - UNUSED
 * 229 - UNUSED
 * 230 - next byte sets PWM0 value
 * 231 - next byte sets PWM1 value
 * 232 - next byte sets PWM2 value
 * 233 - UNUSED
 * 234 - UNUSED
 * 235 - UNUSED
 * 236 - UNUSED
 * 237 - UNUSED
 * 238 - disable all digital inputs
 * 239 - enable all digital inputs
 * 240 - disable all analog inputs
 * 241 - enable 1 analog input (0)
 * 242 - enable 2 analog inputs (0,1)
 * 243 - enable 3 analog inputs (0-2)
 * 244 - enable 4 analog inputs (0-3)
 * 245 - enable 5 analog inputs (0-4)
 * 246 - enable 6 analog inputs (0-5)
 * 255 - cycle marker
 *
 * Pd->Arduino byte cycle
 * ----------------------
 * 0  start of cycle marker (255/11111111)
 * 1  digitalOut 0-6 bitmask
 * 2  digitalOut 7-13 bitmask
 * 
 * Arduino->Pd byte cycle
 * ----------------------
 * 0 start of cycle marker (255/11111111)
 * 1  digitalIn 0-6 bitmask
 * 2  digitalIn 7-13 bitmask
 * 3  analogIn0 byte0
 * 4  analogIn0 byte1
 * 5  analogIn1 byte0
 * 6  analogIn1 byte1
 * 7  analogIn2 byte0
 * 8  analogIn2 byte1
 * 9  analogIn3 byte0
 * 10 analogIn3 byte1
 * 11 analogIn4 byte0
 * 12 analogIn4 byte1
 * 13 analogIn5 byte0
 * 14 analogIn5 byte1
 */

/*
 * CAUTION!! Be careful with the Serial Monitor, it could freeze 
 * your computer with this firmware running!  It outputs data 
 * without a delay() so its very fast.
 */

#define TOTAL_DIGITAL_PINS 14

// for comparing along with INPUT and OUTPUT
#define PWM 2

// this flag says the next serial input will be PWM data
byte waitForPWMData = 0;

// this flag says the first data byte for the digital outs is next
boolean firstInputByte = false;

/* this int serves as an array of bits to store pin status
 * 0 = INPUT, 1 = OUTPUT
 */
int digitalPinStatus;

/* this byte stores the status off whether PWM is on or not
 * bit 9 = PWM0, bit 10 = PWM1, bit 11 = PWM2
 * the rest of the bits are unused and should remain 0
 */
int pwmStatus;

boolean digitalInputsEnabled = true;
byte analogInputsEnabled = 6;

byte analogPin;
int analogData;

// -------------------------------------------------------------------------
void transmitDigitalInput(byte startPin) {
  byte i;
  byte digitalPin;
  byte digitalPinBit;
  byte transmitByte;
  byte digitalData;

  for(i=0;i<7;++i) {
    digitalPin = i+startPin;
    digitalPinBit = OUTPUT << digitalPin;
    // only read the pin if its set to input
    if(digitalPinStatus & digitalPinBit) {
      digitalData = 0; // pin set to OUTPUT, don't read
    }
    else if( (digitalPin >= 9) && (pwmStatus & (1 << digitalPin)) ) {
      digitalData = 0; // pin set to PWM, don't read
    }
    else {
      digitalData = digitalRead(digitalPin);
    }
/* the next line probably needs to be re-thought (i.e. it might not work...) since my 
   first attempt was miserably wrong.  <hans@at.or.at> */
    transmitByte = transmitByte + ((2^i)*digitalData);
  }
  printByte(transmitByte);
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
}

// -------------------------------------------------------------------------
/* this function checks to see if there is data waiting on the serial port 
 * then processes all of the stored data
 */
void checkForInput() {
  if(serialAvailable()) {  
    while(serialAvailable()) {
      processInput( (byte)serialRead() );
    }
  }
}

// -------------------------------------------------------------------------
void processInput(byte inputData) {
  int i;
  int mask;

  // the PWM commands (230-232) have a byte of data following the command
  if (waitForPWMData > 0) {  
    //    printByte(150);
    //    printByte(inputData);
    analogWrite(waitForPWMData,inputData);
    waitForPWMData = 0;
  }
  else if(inputData < 128) {
    //    printByte(151);
    if(firstInputByte) {
      //      printByte(160);
      for(i=0; i<7; ++i) {
        mask = 1 << i;
        //printByte(254);
        //printByte(i);
        //printByte(mask);
        if(digitalPinStatus & mask) {
          digitalWrite(i, inputData & mask);
          //printByte(inputData & mask);
        } 
      }
      firstInputByte = false;
    }
    else {
      //      printByte(161);
      // output data for pins 7-13
      for(i=7; i<TOTAL_DIGITAL_PINS; ++i) {
        mask = 1 << i;
        //printByte(254);
        //printByte(i);
        //printByte(mask);
        if( (digitalPinStatus & mask) && !(pwmStatus & mask) ) {
          // inputData is a byte and mask is an int, so align the high part of mask
          digitalWrite(i, inputData & (mask >> 7));
          //printByte(inputData & (mask >> 7));
        }        
      }
    }
  }
  else {
    //    printByte(152);      
    //    printByte(inputData);
    switch (inputData) {
    case 200:
    case 201:
    case 202:
    case 203:
    case 204:
    case 205:
    case 206:
    case 207:
    case 208:
    case 209:
    case 210:
    case 211:
    case 212:
    case 213:
      setPinMode(inputData-200,INPUT);
      break;
    case 214:
    case 215:
    case 216:
    case 217:
    case 218:
    case 219:
    case 220:
    case 221:
    case 222:
    case 223:
    case 224:
    case 225:
    case 226:
    case 227:
      setPinMode(inputData-214,OUTPUT);
      break;
    case 230:
    case 231:
    case 232:
      waitForPWMData = inputData - 221; // set waitForPWMData to the PWM pin number
      setPinMode(waitForPWMData, PWM);
      break;
    case 238: // all digital inputs off
      digitalInputsEnabled = false;
      break;
    case 239: // all digital inputs on
      digitalInputsEnabled = true;
      break;
    case 240: // analog input off
    case 241: // analog 0 on  
    case 242: // analog 0,1 on  
    case 243: // analog 0-2 on  
    case 244: // analog 0-3 on  
    case 245: // analog 0-4 on  
    case 246: // analog 0-5 on  
      analogInputsEnabled = inputData - 240;
      break;      
    case 255: // incoming digital output bytes
      firstInputByte = true;
      break;
    }
  }
}

// =========================================================================

// -------------------------------------------------------------------------
void setup() {
  byte i;

  beginSerial(9600);
  for(i=0; i<TOTAL_DIGITAL_PINS; ++i) {
    setPinMode(i,INPUT);
  }
}

// -------------------------------------------------------------------------
void loop() {
  checkForInput();  

  // read all digital pins, in enabled
  if(digitalInputsEnabled) {
    transmitDigitalInput(0);
    checkForInput();
    transmitDigitalInput(7);
    checkForInput();
  }
  else if(analogInputsEnabled) {
    // filler bytes, since the first thing sent is always the digitalInputs
    printByte(0);
    printByte(0);
    checkForInput();  
  }

  /* get analog in, for the number enabled
   */
  for(analogPin=0; analogPin<analogInputsEnabled; ++analogPin) {
    analogData = analogRead(analogPin);
    // these two bytes get converted back into the whole number in Pd
	 // the higher bits should be zeroed so that the 8th bit doesn't get set
    printByte(analogData >> 7);  // bitshift the big stuff into the output byte
    printByte(analogData % 128);  // mod by 32 for the small byte
    checkForInput();
  }

  /* end with the cycle marker, if any of the inputs are enabled */
  if( digitalInputsEnabled || analogInputsEnabled) {
    printByte(255); 
  }
}
