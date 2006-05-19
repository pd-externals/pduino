/* Pd_firmware
 * ------------------
 *
 *
 * It was designed to work with the Pd patch of the same 
 * name in:  Help -> Browser -> examples -> hardware
 *
 * (cleft) 2006 Hans-Christoph Steiner
 * @author: Hans-Christoph Steiner
 * @date: 2006-03-10
 * @location: Polytechnic University, Brooklyn, New York, USA
 */

/*
 * Pduino protocol
 * =============== 
 * data: 0-127   control: 128-255
 * 
 * Pd->Arduino commands
 * --------------------
 * 200-213 - set digital pin 0-13 to output
 * 214-227 - set digital pin 0-13 to input
 * 230 - next byte sets PWM0 value
 * 231 - next byte sets PWM1 value
 * 232 - next byte sets PWM2 value
 * 
 *
 * Pd->Arduino byte cycle
 * ----------------------
 * 0  cycle marker (255/11111111)
 * 1  digitalOut 0-6 bitmask
 * 2  digitalOut 7-13 bitmask
 * 
 * Arduino->Pd byte cycle
 * ----------------------
 * 0  digitalIn 0-6 bitmask
 * 1  digitalIn 7-13 bitmask
 * 2  analogIn0 byte0
 * 3  analogIn0 byte1
 * 4  analogIn1 byte0
 * 5  analogIn1 byte1
 * 6  analogIn2 byte0
 * 7  analogIn2 byte1
 * 8  analogIn3 byte0
 * 9 analogIn3 byte1
 * 10 analogIn4 byte0
 * 11 analogIn4 byte1
 * 12 analogIn5 byte0
 * 13 analogIn5 byte1
 * 14 cycle marker (255/11111111)
 * 
 * 
 *      TX         RX
 * -----------------------
 *   					 
 */

/*
 * CAUTION!! Be careful with the Serial Monitor, it could freeze 
 * your computer with this firmware running!  It outputs data 
 * without a delay() so its very fast.
 */

#define TOTAL_DIGITAL_PINS 14

byte i;

// this flag says that the next serial input will be data, not control
boolean waitForData = 0;

/* this int serves as an array of bits to store pin status
 * 0 = INPUT, 1 = OUTPUT
 */
int digitalPinStatus;

byte analogPin;
int analogData;

// -------------------------------------------------------------------------
void outputDigital(byte startPin) {
  byte digitalPin;
  byte digitalPinBit;
  byte digitalOutputByte;
  byte digitalData;
  for(i=0;i<7;++i)
  {
    digitalPin = i+startPin;
    digitalPinBit = OUTPUT << digitalPin;
    // only read the pin if its set to input
    if(digitalPinStatus & digitalPinBit) {
      digitalData = 0; // pin set to OUTPUT, don't read
    }
    else {
      digitalData = digitalRead(digitalPin);
    }
    digitalOutputByte = digitalOutputByte + (2^(i+1-startPin)*digitalData);
  }
  printByte(digitalOutputByte);
}

// -------------------------------------------------------------------------
void setPinMode(int pin, int mode) {
  pinMode(pin,mode);
  if(mode == INPUT) {
      digitalPinStatus = digitalPinStatus &~ (1 << pin);
  }
  if(mode == OUTPUT) {
      digitalPinStatus = digitalPinStatus | (1 << pin);
  }
}

// -------------------------------------------------------------------------
void checkForInput() {
  if(serialAvailable()) {  
    while(serialAvailable()) {
      processInput(serialRead());
    }
  }
}

// -------------------------------------------------------------------------
void processInput(int inputData) {
  if (waitForData > 0) {  
    // the PWM commands (230-232) have a byte of data following the command
    analogWrite(waitForData,inputData);
    waitForData = 0;
  }
  else {
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
        setPinMode(inputData-200,OUTPUT);
        printByte(inputData);
        printByte(inputData);
        printByte(inputData);
        printByte(inputData);
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
        setPinMode(inputData-214,INPUT);
        printByte(inputData);
        printByte(inputData);
        printByte(inputData);
        printByte(inputData);
        break;
      case 230:
      case 231:
      case 232:
        // set waitForData to the PWM pin number
        waitForData = inputData - 221;
        printByte(inputData);
        printByte(inputData);
        printByte(inputData);
        printByte(inputData);
        break;
      case 255:
        break;
    }
  }
}

// =========================================================================

// -------------------------------------------------------------------------
void setup() {
  beginSerial(9600);
  for(i=0; i<TOTAL_DIGITAL_PINS; ++i) {
      setPinMode(i,INPUT);
  }
}

// -------------------------------------------------------------------------
void loop() {
  // read all digital pins
  outputDigital(0);
  outputDigital(7);
  /*
   * get analog in
   */
/*   
  for(analogPin=0; analogPin<5; ++analogPin)
  {
  analogData = analogRead(analogPin);
  // these two bytes get converted back into the whole number in Pd
  printByte(analogData >> 7);  // bitshift the big stuff into the output byte
  printByte(analogData % 128);  // mod by 32 for the small byte
  }
*/  
//  ++analogPin; 
//  if (analogPin > 5) analogPin = 0;   
  /* end with the cycle marker */
  // bitshift the big stuff into the output byte
  printByte(digitalPinStatus >> 7);
  // clear the 8th bit before truncating to a byte for small byte
  printByte(digitalPinStatus % 128); 

  checkForInput();
  
  printByte(255);   
}
