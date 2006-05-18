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
 * 150 - next byte sets PWM0 value
 * 151 - next byte sets PWM1 value
 * 152 - next byte sets PWM2 value
 * 200-213 - set digital pin 0-13 to input
 * 220-233 - set digital pin 0-13 to output
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


byte i;

// this int serves as an array of bits to store pin status
int digitalPinStatus;

byte analogPin;
int analogData;

void outputDigital(byte startPin) {
  byte digitalPin;
  byte digitalPinBit;
  byte digitalOutputByte;
  byte digitalData;
  for(i=0;i<7;++i)
  {
    digitalPin = i+startPin;
    digitalPinBit = 2^digitalPin;
    // only read the pin if its set to input
    if(digitalPinStatus & digitalPinBit) 
      digitalData = digitalRead(digitalPin);
    else
      digitalData = 0;
    digitalOutputByte = digitalOutputByte + (2^(i+1-startPin)*digitalData);
  }
  printByte(digitalOutputByte);
}

void setup() {
  beginSerial(9600);
}

void loop() {
  // read all digital pins
  outputDigital(0);
  outputDigital(7);
  /*
   * get analog in
   */
  analogData = analogRead(analogPin);
  // these two bytes get converted back into the whole number in Pd
  printByte(analogData / 32);  // div by 32 for the big byte
  printByte(analogData % 32);  // mod by 32 for the small byte
  ++analogPin; 
  if (analogPin > 5) analogPin = 0;   
  /* end with the cycle marker */
  printByte(255);   
}
