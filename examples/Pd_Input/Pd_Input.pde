/* Pd_Input
 * ------------------
 *
 * This program reads all of the Analog and 12 Digital Inputs
 * and sends the data to the computer as fast as possible.  
 * It reads one analog pin and two digital pins per cycle 
 * so that the time between reads on all pins remains 
 * constant.
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
 * CAUTION!! Do not turn on the Serial Monitor, it could freeze 
 * your computer with this firmware running!  It outputs data 
 * without a delay() so its very fast.
 */
    
int i;
int analogPin = 0;
int digitalPin = 0;
int ledOut = 1;
int analogData = 0;
int digitalData = 0;
int digitalOutputByte = 0;
void setup() {
  beginSerial(9600);
}

void loop() {
  /*
   * get two digital ins
   */ 
  digitalOutputByte = 0;
  for(i=0;i<7;++i)
  {
    digitalData = digitalRead(i);
    digitalOutputByte = digitalOutputByte + (2^(i+1)*digitalData);
  }
  printByte(digitalOutputByte);
  digitalOutputByte = 0;
  for(i=7;i<13;++i)
  {
    digitalData = digitalRead(i);
    digitalOutputByte = digitalOutputByte + (2^(i-6)*digitalData);
  }
  printByte(digitalOutputByte);
  /*
   * get analog in
   */
  analogData = analogRead(analogPin);
  // these two bytes get converted back into the whole number in Pd
  printByte(analogData / 32);  // div by 32 for the big byte
  printByte(analogData % 32);  // mod by 32 for the small byte
  analogPin = analogPin + 1;
  if (analogPin > 5)
  {
    analogPin = 0;
    // toggle the status LED (pin 13)
    digitalWrite(13,ledOut);
    ledOut = !ledOut;  // alternate the LED
    // the newline (ASCII 10) marks the start/end of the sequence
    printByte(255);   
  }  
}
