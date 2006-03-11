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
    
int analogPin = 0;
int digitalPin = 0;
int ledOut = 1;
int analogIn = 0;
void setup() {
  beginSerial(9600);
}

void loop() {
  /*
   * get two digital ins
   */
/*   
  printByte(digitalRead(digitalPin));
  digitalPin = digitalPin + 1;
  if (digitalPin > 11)
    digitalPin = 0;
  printByte(digitalRead(digitalPin));
  digitalPin = digitalPin + 1;
  if (digitalPin > 11)
    digitalPin = 0;
*/
  /*
   * get analog in
   */
  printByte(65);   
  analogIn = analogRead(analogPin);
  printByte(analogIn / 127);
  printByte(analogIn % 127);
  analogPin = analogPin + 1;
  if (analogPin > 5)
  {
    analogPin = 0;
    // toggle the status LED (pin 13)
    digitalWrite(13,ledOut);
    ledOut = !ledOut;  // alternate the LED
    // the newline (ASCII 10) marks the start/end of the sequence
    printNewline();   
  }  
}
