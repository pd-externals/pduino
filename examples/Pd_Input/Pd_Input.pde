/* Pd_Input
 * ------------------
 *
 * This program reads all of the Analog inputs
 * and sends the data to the computer as fast as
 * possible.  It was designed to work with the Pd
 * patch of the same name in:
 * Help -> Browser -> examples -> hardware
 *
 * (cleft) 2006 Hans-Christoph Steiner
 * @author: Hans-Christoph Steiner
 * @date: 2006-03-04
 * @location: Polytechnic University, Brooklyn, New York, USA
 */

/*
 * CAUTION!! Do not turn on the Serial Monitor, it could freeze 
 * your computer with this firmware running!  It outputs data 
 * without a delay() so its very fast.
 */
    
int potPin = 0;
void setup() {
  beginSerial(19200);
}

void loop() {
  printByte(potPin + 65); // add 65 to get ASCII chars starting with A
  printInteger(analogRead(potPin));
  potPin = potPin + 1;
  if (potPin > 5)
    potPin = 0;
}
