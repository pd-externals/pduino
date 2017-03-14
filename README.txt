pduino - interface Pd with the world easily
-------------------------------------------

[arduino] connects Pd to a real Arduino board and lets you control its pins,
thus enabling Pd to interact with the physical world. You don't even need any
C++ skills, you just need to load the Firmata firmware onto your board. 
[arduino] is a Pd implementation of the Firmata protocol as documented on:

  https://github.com/firmata/protocol/blob/master/protocol.md

[arduino-gui] is graphical faksimile of the most common board layout used
by Leonardo, Uno, Duemilanove, Diecimila, NG (and probably others). Any
user interaction with its pins is translated to direct commands to the board.
Similarly, pin states and state changes are represented graphically. It
should get you started quickly without having to look up all available
commands first. 


Installation:
  
  * this pduino package is supposed to live somewhere in Pd's search path
  
  * You may want to [declare -stdpath pduino] before creating [arduino] or
    [arduino-gui] in Pd.


Dependencies:

  * comport
  

Prerequisites:

  * Any model of an Arduino (or compatible) board with a serial interface.
  
  * Some variant of the Firmata (version >= 2.2) firmware on it.
    Most common is StandardFirmata 


How to load the Firmata firmware onto the board:

  * Get the Arduino IDE from: 
    https://www.arduino.cc/en/Main/Software

  * Start the IDE

  * Connect your Board

  * Load the Firmware: 'File' -> 'Examples' -> 'Firmata' -> 'StandardFirmata'
    (You may use another Firmata, if you know what you're doing)

  * Make sure you have the correct port ('Tools' -> 'Port') and the correct
    board ('Tools' -> 'Board') selected

  * Upload Firmata by clicking the 'Upload' button (right arrow)


HAVE FUN!


