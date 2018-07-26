# SelfDrivingCar
This project using an Arduino Uno aims to realize a car able both to drive both automatically and sending commands from a PC connected with the car through two Xbee devices.
The code doesn't use the Arduino Libraries but it directly writes and reads from the ATMega328, the Arduino internal microcontroller, registers. 
In addition it sends the value of LDR sensor, and eventually turns two LEDs on, and it also sends the RSSI power value among the two Xbees. RSSI is computing following this formula: **-(10 * у) * log(d) - A**, where **y** is the propagation factor, **d** is the distance between the nodes and **A** is the power at 1m.

## Electronic circuit
<a href="url"><img src="https://github.com/marcozecchini/SelfDrivingCar/blob/master/Images/Circuit.jpg"></a>
