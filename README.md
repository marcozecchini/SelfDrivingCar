# SelfDrivingCar
This project using an Arduino Uno aims to realize a car able both to drive both automatically and sending commands from a PC, using a Processing GUI, connected with the car through two Xbee devices. 

----------------------------------------------------------------------------------------------------

## Hardware

The hardware involed in this project is composed by the following components:
* Arduino Uno board
* 2x Xbee S1
* 2x HC-SR04 Ultrasonic Sensor
* LDr sensor
* 2x server motor DC
* H-bridge (SN754410)
* Led 
* Resistors
* Wires

The following picture shows the connections that are made to ensure the process works.

![foto](https://github.com/marcozecchini/SelfDrivingCar/blob/master/Images/Circuit.png)


----------------------------------------------------------------------------------------------------

## Code description

Driving in "automatic" mode the car within certain distances, measured with two ultrasonic sensors HC-SR04,  goes straight either it goes closer an obstacle/wall on the left, if it is too far, or it goes farther if it is too close to it or if it has an obstacle in front it can avoid it. Driving in "no-automatic" version it is possible to send commands from the pc to go straight, turn left or right, stop the vehicle and enable/disable the reverse march.

Every few milleseconds the car sends a set of values regarding the distance sensors measurements and a light sensor (LDR) measurement through the Xbee device installed on the top of Arduino. This information is received by another Xbee that is connected to the PC and that is shown into the Processing GUI.

The code is written enrusing two versions:
* The first not using the Arduino Libraries but it directly writes and reads directly using AVR commands.
* The second one that uses Arduino Libraries, that makes the code more readable.
In order to do so 
```

```
## Activity diagram
