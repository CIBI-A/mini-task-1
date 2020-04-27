# mini-task-1

# Project-1:
#### XY Laser
  
A servo-controlled tilt bracket with an Arduino Uno and joystick module.The working principal of this project is to enable an object to rotate 180 degree in both x and y directions. We mount our laser pointer to the top and simple joystick to control the bracket’s direction

##### Components used

1 Arduino Uno

1 Servo Shield	

2 Servo	motor

1 Pan and Tilt Bracket

1 joystick module

##### How it works
The joystick is essentially a combination of two 10KΩ potentiometers and a tactile switch. When you move the joysticks thumbstick, this alters the wiper on the two potentiometers. The changing resistance on either side of each potentiometer creates a voltage divider for both the X and Y plane. By reading the voltage at the wiper, we can get an approximation on where the wiper is.
Servo motors are used for the rotating the bracket and it is directly connected to Shield.

joystick is connected to servo motors
