# mini-task-1

## Project-1:
### [XY Laser](https://diyodemag.com/projects/xy_lasyer_arduino_pan_tilt_servo_laser_pointer_project)
  
A servo-controlled tilt bracket with an Arduino Uno and joystick module.The working principal of this project is to enable an object to rotate 180 degree in both x and y directions. We mount our laser pointer to the top and simple joystick to control the bracket’s direction

In place of laser we can mount a camera and use for surveillance

#### Components used

1 Arduino Uno,
2 Servo	motor,
1 Pan and Tilt Bracket,
1 joystick module,

#### How it works
The joystick is essentially a combination of two 10KΩ potentiometers and a tactile switch. When you move the joysticks thumbstick, this alters the wiper on the two potentiometers. The changing resistance on either side of each potentiometer creates a voltage divider for both the X and Y plane. By reading the voltage at the wiper, we can get an approximation on where the wiper is.
Servo motors are used for the rotating the bracket.Servos operate dependent on the duration of a received pulse with a 50Hz frequency.

#### Implementation
Joystick is connected to analog pins of Arduino and servo motors are connected to digital pins.Value on the potentiometer on the Joystick is between 0 and 1023, which we can then map to the range the servo library expects. i.e. 0 – 180.Tactile switch is used to detect if the button has been pressed.


## Project-2:
### [Camera Trap](https://diyodemag.com/projects/camera_trap)

 
