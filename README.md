# mini-task-1

#### Components used
#### How it works
#### Implementation

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
Joystick is connected to analog pins(input pins) of Arduino and servo motors are connected to digital pins(output pins).Value on the potentiometer on the Joystick is between 0 and 1023, which we can then map to the range the servo library expects. i.e. 0 – 180.Tactile switch is used to detect if the button has been pressed.


## Project-2:
### [Camera Trap](https://diyodemag.com/projects/camera_trap)

Camera trap is essentially an autonomous DSLR camera with a sensor built in to detect the subject. When the sensor detects a subject, the camera takes a photo

#### Components used

1 Arduino Nano, 
1 PIR Sensor,
2 Optocouplers,	
1 2.5mm Stereo Plug,

#### How it works

PIR sensor generates voltage when it detects change in the heat. 2.5 mm audio plug is used in which the tip is shutter, ring is focus and sleeve in camera ground.The tip and ring connection on the plug shown here are held to 3.3V via a pullup resistor. The camera has a physical switch with two positions; the first position allows the camera to automatically focus (AF) the shot based on your AF settings, and the second position signals the camera to actuate the shutter mechanism. Optocouplers are used to isolate out circuit from camera's internal circuit because cameras are very expensive. Resistors are chosen from the conditions given in datasheet in this project 100kohm is used.  
#### Implementation

PIR sensor is connected to input pin and optocouplers pin 1 is connnected output pins. Pin 5 of optocouplers are connected to focus and shutter and pin 4 is connected to sleeve. In code **focus time** (number of milliseconds between triggering the focus function and then taking the picture),**min time** (delay after triggering and is used to prevent the same trigger event causing multiple unwanted camera triggers),**shots** (number of photos that will be taken after a trigger event),**pause** (used to set the time between shots and gives the camera sufficient time to write to the memory card, and to detect that the shutter pin has changed states), these value should be chosen as per camera. When PIR sensor detects high ,program pulls the focus to camera ground and wait till focus time ,then pulls the shutter and pauses and this repeats whenever the PIR sense.

## Project-3:
### [RPi based Jarvis themed speaking Alarm clock](https://circuitdigest.com/microcontroller-projects/raspberry-pi-based-jarvis-themed-speaking-alarm-clock)

As name suggects this is alarm clock which has a voice. Whenever alarm goes it tells the time, day and some predefined text 

#### Components used

Raspberry Pi,
3.5” TFT LCD Screen,
Speaker,
AUX cable,
Internet Connection,

#### Implementation

LCD is interfaced with RPi and speaker is connected to RPi through AUX cable.

RPi is flashed with Rasbian Jessie and connected to the internet.LCD is interfaced with RPi to display time and date. Text To speech (TTS) Engine is to be used to make Pi speak. In this project Espeak Engine is installed. GUI is developed using python so the user can to view time and date which will be displayed in LCD. GUI is designed using Qt designer software. In Python code the necessary libraries(PyQt4,espeak and strftime) are imported, Qt designer makes the code simply by converting the GUI into necassary python code and editing a little bit of code to assign the purpose of widget used is needed to be done and the text is added which will be delivered as speech by speaker when the alarm triggers.

Make sure Espeak and PyQt4 is installed in Pi and run the python program.
 
