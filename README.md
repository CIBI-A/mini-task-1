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

## Project-4:
### [IOT based Irrigation system](https://circuitdigest.com/microcontroller-projects/iot-based-smart-irrigation-system-using-esp8266-and-soil-moisture-sensor)

#### Components used

NodeMCU ESP8266,
Soil Moisture Sensor Module,
Water Pump Module,
Relay Module,
DHT11,

#### How it works

The **DHT11** is a digital temperature and humidity sensor. It uses a capacitive humidity sensor and a thermistor to measure the surrounding air, and spits out a digital signal on the data pin. **Relay** is a switch which controls (open and close) circuits electromechanically. The main operation of this device is to make or break contact with the help of a signal without any human involvement in order to switch it ON or OFF.The **Soil Moisture** Sensor uses capacitance to measure dielectric permittivity of the surrounding medium. In soil, dielectric permittivity is a function of the water content. The sensor creates a voltage proportional to the dielectric permittivity

#### Implementation

Moisture sensor is connected to A0 pin, water pump is connected to relay which is connected D0 pin and DHT11 sensor is connected to D3 pin of NodeMCU.

DHT11 sensor library is used to program ESP8266. In this project ThingSpeak Server is used, its API key is taken to communicate wth its server. Wi-Fi credentials and sensor pins are defined.In this project millis() is used instead of delay(). delay() will haut the function so we can't read sensor values during this period. This problem will not occur when we use millis().Connect to Wi-Fi with the given credentials.Initially turn off the motor and start the DHT11 sensor reading temperature and humidity data and save them into variables.Read the moisture reading from sensor and save the reading.
If the moisture reading is in between the required soil moisture range then keep the pump off or if it goes beyond the required moisture then turn the pump ON.Now after every 10 seconds call the sendThingspeak() function to send the moisture, temperature and humidity data is sent to ThingSpeak server.


## Project-5:
### [Optical Character Recognition](https://circuitdigest.com/microcontroller-projects/optical-character-recognition-ocr-using-tesseract-on-raspberry-pi)

In this project we will make RPi to read the data present in a image using Tesseract. 

This project can be further improved by image processing and use for face detection and security purposes.  
#### Components used
RPi

#### How it works

Tesseract 4.0 uses a Deep Learning model to recognize characters and even handwritings. Tesseract 4.0 uses Long Short-Term Memory (LSTM) and Recurrent Neural Network (RNN) to improve the accuracy of its OCR engine.OpenCV is used to remove noise from the program and then configure the Tesseract OCR engine based on the image to get better results

#### Implementation 
 
OpenCV library is installed in RPi.Installing Tesseract OCR engine in RPi to perform Optical Character Recognition. Then install PyTesseract which helps to use Tesseract in python. Pillow should be installed initially.Pytesseract allows us to configure the Tesseract OCR engine by setting the flags which changes the way in which the image is searched for characters. The three main flags used in a Tesseract OCR that is language (-l), OCR Engine Mode (--oem) and Page Segmentation Mode (- -psm). language flag is used to set language, is it possible to recognise two or more language in same image. OCR engine has four modes, in this project we use neural networks. Page segmentation mode flag is very important it can used to find details from image that has so much background details along with the characters or the characters are written in different orientation or size.It has 14 different mode, in this project 'sparse text with OSD' mode is selected. OpenCV can be combined with tesseract to get better results. 

## Project-6:
### [Blind Stick Navigator](https://www.hackster.io/mero/blind-stick-navigator-b119f5)

A blind assist tool that provide obstacles notification and GPS location to the guardian / authority via SMS.

#### Components used

1Sheeld, Arduino UNO, Relay, Toggle switch, 2 Ultrasonic sensor, DC  motor  
#### How it works
Ultrasonic sensors emit short, high-frequency sound pulses at regular intervals. These propagate in the air at the velocity of sound. If they strike an object, then they are reflected back as echo signals to the sensor, which itself computes the distance to the target based on the time-span between emitting the signal and receiving the echo.

1Sheeld is a new easily configured shield for Arduino. It is connected to a mobile app that allow the usage of all of Android smartphones' capabilities such as LCD Screen, Gyroscope, Switches, LEDs, Accelerometer, Magnetometer, GSM, Wi-Fi, GPS …etc. into your Arduino sketch. 

#### Implementation

2 Ultrasonic sensor(input) are used one at ground level and other at head level.DC motor(output) is connected to Arduino through relay. Whenever ultasonic sensor detects object within some specific distance then motor starts to vibrate to warn blind people. Toggle switch is used, whenever blind people feels that they have lost route or feel like stuck somewhere if they switch on the toggle it will send 'GPS location' to guardian via sms. This can be implented using 1Sheeld.   

## Project-7:
### [Wireless Doorbell](https://circuitdigest.com/microcontroller-projects/wireless-doorbell-using-arduino)

In this project, we are going to build a Wireless Doorbell using Arduino. We will have a button which when pressed will wirelessly play a melody of our choice to indicate someone is at the door. For wireless connectivity, we will use the 433 MHz RF module.
#### Components used
RF module,
Arduino,
Buzzer,
Push-button,
Breadboard,
Connecting wires,
#### How it works

A RF transceiver module will always work in a pair that is it needs a Transmitter and Receiver to send and Send data. A transmitter can only send information and a Receiver and can only receive it, so data can always be sent from one end to another and not the other way around.
A transmitter consists of a SAW resonator, which is tuned to 433MHz frequency, a switching circuit, and a few passive components.When the input to the data pin is HIGH, the switch will act as a short circuit and the oscillator runs which produces a fixed amplitude carrier wave and a fixed frequency for some period of time.
An RF receiver is a simple circuit that consists of an RF tuned circuit, an amplifier circuit, and a phase lock loop circuit.With all these components, the receiver receives the signal from the antenna which is  then tuned by RF tuned circuit and this weak signal is amplified using OP-Amp, and this amplified signal is further used as input to PLL, which makes the decoder to lock onto the incoming digital bits which gives an output which is less in noise.

ASK(Amplitude Shift Keying) Modulation is used in transmitter 

#### Implementation
The Arduino pin 5 is connected to the one end of the doorbell switch, and the other end of the switch is connected to the supply voltage. A pull-down resistor of 10kohm is connected to pin 5 as shown in the fig. Pin 11 is connected to the data pin of the transmitter module. Vcc is connected to the supply voltage, and the ground pin of the transmitter module is grounded.In receiver side, we connect pin 7 of the Arduino to the buzzer positive terminal, and the negative terminal is grounded. A supply voltage of VCC is given to the receiver module, and the GND pin of the module is connected to the ground. The out pin of the receiver module is connected to the 12th pin of the Arduino.


## Project-8:
### [Smart Lamp](https://www.instructables.com/id/IRIS-the-Lamp-That-Knows-When-Yourre-Around/)

Automatic detection of the user to turn on / off the light
Automatically lights up your path when you want to go somewhere in the middle of the night
Automatically adjusts brightness based on surrounding light
Can turn on smart mode automatically after sunset or when no other light source is around.
#### Components used
Arduino Uno,
2 Warm white LED strip,
Single channel Relay,
Proximity sensor,
LDR sensor
#### How it works
An LDR is a component that has a (variable) resistance that changes with the light intensity that falls upon it.A proximity sensor often emits an electromagnetic field or a beam of electromagnetic radiation (infrared, for instance), and looks for changes in the field or return signal.
#### Implementation

A proximity sensor is placed under the table and connected to Arduino as input. Bulb(output) is connected to Arduino through relay. Bulb is made of two white LED strip stuck on aluminum.To adjust brightness LDR is used and the lamp should now be powered directly from the Arduino for which another relay can be used. You can use PWM to adjust the brightness of the lamp based on the analog input from LDR.

## Project-9:
### [XY Laser](https://diyodemag.com/projects/xy_lasyer_arduino_pan_tilt_servo_laser_pointer_project)
#### Components used
#### How it works
#### Implementation

## Project-10:
### [XY Laser](https://diyodemag.com/projects/xy_lasyer_arduino_pan_tilt_servo_laser_pointer_project)
#### Components used
#### How it works
#### Implementation

## Project-11:
### [XY Laser](https://diyodemag.com/projects/xy_lasyer_arduino_pan_tilt_servo_laser_pointer_project)
#### Components used
#### How it works
#### Implementation

## Project-12:
### [XY Laser](https://diyodemag.com/projects/xy_lasyer_arduino_pan_tilt_servo_laser_pointer_project)
#### Components used
#### How it works
#### Implementation

## Project-13:
### [XY Laser](https://diyodemag.com/projects/xy_lasyer_arduino_pan_tilt_servo_laser_pointer_project)
#### Components used
#### How it works
#### Implementation

## Project-14:
### [XY Laser](https://diyodemag.com/projects/xy_lasyer_arduino_pan_tilt_servo_laser_pointer_project)
#### Components used
#### How it works
#### Implementation

## Project-15:
### [XY Laser](https://diyodemag.com/projects/xy_lasyer_arduino_pan_tilt_servo_laser_pointer_project)
#### Components used
#### How it works
#### Implementation

## Project-16:
### [XY Laser](https://diyodemag.com/projects/xy_lasyer_arduino_pan_tilt_servo_laser_pointer_project)
#### Components used
#### How it works
#### Implementation

## Project-17:
### [XY Laser](https://diyodemag.com/projects/xy_lasyer_arduino_pan_tilt_servo_laser_pointer_project)
#### Components used
#### How it works
#### Implementation

## Project-18:
### [XY Laser](https://diyodemag.com/projects/xy_lasyer_arduino_pan_tilt_servo_laser_pointer_project)
#### Components used
#### How it works
#### Implementation

## Project-19:
### [XY Laser](https://diyodemag.com/projects/xy_lasyer_arduino_pan_tilt_servo_laser_pointer_project)
#### Components used
#### How it works
#### Implementation

## Project-20:
### [XY Laser](https://diyodemag.com/projects/xy_lasyer_arduino_pan_tilt_servo_laser_pointer_project)
#### Components used
#### How it works
#### Implementation

## Project-21:
### [XY Laser](https://diyodemag.com/projects/xy_lasyer_arduino_pan_tilt_servo_laser_pointer_project)
#### Components used
#### How it works
#### Implementation

## Project-22:
### [XY Laser](https://diyodemag.com/projects/xy_lasyer_arduino_pan_tilt_servo_laser_pointer_project)
#### Components used
#### How it works
#### Implementation

## Project-23:
### [XY Laser](https://diyodemag.com/projects/xy_lasyer_arduino_pan_tilt_servo_laser_pointer_project)
#### Components used
#### How it works
#### Implementation

## Project-24:
### [XY Laser](https://diyodemag.com/projects/xy_lasyer_arduino_pan_tilt_servo_laser_pointer_project)
#### Components used
#### How it works
#### Implementation

## Project-25:
### [XY Laser](https://diyodemag.com/projects/xy_lasyer_arduino_pan_tilt_servo_laser_pointer_project)
#### Components used
#### How it works
#### Implementation
