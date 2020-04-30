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

A proximity sensor is placed under the table and connected to Arduino as input.So whenever it detects an object nearby Bulb will glow for 15 seconds and after 15 seconds if the sensor is high bulb will glow orelse it will switch off. Bulb(output) is connected to Arduino through relay. Bulb is made of two white LED strip stuck on aluminum.To adjust brightness LDR is used and the lamp should now be powered directly from the Arduino for which another relay can be used. You can use PWM to adjust the brightness of the lamp based on the analog input from LDR.

## Project-9:
### [Coronavirus Live Updator](https://www.hackster.io/isaichakri/coronavirus-live-updator-7febb3)

This project does two functions one it's a live updator and trigger the buzzer if the no of cases increases 500 people for like 10min.

#### Components used
NodeMCU,
Nokia LCD 5110

#### Implementation

NodeMCU and Nokia LCD are connected(CLK to D4,
DIN to D3,
RESET to D0,
DC to D2,
CE to D1).Network credentials are given to enable internet connection. The neccesary libraries are included to connect to the server, fetch data and display in LCD
.In python code we will add the URL providing the real time data of different countries across the world and display it in the LCD of each country for some specific time.

## Project-10:
### [DTMF Controlled Robot](https://www.electronicshub.org/dtmf-mobile-controlled-robot-without-microcontroller/)
In this project we will design a simple Robotic vehicle that can be controlled using a phone without using microcontroller.
#### Components used
DTMF Tone decoder IC,
Motor driver IC,
Motors

#### How it works
When a key is pressed from our mobile, it generates a tone, which is a combination of two frequencies, one is high frequency and another one is low frequency. This frequency can be decoded by the decoder IC into binary sequence. Using this binary sequence, the robot is controlled.
#### Implementation
Decoder IC is The decoder IC internally, consists of operational amplifier, whose output is given to pre filters to separate low and high frequencies. Then it is passed to code detector circuit and it decodes the incoming tone into 4bits of binary data. This data at the output is directly given to the driver IC to drive the two motors.

## Project-11:
### [Pix-a-Sketch](https://www.hackster.io/gatoninja236/pix-a-sketch-a-virtual-etch-a-sketch-on-an-led-matrix-dd3bae)
Sketching a picture on LED matrix using rotary encoders.

#### Components used
RPi 3,
Adafruit RGB matrix,
Arduino nano,
Rotary encoder with push buttons,
DFRobot 64 x 64 RGB LED Matrix Panel,
DFRobot 6 DOF sensor- MPU6050

#### Implementation
Connect RPi to Wi-fi and install rgb-matrix library and select the Adafruit Matrix HAT.Also install mpu6050 library. GPIO pins are short to connect rotary encoder because the RGB matrix takes up so many GPIO pins so Arduino nano is used as I2C slave device that would be able to count the rotations of each encoder and then send that information to the host device when requested. Upon each request for encoder data, the total number of rotations is set back to zero, which gives relative positional data rather than absolute positioning. If the device detects it is being shaken(DOF sensor is used for this), 5 random lit-up pixels are selected and then turned off. To erase the entire matrix and return the cursor to its origin (0, 0), the hardware button can be held down for two seconds.

## Project-12:
### [Non-contact IR Thermometer](https://circuitdigest.com/microcontroller-projects/ir-thermometer-using-arduino-and-ir-temperature-sensor)
#### Components used

Arduino Pro Mini
MLX90614 Infrared Temperature Sensor
OLED Display 
Laser Diode
Push button
#### How it works

The MLX90614 is one such sensor that uses IR energy to detect the temperature of an object.
IR temperature sensors work by focusing the infrared energy emitted by an object onto one or more photodetectors.The emitted infrared energy of any object is proportional to its temperature, the electrical signal provides an accurate reading of the temperature of the object that it is pointed at.The computational unit converts it into temperature value using a 17-bit in-built ADC and outputs the data through I2C communication protocol
#### Implementation

OLED,IR temperature sensor and laser diode are connected with Arduino. Push button is also used, whenever it is pressed sensor sends the data to Arduino and displays the output(temperature of pointed object) in OLED. In this project 9V battery is used so when the push button is pressed the 9V battery is connected to the RAW pin of Arduino which is then regulated to 5V using the on-board voltage regulator.


## Project-13:
### [Gesture Remote-control](https://www.circuito.io/blog/gesture-remote-control/)
This project is used to control the TV using gesture.
#### Components used
SparkFun APDS-9960 - RGB and Gesture Sensor,
Infrared (IR) LED,
IR Receiver Diode - TSOP38238,
Lithium Battery, 
Arduino Pro Mini 328,
220 Ohm Resistor,
Transistor - NPN BC337,


#### Implementation
Gesture sensor is connected to Arduino.IR LED is connected to Arduino through resistor and transistor. Power supply for the Arduino is given by Lithium battery.To use this gesture control remote we need to find which IR signals needed to be transmitted. This is done by connecting IR receiver to Arduino and uploaded the IR Receive Dump code example from the IR remote library.
Opening the serial monitor on the Arduino IDE and clicking on the original TV remote buttons we can determine which IR codes are used for each remote function.And in the code we define functions for different gestures like Waving left or right, lets you swap between different channels
Waving up and down controls the volume


## Project-14:
### [Raspberry Pi TrafficLight](https://www.electronicsforu.com/electronics-projects/python-computer-vision-based-traffic-light-using-raspberry-pi)
Many time it is seen the people are waiting at crossing  to get the traffic clear and the people have to wait and keep standing for an hours to cross the road it become more complicated when any child and old women have to wait for long time to cross the road so today we are going to Raspberry Pi TrafficLight Using TensorFlow & Python that checks how many people are waiting at zebra crossing and from how long they are waiting at zebra crossing and  give priority to people rather than vehicles accordingly.
#### Components used
Raspberry Pi 4,
RPi camera
#### How it works
First a camera streams the live video at zebra crossing 
Then that video is cut in certain frames and a tensorflow with computer vision modules checks the number of people and time from which they are waiting to cross the 
If the system get that the number of people is greater than the 3(can be changed ) waiting at zebra crossing then it give priority to them. If the number of people at zebra crossing is below 3 but waiting for more than 60 seconds (can be changed)  then it give priority to them in crossing.
#### Implementation
RPi is installed with Raspbian os and with Python3 environment on raspberry pi and OpenCV. After the installation you can now proceed with the cloning of TF modules. In python we will add necessary libraries and next we will set the path for tensor flow detection modules and then we will set the name and path for labels. Next part of code will check the camera video and cut it in various frames and after that we have code that will try to detect the objects in each frame and then map with the labels. Now in next part of the code we check the label of object detected and here we have set a substring ie “person” we use this substring to count the number of people in image.

## Project-15:
### [Virtual Assistant](https://www.instructables.com/id/Pi-Home-a-Raspberry-Powered-Virtual-Assistant/)

#### Components used
Raspberry Pi 3,
MicroSD Card, 
A USB Microphone,
Speakers

#### Implementation
RPi is installed with raspbian os.The Pi doesn’t have microphones inbuilt so USB microphone is attached to record audio.And Set external mic as the audio capture device and your inbuilt sound card as the speaker device. Connect the speaker to 3.5mm jack of RPi and configure it.Python environment is created.In python install the Google Assistant SDK package, which contains all the code required to run the Google Assistant on the Pi. And then enable the google assistant cloud project and Authenticating RPi.Then we will run python script and introduce the Google Assistant.

## Project-16: 
### [TDS Meter Using MCU](https://www.electronicsforu.com/electronics-projects/implementing-low-cost-battery-powered-tds-meter-using-mcu-psoc)
One of the direct methods to measure the quality of water is the measurement of TDS in water

#### How it works
The MCU will generate the waveform in digital format which get converted to analog by the DAC and passed to opamp.The conductance to voltage conversion is done by giving conductance probe to the feedback loop of opamp. The output of the Op-amp is passed through a rectifier to calculate the RMS value of the received signal. The RMS value of the received signal is then converted to digital value by the ADC.This can be implemented in PSoc.The PSoC is a combination of MCU and programmable analog and digital blocks

#### Implementation
Sine wave samples stored in a LUT in the flash of the MCU send to the DAC. Conductance to voltage conversion circuit can be implemented using the internal Op-amp of PSoC, an external resistance and the conductance probe.An Op-amp based precision rectifier is required. Again the internal Op-amp of the PSoC can be utilized with external two diodes and three resistors.The output of the rectifier stage is fed to the internal ADC of PSoC. The RMS value of the received signal can be calculated by MCU from the ADC output.To measure the RMS value of the transmitted signal, the output of the DAC can be bypassed to the rectifier input using the internal Analog MUX. The conductance value can be calculated from the RMS values of the transmitted and received signals and can be converted to TDS value in ppm.

## Project-17:
### [Wireless RF Communication](https://circuitdigest.com/microcontroller-projects/wireless-rf-communication-between-arduino-and-raspberry-pi-using-nrf24l01)
We will use nRF24L01 – 2.4GHz RF Transceiver module with Arduino UNO and Raspberry Pi to establish a wireless communication between them.
#### Components used
nRF24L01 RF Module
Rpi,
Arduino
#### How it works
nRF24L01 modules operate on 2.4GHz (ISM band) with baud rate from 250Kbps to 2Mbps and with proper antennas these modules can transmit and receive signals upto a distance of 100 meters between. .They are half-duplex they can either send or receive data at a time. nRF24L01 IC communicates using the SPI protocol
#### Implementation
nRF24L01 is connected with Arduino by SPI interface and LCD is interfaced with I2C protocol. Even RPi uses SPI interface to connect nRF24l01.Python 3 will used to program because it already has the library.Message is sent using Raspberry Pi & nRf24l01 and receiving it using Arduino UNO & nRF24l01. The message will be printed in the 16x2 LCD.



## Project-18:
### [Distributed Sensor Network](https://diyodemag.com/projects/distributed_sensor_network)
#### Components used
#### How it works
#### Implementation

## Project-19: 
### [Raspberry Pi Based COVID -19 Ventilator](https://www.electronicsforu.com/electronics-projects/raspberrypi-based-covid-19-ventilator-and-health-monitoring-device)
Creating a RPi based ventilator to overcome the demand of ventilators. It will also be capable of monitoring our health and provide data about our heartbeat and SPO2 levels. 
#### Components used
RPi,
Arduino,
servo motor,
MAX30100 sensor,
#### How it works
Ventilator use a servo motor that applies pressure on an air sack (BVM bag), thus pushing oxygen-concentrated air into the lungs. When the servo motor comes back to its earlier position, it results in pressure being released from the air sack (BVM bag), making it retain its original shape. This helps to draw out CO2 from the lungs.MAX30100 sensor gives us live data about the rise and fall of pulse rate and oxygen level in the blood of a patient
#### Implementation
servo motor is interfaed with RPi and the speed of motor is adjust in program to match the respiratory rate of patient.MAX30100 and OLED is interfaced with Arduino and code in arduino IDE . To visualize graph we will setup Processing 3 in RPi and connect the Arduino with MAX30100 into the Raspberry Pi USB port. 

## Project-20:  
### [RFID Based Attendance System](https://www.electronicshub.org/rfid-based-attendance-system/)
ATMEGA8 Microcontroller,
RFID Reader,
RFID Tags,
LCD display
#### How it works
RFID or Radio Frequency Identification System is a technology based identification system which helps identifying objects just through the tags attached to them, without requiring any light of sight between the tags and the tag reader. All that is needed is radio communication between the tag and the reader.RFID can be interfaced to microcontroller through USART. Data is transferred from RFID cards to reader and from there to microcontroller.
#### Implementation

The RFID Reader is connected to the transmit and receive pins of the micro controller.When this circuit is powered ON, initially the microcontroller will display the message as Swipe the card on the LCD display. When the RFID reader detects the ID card, it will send the unique card no to the microcontroller via serial terminal.Microcontroller compares the tag with the database. If the tag is matched LCD displays “authenticated” and takes your attendance
Now place another card which is not present in the database and check for authentication.
Now LCD displays “Unauthorised” and it will never take the attendance.This way we can take attendance

## Project-21:
### [Fire Fighting Robot](https://circuitdigest.com/microcontroller-projects/arduino-fire-fighting-robot-code)
A robot using Arduino that could move towards the fire and pump out water around it to put down the fire
#### Components used
Arduino UNO,
Fire sensor or Flame sensor, 
Servo Motor,
L293D motor Driver module,
Mini DC Submersible Pump,
Small Breadboard,
Robot chassis with motors (2) and wheels(2),
A small can
#### How it works
This sensor has an IR Receiver (Photodiode) which is used to detect the fire. When fire burns it emits a small amount of Infra-red light, this light will be received by the IR receiver on the sensor module. Then we use an Op-Amp to check for change in voltage across the IR Receiver, so that if a fire is detected the output  pin (DO) will give 0V(LOW) and if the is no fire the output pin will be 5V(HIGH).
L293D is a Motor Driver IC which allows DC motor to drive on either direction.
#### Implementation
3 fire sensors are placed in different directions so that if fire from any of those directions could be detected. When fire is detected we move the robot to that direction using motor through L293D motor driver module. we use a small container which can carry water, a 5V pump is also placed in the container and the whole container is placed on top of a servo motor so that we can control the direction in which the water has to be sprayed.

## Project-22:
### [Wi-CASSS](https://www.hackster.io/govindanunni07/wifi-controlled-audio-source-selector-switch-wi-casss-d40ed3)
Remote WiFi Controlled Audio Source Selector Switch (Wi-CASSS) which enables the user switch between two connected input/output audio devices over internet.
#### Components used
DPDT Relay,
ESP8266,
1N4007 Diode,
NPN Transistor
#### How it works
Double Pole Double Throw(DPDT) Relay switches the Left & Right Channel of the audio signals into two separate channels. Since relays are just an electromechanical switch, they don't distort the audio signals and most importantly, they are bi-directional.
#### Implementation
Transistor switching circuit is used to switch on/off relay.3.3V regulator is connected to ESP8266.6 pin female header is converted to 2 male 3.5mm audio jack. The ground pin of audio device and power source are isolated so there will be no distortion in audio signal. Output(3.5mm PCB Mountable Female Audio Jack) is connected to speaker. In this ESP is programmed using Arduino nano. Blynk App is used and we can switch the audio device from phone.

## Project-23:
### [NodeMCU WiFi Manager to Scan and Connect to Wi-Fi Networks](https://circuitdigest.com/microcontroller-projects/using-wifi-manager-on-nodemcu-to-scan-and-connect-wifi-networks)

As we know most of the IoT devices have to be connected to the internet to begin operation. While prototyping or testing our IoT Projects we can easily hardcode the Wi-Fi SSID and Password in our program and make it work. But when the device is handed over to the consumer, they should be able to scan and connect to own Wi-Fi Network without changing the program. This is where the ESP8266 Wi-Fi manager will be helpful.In this project, we are going to use NodeMCU, and program it to operate in two different modes, namely Access point (AP) mode, and Station (STA) mode

#### Components used

NodeMCU,
Breadboard,
Pushbutton,
LEDs,
#### Implementation
Circuit connection is very simple just connect pushbutton and LED with resistors to NodeMCU.
LEDs are used to indicate whether it is in STA or AP. Pushbutton is used to switch between modes, if we press while switching on NodeMCU will enter AP mode.We are going to code in such a way that, when powering up the module, if the switch is in ON state, it will set the ESP to Access Point (AP) mode and resets the saved settings. It will stay in this mode until unless the user uses a Wi-Fi-enabled device, and connects it to this access point. When connected to this access point, it will redirect the user to a web page, where the user can configure to new SSID and password. After setting the credentials, the ESP will reboot itself and works as a Station (STA) mode.NodeMCU will remeber this Wi-fi credential and whenever we switch on it will automatically connect to that network. By this we can remove the burden of coding every time whenever you need to connect to a new network.

## Project-24:
### [Illuminated Optical Magnifier](https://www.electronicsforu.com/electronics-projects/illuminated-optical-magnifier)
This can be used to read the values of ICs and miniature SMD components, detecting cracks or shorts in tracks in a PCB, or reading a finely-graduated vernier scale.This is a simple optical magnifier that simultaneously magnifies and illuminates the object.
#### Components used
convex lens (focal length of about 10cm),
a set of eight white LEDs,
LED driver circuit
#### How it works
An LED driver's main purpose is to rectify higher AC voltage to low DC voltage. LED drivers also protect LEDs from voltage or current fluctuations. LED light output is proportional to its current supply.In this project LED driver is made of 2 capacitors, 4 diodes(IN4007),one inductor and one resistor.
#### Implementation
Capacitor C1 at the input reduces the line-input voltage of 230V to a very-low-level AC voltage. The full-wave bridge rectifier, comprising diodes D1 through D4, converts the low-level AC voltage into DC voltage. Another capacitor is used for smoothening. Series current-limiting resistor and series inductor coil to avoid voltage spikes.
8 LEDs are connected in a circular shape. And convex lens is placed in between the LEDs.

## Project-25:
### [Metal Detector using Arduino](https://circuitdigest.com/microcontroller-projects/arduino-metal-detector-circuit-code)
#### Components used

Arduino, 
Coil,
10nF capacitor,
Buzzer,
The 1k resistor,
330-ohm resistor,
LED,
1N4148 diode,
#### How it works
Whenever some current passes through the coil, it generates a magnetic field around it. And the change in the magnetic field generates an electric field. Now according to Faraday's law, because of this Electric field, a voltage develops across the coil which opposes the change in magnetic field and that’s how Coil develops the Inductance.When any metal comes near to the coil then coil changes its inductance
The 1N4148 is a standard silicon switching diode. It is one of the most popular and long-lived switching diodes because of its dependable specifications and low cost. The 1N4148 is useful in switching applications up to about 100 MHz with a reverse-recovery time of no more than 4 ns.
#### Implementation
 We will use a LR circuit.We have used an Arduino Nano for controlling whole this Metal Detector Project. A LED and Buzzer are used as metal detection indicator. A Coil and capacitor is used for detection of metals. A signal diode is also used for reduce the voltage. And a resistor for limiting the current to the Arduino pin. A pulse is sent by Arduino to LR high pass filter and short spikes will be generated. The pulse length of the generated spikes is proportional to the inductance of the coil. a capacitor which is charged by the rising pulse or spike. And it required few pulses to charge the capacitor to the point where its voltage can be read by Arduino analog pin A5. Then Arduino read the voltage of this capacitor by using ADC. After reading voltage, capacitor quickly discharged by making capPin pin as output and setting it to low. We repeat measurement and took an average of the results. That’s how we can measure the approximate inductance of Coil. After getting the result we transfer the results to the LED and buzzer to detect the presence of metal. 

