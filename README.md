# Disco wheelchair
To improve interaction between children in their breaks at school we designed the Discochair. This chair uses different sensors to change the music and ligths on the wheelchair. Some are controlled by the wheelchair users and some are controlled by the people surrounding the wheelchair user.

# Sensors
The following list of sensors should be implemented for the realization of the “Disco chair” a chair that enables and stimulates physical activity and interaction between children with a wheelchair and children without.

- gesture sensor (Adafruit_APDS9960)

properties and behaviours: detects a hand going up, down, left or right.

Motivation of choice: We want to introduce the movements of remixing a song and use these movements to give the music a backspin effect.
- proximity sensor (SHARP_2Y0A02 x 3)

properties and behaviours: The sensor measures if something is in front of him and based on the distance the voltages changes. This can be converted into distance in cm.

Motivation of choice: When a person approaches between 20 cm and 1 m of distance a delay will be added to the music. In this way interaction with the user of the wheelchair is promoted, as positioning oneself close to the wheelhair changes the music.
- pressure sensor x 4 (FSR 406)

Properties and behaviours: measures the pressure on the seat of the user.

Motivation of choice: by leaning left, right, forward or backwards we want to control the pitch of the music and the filtering of the music. This introduces new tools for the children to remix the song. Leaning forward means a pitch increase, leaning backwards leads to a lower pitch. Leaning to the right applies a high pass filter to the music and leaning leaning left applies a lowpass filter to the music. Both filters are controlled by a ramp function that is dependent on how long the leaning is performed.


# Actuator

- NeoPixel :

Properties and behaviour: an LED lights up once a current is sent through the diode. LEDs are suitable for flashing and are available in RGB, but effectively every colour can be made. LEDs can also easily be dimmed by lowering the current or by pulse width modulation.  

Motivation for choice: We want to create a wheelchair that is nice to use on a dance floor. The LEDs can be used to trigger communication/interaction with others on the dancefloor. Furthermore, the LEDs can light up in a pattern that is linked to the rhythm of the music, and by doing so, making the wheelchair the centrepiece of the dancefloor.

Ideally, the lights of the club would change accordingly to the music remixes of the wheelchair user, in order to prototype this LED lights can be used.

- Speaker:

Properties and behaviour: A Speaker is a electroacoustic transducer which converts an electrical audio signal into a corresponding sound. The speaker that will be used is a dynamic speaker. Dynamic speakers work when an alternating current electrical audio signal is applied to a coil of wire suspended in a circular gap between the poles of a permanent magnet. The rapid movement back and forth of the coil causes the movement of a diaphragm attached to the coil, this creates sound waves.

Motivation for choice: The selection of the speaker as an actuator for the project is based on the main objective of the project: Making the movements and interaction with the surroundings of the wheelchair expressed in music variation in tone, pitch and effects for the music. The speaker will, therefore, allow releasing the remixed tones based on the data collected.

# Other components

 - Aruidno Mega
 - Raspberry pi
 - 2 x Powerbank
 - Breadboard
 - Sensors
	  - gesture sensor (Adafruit_APDS9960)
	  -  proximity sensor (SHARP_2Y0A02 x 3)
	  - 4 xpressure sensor (FSR 406)
	  - Microphone (sparkfun sound detector)
- Actuators
	- LED strip (Neopixel rgbw
	- Speaker

# Arduino
quick introduction here
## Wiring Schematic
enter picture plus explanation here

## Arduino Sound & LEDS
explain the sound sensor and led actuation here
## Arduino Other Sensors
explain all the sensors here plus communication to tthe python code
# Raspberry
introduction of what is running on the raspberry
## Python Code
explanation of the code plus reading the serial monitor & sending everything to the hub
## Machine learning

## Pure data

Description Pure Data

Pure Data is a visual programming language that can be used to manipulate data (e.g. video data or audio data). The advantage of Pure Data is, that it runs on linux and thus can be run on the raspberry pi.

The latest version of Pure Data can be found on the Pure Data website (https://puredata.info/downloads)

To install Pure Data on the raspberry, connect to the raspberry and type the following text in the command line:

sudo apt-get install puredata

explain what it is plus what we do with it

## PD & Python
explain communication between PD & Python

# Sound controlling software

In order to manipulate the music that is being played on the Pi, another programming language is used, which suits itself well to music manipulation. This language is called Pure Data. Pure Data is used as the link between the sensor data that is processed in python on the Pi and the audio output.

After the processing of the sensor data in python, the commands that correspond with the behavior are sent to PureData over a socket, using TCP communication. PureData receives these commands and routes them to the right parts in the code. This then controls the different variables that adjust pitch/play direction/delay.

# A Noisy wheelchair


Wheelchair Design Platform is a repository that contains some resources to help
designers and developers speak the same language, and work together towards
addressing relevant challenges for wheelchair users. It is a collection of
workshop materials, code examples and also a compilation of resources to foster
a prospering research and design community around wheelchair users.


![IoT1 Exhibition](/docs/workshops/images/iot1_exhibition.jpg)

## Workshops

* [Getting started](/docs/workshops/GettingStarted.md)
* [Workshop 1: Building an Internet-Connected Wheelchair](/docs/workshops/Workshop1.md)
* [Workshop 2: Integrating and Visualising Sensor-Based Data](/docs/workshops/Workshop2.md)
* [Workshop 3: Developing Algorithms and Controlling Actuators](/docs/workshops/Workshop3.md)
* [Workshop 4: Developing and Conducting a Data Collection Campaign](/docs/workshops/Workshop4.md)
* [Workshop 5: Implementing a Machine Learning Pipeline](/docs/workshops/Workshop5.md)
* [Workshop 6: Developing a Product Analytics Dashboard](/docs/workshops/Workshop6.md)

## Resources

* This platform uses two programming languages, Python on computers and C on
micro-controllers. While descriptions and examples of code should help you
get started, you can find some additional resources
[here](/docs/resources/software.md "Python and C resources").

* Documentation of your project is key,
[here are some tips and examples](/docs/resources/documentation.md "Documentation tips and examples").

* [Git manipulation such as Pull Request](/docs/resources/git.md "Git manipulation").

## Main Components

__**Disclaimer:**__ the design of this platform focuses on flexibility and
technology exploration rather than optimisation.

The main design includes a Raspberry Pi 3 and an Arduino Mega 2560 on the wheelchair frame.

The Arduino Mega is the micro-controller of the platform. Fixed on the main frame of the wheelchair,
it can collect data from sensors (e.g. force sensors, accelerometers), and trigger actions from actuators
(e.g. LEDs, vibration motors).

More on the Arduino Mega can be found [here](/docs/resources/arduino.md "Arduino resources").

Raspberry Pi is a small computer. It is also fixed to the main frame of the wheelchair,
where it can:
* interact with the Arduino Mega via USB to receive data and transmit commands;
* interact with the Internet to transmit commands and receive data;
* store data locally in files;
* run (machine learning) algorithms.

More on the Raspberry Pi can be found [here](/docs/resources/raspberrypi.md "Raspberry Pi resources").

These components fit together as shown on the following diagram. A large powerbank
powers the Raspberry Pi. The Arduino Mega communicates and receives power from the
Raspberry Pi via USB. A Feather (Arduino-like development board) on the wheel connects to
the Raspberry Pi via Bluetooth to sense and actuate from the wheel.

![Main Wheelchair components](/docs/workshops/images/wheechair-components.png)

## List of suggested components:

On the frame:

* 1 Raspberry Pi 3B;
* 1 SD card (Some come directly with NOOBS installed);
* 1 Arduino Mega;
* 1 Large power bank;
* 1 large breadboard;
* 1 USB cable A/micro (Powerbank to Raspberry Pi);
* 1 USB cable A/B (Raspberry Pi to Arduino Mega).

On the wheel:

* 1 Feather (Bluetooth enabled);
* 1 small power bank;
* 1 small breadboard;
* 1 USB cable A/B (power bank to Arduino Uno).


## Contact and Existing projects

* [The hiking wheelchair](https://github.com/cprecioso/wheelchair-design-platform)
* [The EDU wheelchair](https://github.com/ctsai-1/wheelchair-design-platform)
* [Weelchair tracking for basketball players](https://github.com/FabianIDE/wheelchair-design-platform)
* [Disco Wheelchair](https://github.com/MatthijsBrem/wheelchair-design-platform)
* [Wheelchair Madness 2222](https://github.com/pherkan/wheelchair-design-platform/tree/master/wheelchair)
* [Who is sitting?](https://github.com/Rosanfoppen/wheelchair-design-platform/tree/master/wheelchair)
* [Magic Wheelchair](https://github.com/Yuciena/wheelchair-design-platform)
* [Yoga Wheelchair](https://github.com/artgomad/wheelchair-design-platform)


Feel free to contact us at jacky@datacentricdesign.org. We welcome feedback, pull requests
or links to your project.

#Natasa was here

*hello


#learning github
by Tjapko Vermeulen
<!--stackedit_data:
eyJoaXN0b3J5IjpbMzI2OTM4NzczLDU2NjUzNzU0LC0xMDczMz
U4MzkxLC0xMDk3ODUxODY2XX0=
-->