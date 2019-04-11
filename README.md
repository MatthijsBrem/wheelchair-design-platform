# Disco wheelchair
![Poster](/docs/resources/Wheelchair_visual_idea.jpg)
To improve interaction between children in their breaks at school we designed the Discochair. This chair uses different sensors to change the music and lights on the wheelchair. Some are controlled by the wheelchair users and some are controlled by the people surrounding the wheelchair user.


To achieve this we used a Arduino mega, a Raspberry Pi and several sensors and actuators which can be found below. The Arduino was mainly used for gathering sensor data and actuating the LED's. The Raspberry Pi was used for processing the data in python using machine learning.  The processed data was then used as an input for PureData, a graphical programming environment for audio and video processing. Which also ran on the Raspberry Pi.

![Dataschemetic](/docs/resources/Poster_Group_4_Disco_Wheelchair_Data_Flow.jpg)


This project was done for the elective Developing connected products and services for the internet of things. Part of the TU Delft IDE Master track. It uses [this](https://github.com/datacentricdesign/wheelchair-design-platform) repository as bassis and other libraries provided by the course.

## Sensors
The following list of sensors should be implemented for the realization of the “Disco chair” a chair that enables and stimulates physical activity and interaction between children with a wheelchair and children without.

- gesture sensor (Adafruit_APDS9960)

properties and behaviours: detects a hand going up, down, left or right.

Motivation of choice: We want to introduce the movements of remixing a song and use these movements to give the music a backspin effect.
- proximity sensor (SHARP_2Y0A02 x 3)

properties and behaviours: The sensor measures if something is in front of him and based on the distance the voltages changes. This can be converted into distance in cm.

Motivation of choice: When a person approaches between 20 cm and 1 m of distance additional samples will be added the music.
- pressure sensor x 4 (FSR 406)

Properties and behaviours: measures the pressure on the seat of the user.

Motivation of choice: by leaning left, right, forward or backwards we want to control the pitch of the music and the filtering of the music. This introduces new tools for the children to remix the song. Leaning forward means a pitch increase, leaning backwards leads to a lower pitch. Leaning to the right applies a high pass filter to the music and leaning left applies a lowpass filter to the music. Both filters are controlled by a ramp function that is dependent on how long the leaning is performed.


## Actuator

- NeoPixel :

Properties and behaviour: an LED lights up once a current is sent through the diode. LEDs are suitable for flashing and are available in RGB, but effectively every colour can be made. LEDs can also easily be dimmed by lowering the current or by pulse width modulation.  

Motivation for choice: We want to create a wheelchair that is nice to use on a dance floor. The LEDs can be used to trigger communication/interaction with others on the dancefloor. Furthermore, the LEDs can light up in a pattern that is linked to the rhythm of the music, and by doing so, making the wheelchair the centrepiece of the dancefloor.

Ideally, the lights of the club would change accordingly to the music remixes of the wheelchair user, in order to prototype this LED lights can be used.

- Speaker:

Properties and behaviour: A Speaker is a electroacoustic transducer which converts an electrical audio signal into a corresponding sound. The speaker that will be used is a dynamic speaker. Dynamic speakers work when an alternating current electrical audio signal is applied to a coil of wire suspended in a circular gap between the poles of a permanent magnet. The rapid movement back and forth of the coil causes the movement of a diaphragm attached to the coil, this creates sound waves.

Motivation for choice: The selection of the speaker as an actuator for the project is based on the main objective of the project: Making the movements and interaction with the surroundings of the wheelchair expressed in music variation in tone, pitch and effects for the music. The speaker will, therefore, allow releasing the remixed tones based on the data collected.

## Other components

 - Arduino Mega
 - Raspberry pi
 - 2 x Powerbank (5v)
 - Breadboard
 - Sensors
	  - gesture sensor (Adafruit_APDS9960)
	  -  proximity sensor (SHARP_2Y0A02 x 3)
	  - 4 x pressure sensor (FSR 406)
	  - Microphone (sparkfun sound detector)
- Actuators
	- LED strip (AdaFruit Neopixel)
	- Speaker
- 4x 220 ohm resistor
-  470 ohm resistor
- 1 Farad capacitor

# Arduino
All our sensors are Attached to the Arduino, the wiring can be seen below. Afterwards we explain how we use the speaker as an input for out LED strip. Then we will explain how we gather data from the sensors and communicate them with the Raspberry Pi.
The Arduino code was written in another editor then the standard Arduino editor which is why it is an .cpp file and not a .ino file. However is should work perfectly fine if you copy all the code to an .ino file.
## Wiring Schematic
![Wiring](/docs/resources/SchematicWiring.jpg)
The above image can be used when connecting sensors and actuators to the Arduino Board and Raspberry Pi.
## Arduino Sound & LEDS
explain the sound sensor and led actuation here
## Arduino Other Sensors
A function is written for reading each sensor, so our void loop() calls each sensor individually. This is done to keep the code more clean and readable.
### gesture sensor
For the gesture sensor the library [Adafruit_APDS9960](https://github.com/adafruit/Adafruit_APDS9960) is used. To initialize the sensor create a variable Adafruit_APDS9960.

    Adafruit_APDS9960 apds;

Then in the void setup begin reading the sensor and reading the proximity and gesture.

    if (!apds.begin())
    { // Begining the work period of the sensor
    Serial.println("Failed to initialize Sensor! Please check your wiring.");
    }
    else
    Serial.println("Gesture Sensor initialized!");
    apds.enableProximity(true);
    apds.enableGesture(true);

In the function that is called from the loop, read the gesture sensor and store that into a uint8_t. To Check  which gesture is measured compare it to either APDS9960_UP, APDS9960_DOWN, APDS9960_LEFT or APDS9960_RIGHT.

    uint8_t gesture = 0;
    void gestureSensor()
    {

      gesture = apds.readGesture(); // Read gesture into the variable
      if(gesture == APDS9960_UP)
      {
        // then sensor reads up
      }

It is good to know the gesture only reads something if it actually measures a gesture. If there is trouble with how reliable the sensor detects your gesture try adding `apds.setLED(APDS9960_LEDDRIVE_12MA, APDS9960_LEDBOOST_100PCNT);` to your void setup(). These values worked best for us but you can change the first argument to one of the following options:
- APDS9960_LEDDRIVE_100MA
- APDS9960_LEDDRIVE_50MA
- APDS9960_LEDDRIVE_25MA
- APDS9960_LEDDRIVE_100MA

and the second argument to:
- APDS9960_LEDBOOST_100PCNT
- APDS9960_LEDBOOST_150PCNT
- APDS9960_LEDBOOST_200PCNT
- APDS9960_LEDBOOST_300PCNT

### Distance

### Pressure Sensor
Four Force Resistive Sensors are used on the seating of the wheelchair. The code for each of them follows the following structure.

    #define PRESSURE_PIN1 A1                                  // Setting up pin to receive voltage PRESSURE 1
    int value_Pressure_1, prev_value_Pressure_1 = -10000;     // int values (read from analog port, both the current and the previous)
    int deviationPressure = 0;                                // setting the minimum deviation between the measurements

### communication
Communication with the python code is done through the serial port. This means that in Arduino, the values the sensors measure are printed to the Serial port with an unique identifier in front of them. Then the python code reads the serial port of the Raspberry Pi and processes it. The example given is from the Pressure sensor. To prevent errors it is advisable to first put everything into one string before printing it.

    String pressureStringBuf;
    pressureStringBuf += discopressure-5988;
    pressureStringBuf += ",";
    pressureStringBuf += String(newton_value_Pressure_1);
    pressureStringBuf += ",";
    pressureStringBuf += String(newton_value_Pressure_2);
    pressureStringBuf += ",";
    pressureStringBuf += String(newton_value_Pressure_3);
    pressureStringBuf += ",";
    pressureStringBuf += String(newton_value_Pressure_4);
    Serial.println(pressureStringBuf);

The python code will then use the comma's as a separator to process the data.
# Raspberry Pi
The Raspberry Pi runs a python script and a Pure Data sketch. The python script reads the serial port of the Raspberry pi, to which the Arduino is connected. The Raspberry pi processes this data and sends it to the Pure Data sketch that controls the audio based on this input.

From the Arduino its gets the Distance, Gesture and Pressure data. The Distance and Gesture is send to Pure Data directly. The Pressure data is run through a trained model which classifies the data into 6 different Posture of sitting. The postures are : not sitting, sitting normal, leaning forward, leaning backward, leaning left, and leaning right.
## Reading the Serial Monitor
To open the serial port in the python script the following code is added.

    import os                       

    ser = serial.Serial(
        port = os.environ['SERIAL'],
        baudrate = 9600,
        write_timeout = 0)
To tell the python where the port is edit the .env file in the Raspberry Pi. This can be done with the following command: `nano .env` from the command line inside the Raspberry pi. The add the following line.

    SERIAL=/dev/TTYUSB0

The port can also be ttyS0 or ttyAM0

To read from the now opened serial port a function is created. This function checks if there are bites in the line, then it decodes it into a string using utf-8. From there the values are split using the ",". The first value is the unique identifier which is the first value that is put into the string on the Arduino.

    def serial_Reader():
        line_bytes = ser.readline()
        if(len(line_bytes)) > 0:
            line = line_bytes.decode('utf-8')
            values = line.split(',')
            property_id = values.pop(0)
            print(property_id)

Then depending on the identifier it is either send to Pure Data (Gesture, and Distance) or it is send to the trained classification model and a prediction is made.
## Machine learning
For classifying the pressure sensor data into different postures, example code provided by the course was used. This consisted of two python scripts, the first one collects data, the second on creates and trains a model.

### training
In the first file we defined which classes we wanted, set the label name and data prop for the communication with the provided server. Followed by the amount of samples we want for each class and the delay in-between each class

    # Sitting classes
    CLASSES = ["No Sitting", "Normal Sitting", "Forward",          "Backward", "Left", "Right"]

    LABEL_PROP_NAME = "discoPostures"
    DATA_PROP_NAME = "discoPressure"

    # How many samples do we want for each class
    MAX_SAMPLES = 2000
    # How much time (in seconds) to leave between the collection of each class
    DELAY_BETWEEN_POSTURE = 10

When the code runs it asks you to sit in each posture until it has gathered the amount of samples you want and then it goes to the next posture. This needs to run on the raspberry pi which is connected to the Arduino because it reads the sensors from the Arduino.
### classification
The second file uses the libraries Sk-learn and Pands which can by installed by typing the following command in the command line on the Raspberry Pi.

    pip install -U scikit-learn
    pip install pandas

If that does not work which was our case, you can also try:

    sudo apt-get install python3-sklearn
    sudo apt-get install python3-pandas

Having installed the libraries the python code needs to know where to look on the server. This is done by giving it the start and end time of when you gathered data in Unix time stamp in your local time and giving it the same label and data that was given to it in the previous file.

    START_TS = 1554468600000
    END_TS = 1554468600000 + 780000

    # Property ID
    PROPERTY_DATA = "discoPressure"
    PROPERTY_LABEL = "discoPostures"

Running the code should provide a file called "model.pickle". This model is accessed from the python file that constantly runs on the Raspberry Pi .

### Reading the model
In the original file where the serial monitor was read the model needs to be imported.

    MODEL_FILE_NAME = "model.pickle"

    #load classifier
    with open("model.pickle", 'rb') as file:
      neigh = pickle.load(file)

    classes =["No Sitting", "Normal Sitting", "Forward","Backward", "Left", "Right"]

The values that were read from the serial monitor need to be reshaped to use as input for our model.

    values = [float(x) for x in values]
    values = [values]
    np.array(values).reshape(1, -1)
    predict(values)

As can be seen in the code above the function predict is called. This function uses the model to make a prediction based on the data, prints it and sends it to the function that communicates it with Pure data.

    def predict(values):
        result = neigh.predict(values)
        print(classes[result[0]])
    translatePredictionToPD(result)


### Filtering the data
The translatePredictionToPD() has a bit of processing to filter out some inaccuracies. We noticed the model could predict the same posture for several seconds but would then suddenly have a one off prediction and then go back to the old prediction. Because this would created undesired behaviour this was filtered

Every prediction is put into an list with all predictions. For each new prediction the script checks if this prediction is the same as the previous three prediction and not the same as the first prediction. This means there are 4 consecutive predictions which are not the current prediction. Meaning it is not a one off prediction but a new posture. The list is then cleared and the new prediction is added to the list.

    def translatePredictionToPD(prediction):
        if len(lastpredictions) > 4:
            if lastpredictions[-1] == prediction:
                if lastpredictions[-2] == prediction:
                    if lastpredictions[-3] == prediction:
                        if lastpredictions[0] != prediction:
                            if lastpredictions[0] == 0:
                                music_on = "0 1 ;"
                                s.send(music_on.encode('utf-8'))
                                print("turning the music on")
                            lastpredictions.clear()
                            print("Time to clear the list")

    lastpredictions.append(prediction[0])

There is also a check to see if the previous prediction was not sitting because this means that the music needs to be turned on no matter the current predicted posture. Inside this check you get a preview of how the communication with pure data works, more about this in the next chapter.
#    Pure data

Pure Data is a visual programming language that can be used to manipulate data (e.g. video data or audio data). The advantage of Pure Data is, that it runs on linux and thus can be run on the raspberry pi.

The latest version of Pure Data can be found on the Pure Data website (https://puredata.info/downloads)

To install Pure Data on the raspberry, connect to the raspberry and type the following text in the command line:

    sudo apt-get install puredata

Now we take a look at the Pure Data file short.pd:

It is best to open the pure data file on your mac or pc for inspection, as a GUI will open. The Gui will show a couple of different building blocks. The left side of the patch is for receiving signals from python, this will be explained in the paragraph under this one.

The rest of the patch is dedicated to manipulating the sound of the short audio file that the patch opens (shortloop.wav).

Upon opening the short.pd patch, the loadbang will trigger three elements;

1.  The pitch is set to 0.06 (this corresponds roughly to a normal
    playing speed, in case of a different file, this value has to be retuned, the value for the normal playing speed can be read under the 44100/$f1 block.

2.  The audio file is loaded into soundData and cut into samples. If an      
    other file is used, just change the file name. In case it is placed in the same folder as the pd file, no directory path is needed.

3.  The dsp is turned on, this means that audio can be broadcasted by Pure
    Data.

The right side of the patch is dedicated to playback speed and direction of the sound; both the direction and pitch are controlled based upon the inputs from python.

After the pitch has been set, the sound is then again compiled from the samples, hereafter the effects are applied to the sound.

# PD & Python communication

For the communication the sockets are used, so start the code with

    import socket

After this the socket is initialized and the port number is set. Make sure the netreceive in Pure Data is set to the same port number.

    s = socket.socket()
    host = socket.gethostname()
    port = 3000

Now the connection can be established

    s.connect((host,port))

Commands are now send using:

    s.send(command.encode('utf-8'))

where command is built up like:

    command= “PDport “ + “signal” + “ ;”

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

More on the Arduino Mega can be found [here](/docs/resources/Arduino.md "Arduino resources").

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
