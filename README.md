# Disco wheelchair
![Poster](/docs/resources/Wheelchair_visual_idea.jpg)
To improve interaction between children in their breaks at school we designed the Discochair. This chair uses different sensors to change the music and lights on the wheelchair. Some are controlled by the wheelchair users and some are controlled by the people surrounding the wheelchair user.
A video explanation can be seen [here](https://youtu.be/QbcIBNa1BgE)

To achieve this we used a Arduino mega, a Raspberry Pi and several sensors and actuators which can be found below. The Arduino was mainly used for gathering sensor data and actuating the LED's. The Raspberry Pi was used for processing the data in python using machine learning.  The processed data was then used as an input for PureData, a graphical programming environment for audio and video processing. Which also ran on the Raspberry Pi.

![Dataschemetic](/docs/resources/Poster_Group_4_Disco_Wheelchair_Data_Flow.jpg)


This project was done for the elective Developing connected products and services for the internet of things. Part of the TU Delft IDE Master track. It uses [this](https://github.com/datacentricdesign/wheelchair-design-platform) repository as basis and other libraries provided by the course.

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
In order to be able to use the Neoprixel LED Strip the Adafruit NeoPixel library [Adafruit_NeoPixel.h](https://github.com/adafruit/Adafruit_NeoPixel) has to be used and included.

    #include <Adafruit_NeoPixel.h>  //Library to simplify interacting with the LED strand
    #ifdef __AVR__
    #include <avr/power.h>   //Includes the library for power reduction registers if your chip supports them.
    #endif

The following step is to define constants

    #define LED_PIN   6  //Pin for the pixel strand.
    #define LED_TOTAL 80  //Number of LEDs in your strand.
    #define LED_HALF  LED_TOTAL/2 Sets the starting point from which the leds are going to turn on.
    #define AUDIO_PIN A5  //Pin for the envelope of the sound detector

In the globals the values used to make comparisons of the recordings of the Sound detector are stated:

    Adafruit_NeoPixel strand = Adafruit_NeoPixel(LED_TOTAL, LED_PIN, NEO_GRB + NEO_KHZ800);  //LED strand objetc

    uint16_t gradient = 0; //Used to iterate and loop through each color palette gradually
    uint8_t volume = 0;    //Holds the volume level read from the sound detector.
    uint8_t last = 0;      //Holds the value of volume from the previous loop() pass.
    float maxVol = 15;     //Holds the largest volume recorded thus far to proportionally adjust the visual's responsiveness.
    float avgVol = 0;      //Holds the "average" volume-level to proportionally adjust the visual experience.
    float avgBump = 0;     //Holds the "average" volume-change to trigger a "bump."

    bool bump = false;     //Used to pass if there was a "bump" in volume
In the void setup the reading is set

    void setup() {    

    strand.begin(); //Initialize the LED strand object.
    strand.show();  //Show a blank strand, just to get the LED's ready for use.  
    }

The void loop section is where the code gets more interesting. Initially the volume of the music is read.

    volume = analogRead(AUDIO_PIN);       //Record the volume level from the sound detector
    avgVol = (avgVol + volume) / 2.0;     //Take our "average" of volumes.

The following line sets which level to consider as noise. In our case it was set to 15

     if (volume < avgVol / 2.0 || volume < 15) volume = 0;

the code overwrites the information once the current volume is larger than the loudest value recorded

     if (volume > maxVol) maxVol = volume;

The gradient is reset every time a palette gets completed

     if (gradient > 1529) {
         gradient %= 1530;
         maxVol = (maxVol + volume) / 2.0;
       }

If there is a decent change in volume since the last pass, we average it into "avgBump" and if there is a notable change in volume, trigger a "bump"

       if (volume - last > avgVol - last && avgVol - last > 0) avgBump = (avgBump + (volume - last)) / 2.0;

       bump = (volume - last) > avgBump;

The visual Pulse is then displayed and the gradient incremented. Afterwards, the current volume is recorded for the next steps and paces the visuals to make them more enjoyable

       Pulse();  
       gradient++;   
       last = volume; //Records current volume for next pass
       delay(30);

In the following section the functions of graphical display of the LEDs are set

    void Pulse() {

    fade(0.75);   //dims the colors a little bit each pass of loop()

    //Advances the gradient to the next noticeable color if there is a "bump"
    if (bump) gradient += 64;

    //If it's silent, we want the fade effect to take over, hence this if-statement
    if (volume > 0) {
    uint32_t col = Rainbow(gradient); //Our retrieved 32-bit color

These variables determine where to start and end the pulse since it starts from the middle of the strand. The quantities are stored in variables so they only have to be computed once.

    int start = LED_HALF - (LED_HALF * (volume / maxVol));
    int finish = LED_HALF + (LED_HALF * (volume / maxVol)) + strand.numPixels() % 2;

Listed above, LED_HALF is simply half the number of LEDs on your strand. ↑ this part adjusts for an odd quantity.

    for (int i = start; i < finish; i++) {

      //"damp" creates the fade effect of being dimmer the farther the pixel is from the center of the strand.
      //  It returns a value between 0 and 1 that peaks at 1 at the center of the strand and 0 at the ends.
      float damp = float(
                     ((finish - start) / 2.0) -
                     abs((i - start) - ((finish - start) / 2.0))
                   )
                   / float((finish - start) / 2.0);

Sets the each pixel on the strand to the appropriate color and intensity. strand.Color() takes 3 values between 0 & 255, and returns a 32-bit integer.

      strand.setPixelColor(i, strand.Color(
                             split(col, 0) * pow(damp, 2.0) ,
                             split(col, 1) * pow(damp, 2.0) ,
                             split(col, 2) * pow(damp, 2.0)
                           ));
    }
Sets the max brightness of all LEDs. If it's loud, it's brighter.

    strand.setBrightness(255.0 * pow(volume / maxVol, 2));
    }

This command actually shows the lights. If you make a new visualization.

    strand.show();
    }

This function fades lights by multiplying them by a value between 0 and 1 each pass of loop().
    void fade(float damper) {

    //"damper" must be between 0 and 1, or else you'll end up brightening the lights or doing nothing.
    if (damper >= 1) damper = 0.99;

    for (int i = 0; i < strand.numPixels(); i++) {

    //Retrieve the color at the current position.
    uint32_t col = (strand.getPixelColor(i)) ? strand.getPixelColor(i) : strand.Color(0, 0, 0);

    //If it's black, you can't fade that any further.
    if (col == 0) continue;

    float colors[3]; //Array of the three RGB values

    //Multiply each value by "damper"
    for (int j = 0; j < 3; j++) colors[j] = split(col, j) * damper;

    //Set the dampened colors back to their spot.
    strand.setPixelColor(i, strand.Color(colors[0] , colors[1], colors[2]));
    }
    }

    uint8_t split(uint32_t color, uint8_t i ) {

    //0 = Red, 1 = Green, 2 = Blue

    if (i == 0) return color >> 16;
    if (i == 1) return color >> 8;
    if (i == 2) return color >> 0;
    return -1;
    }


This function simply take a value and returns a gradient color in the form of an unsigned 32-bit integer. The gradient returns a different, changing color for each multiple of 255. This is because the max value of any of the 3 LEDs is 255, so it's an intuitive cutoff for the next color to start appearing. Gradients should also loop back to their starting color so there's no jumps in color.

    uint32_t Rainbow(unsigned int i) {
    if (i > 1529) return Rainbow(i % 1530);
    if (i > 1274) return strand.Color(255, 0, 255 - (i % 255));   //violet -> red
    if (i > 1019) return strand.Color((i % 255), 0, 255);         //blue -> violet
    if (i > 764) return strand.Color(0, 255 - (i % 255), 255);    //aqua -> blue
    if (i > 509) return strand.Color(0, 255, (i % 255));          //green -> aqua
    if (i > 255) return strand.Color(255 - (i % 255), 255, 0);    //yellow -> green
    return strand.Color(255, i, 0);                               //red -> yellow
    }
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

On the back of the wheelchair a distance sensor (SHARP_2Y0A02) is placed. This sensor is used to sense what distance other people are located from the chair. By coming closer to the chair, new sound elements will be added.

The following definitions need to be made for the distance sensor:

    #define IR_PIN A0 // Setting up pin to receive voltage from IR

    int value, prev_value = -10000;       // int values (read from analog port, both the current and the previous)
    int deviation = 0;                                       
    double voltage_value, distance_value; // Converted to Voltage

In the first part of the code, the variables and the pin on the Arduino that is used for receiving the signal are defined.

To keep the results somewhat reliable the next part of code checks whether the sensor data is within the limits in which the sensor can make usable predictions

    double convert_to_distance(double voltage)
    {
        if (voltage_value < 0.35 || voltage_value > 2.85)
        return (0);


The next piece of code is just a piece of commentary that specifies the parameters with which te sensor works.

      double a = 4498;
      double b = -6.351;
      double c = 104.9;
      double d = -0.6928;

      return (a * exp(b * voltage) + c * exp(d * voltage));
    }


The next part of the code reads the data from the sensor and checks whether it is the same +/- the standard deviation of the former measurement. If it is the same, it returns, otherwise it reads the signal and converts it to a voltage.

    void distance()
    {
        value = analogRead(IR_PIN); // reading our analog voltage, careful we only have 10 bit resolution so each measurement step is only 5V ÷ 1024, so our result will be 0 - 1023

        // if value is within the range of [ previous - σ , previous + σ], ignore it (if value is relatively the same) this will help with having data ocuppy your buffer that is not a significant deviation.
        if (value >= (prev_value - deviation) && value <= (prev_value + deviation))
          return;

        voltage_value = double((value * 5)) / 1023; // converting to voltage [ 0, 5] v.


The next piece of code is used to translate the voltage value into a distance value (cm). It then checks whether the measured signal is within the range of measurement. If it is out of the range, the code will return.

    distance_value = convert_to_distance(voltage_value); // getting actual distance value(cm)
Careful using this, accuracy may not be ideal. Due to the functioning of the sensor, once you're closer than around 20 cm, it will start predicting higher distances again. Be careful with this, this is something you can solve with software, however (if previous results are close to 20 and its going down) then do something.... to ignore results, perhaps.

    if (distance_value < 20 || distance_value > 150) // We will ignore values outside the range of measurement, this will happen around 2.7 -0.4 v
      return;


The last bit of code is used to print the measured value into the serial & it updates the previous value that was measured to the new value.


### Pressure Sensor
Four Force Resistive Sensors are used on the seating of the wheelchair. The code for each of them follows the following structure.

    #define PRESSURE_PIN1 A1                                  // Setting up pin to receive voltage PRESSURE 1
    int value_Pressure_1, prev_value_Pressure_1 = -10000;     // int values (read from analog port, both the current and the previous)
    int deviationPressure = 0;                                // setting the minimum deviation between the measurements
The data of the Voltage measured by the FSRs are converted in Newtons

    double convert_to_newtons(double voltage)
    {double a = 0.01419;
    double b = 0.9523;
    double c = -0.01461;
    double d = -2.231;

    return ((a * exp(b * voltage) + c * exp(d * voltage)) * 9.81);
    }

The void loop for the pressure sensors sets the sensibility of the sensors by comparing the measured value to the previous ones
```
{  value_Pressure_1 = analogRead(PRESSURE_PIN1);
  if (value_Pressure_1 >= (prev_value_Pressure_1 - deviationPressure) && value_Pressure_1 <= (prev_value_Pressure_1 + deviationPressure))
    return;

  voltage_value_Pressure_1 = double((value_Pressure_1 * 5)) / 1023;       // converting to voltage [ 0, 5] v.
  newton_value_Pressure_1 = convert_to_newtons(voltage_value_Pressure_1); // getting actual force value (careful using this, accuracy may not be ideal)
                                                                          // sensitivity after 1Kgf and before 0.06kgf is limited, you can lower the deviation

  prev_value_Pressure_1 = value_Pressure_1;

  prev_value_Pressure_2 = value_Pressure_2; // PRESSURE SENSOR 2

  value_Pressure_2 = analogRead(PRESSURE_PIN2);

  String pressureStringBuf;
  pressureStringBuf += pressureID;
  pressureStringBuf += ",";
  pressureStringBuf += String(newton_value_Pressure_1);
  pressureStringBuf += ",";
  pressureStringBuf += String(newton_value_Pressure_2);
  pressureStringBuf += ",";
  pressureStringBuf += String(newton_value_Pressure_3);
  pressureStringBuf += ",";
  pressureStringBuf += String(newton_value_Pressure_4);
  Serial.println(pressureStringBuf);
  ```

As with the rest of the sensors the loop is called in the end in order to merge informstions in one file

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

In order to manipulate the music that is being played on the Pi, another programming language is used, which suits itself well to music manipulation. This language is called Pure Data. Pure Data is used as the link between the sensor data that is processed in python on the Pi and the audio output.

After the processing of the sensor data in python, the commands that correspond with the behavior are sent to PureData over a socket, using TCP communication. PureData receives these commands and routes them to the right parts in the code. This then controls the different variables that adjust pitch/play direction/delay. The advantage of Pure Data is, that it runs on linux and thus can be run on the raspberry pi.

The latest version of Pure Data can be found on the Pure Data website (https://puredata.info/downloads)

To install Pure Data on the raspberry, connect to the raspberry and type the following text in the command line:

    sudo apt-get install puredata

Now we take a look at the Pure Data file short.pd:

![Pure Data Overview](/docs/resources/PureD.png)

It is best to open the pure data file on your mac or pc for inspection, as a GUI will open. The Gui will show a couple of different building blocks, like te picture above. The left side of the patch is for receiving signals from python, this will be explained in the paragraph under this one.

The rest of the patch is dedicated to manipulating the sound of the short audio file that the patch opens (shortloop.wav).

Upon opening the short.pd patch, the loadbang will trigger three elements;

1.  The pitch is set to 0.06 (this corresponds roughly to a normal playing speed, in case of a different file, this value has to be retuned, the value for the normal playing speed can be read under the 44100/$f1 block.

2.  The audio file is loaded into soundData and cut into samples. If an other file is used, just change the file name. In case it is placed in the same folder as the pd file, no directory path is needed.

3.  The dsp is turned on, this means that audio is send to the soundcard by Pure Data.

The right side of the patch is dedicated to playback speed and direction of the sound; both the direction and pitch are controlled based upon the inputs from python.

After the pitch has been set, the sound is then again compiled from the samples, hereafter the effects are applied to the sound.

With the input data of the sensors, several audio effects can be controlled within Pure Data. In this example, a highpass and low pass filter are present, a delay is present, the pitch can be controlled within a range and the audio file can be played in reverse. Furthermore the delay effect is also controlled by the gesture sensor data. An upward gesture activates the delay and a downward gesture deactivates the delay.



# PD & Python communication

For the communication between pure data and python sockets are used. The complete code can be found in the python script reading_from_terminal.py

To enable the sockets, the code starts with

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

The routing in Pure Data is set in such a way, that based on the Pdport value that is sent in the message, the signal is routed to the right input (either controlling the frequency of the filters, the tempo or the reverse/stop/play setting)

In the python code, the control of the actuators is defined. That is done in the following pieces of code, which is part of the reading_from_terminal.py code:

```
if lastpredictions[0] == 0:
    music_off = "1 1 ;"
    s.send(music_off.encode('utf-8'))
    print("turning the music off")
```

Not sitting turns the music off

```
elif lastpredictions[0] == 1:
    #music_on = "0 1 ;"
    #s.send(music_on.encode('utf-8'))
    #print("turning the music on")
```
Sitting turns the music on

Next are the filters. The filters are reset if no prediction is made of either sitting to the left or sitting to the right.

```
    if lowpass != 20000:
        lowpass = 20000
        lowpass_normal = "4 20000 ;"
        s.send(lowpass_normal.encode('utf-8'))
    if highpass != 20:
        highpass = 20
        highpass_normal = "3 20 ;"
        s.send(highpass_normal.encode('utf-8'))
```

Next the pitch is ontrolled by sitting forward (prediction ==2) or sitting backwards (prediction == 3) The pitch is increased for sitting forward and decreased for sitting backwards

```
elif lastpredictions[0] == 2:
    if currentPitch < 0.08:
        currentPitch += 0.000428
    else:
        currentPitch = 0.08
    pitch_decrease = '6 ' + str(currentPitch) + " ;"
    s.send(pitch_decrease.encode('utf-8'))
    print("the pitch that is send is")
    print(currentPitch)
elif lastpredictions[0] == 3:
    if currentPitch > 0.044:
        currentPitch -= 0.000428
    else:
        currentPitch = 0.044
    pitch_decrease = '6 ' + str(currentPitch) + " ;"
    s.send(pitch_decrease.encode('utf-8'))
    print("the pitch that is send is")
    print(currentPitch)
```

The lowpass filter frequency is lowered for sitting to the left (prediction == 4) and the highpass filter frequency is increased for sitting to the right (prediction == 5)
```
elif lastpredictions[0] == 4:
    if lowpass > 200:
        lowpass -= 250
    else:
        lowpass = 200
    lowpass_str = "4 " + str(lowpass) + " ;"
    s.send(lowpass_str.encode('utf-8'))
elif lastpredictions[0] == 5:
    if highpass < 5000:
        highpass += 25
    else:
        highpass = 5000
    highpass_str = "3 " + str(highpass) + " ;"
    s.send(highpass_str.encode('utf-8'))
```

Then finally, the gesture sensor inputs are used to controll the reverse (reverse on for a right to left gesture and reverse off for left to right gesture) & for adding a delay effect to the music (delay on for gesture upwards and off for gesture downwards)

```
elif(property_id == GestureID):
        prop = my_thing.properties[property_id]

        if prop is not None:
            prop.update_values([float(x) for x in values])
        else:
            print('Warning: unknown property ' + property_id)

        print("the gesture is as follows: ")
        values = [float(x) for x in values]
        print (values[0])
        if values[0] == 4:
            reverse_str = "2 1 ;"
            s.send(reverse_str.encode('utf-8'))
            print("enabling reverse")
        elif values[0] == 6:
            reverse_str = "0 1 ;"
            s.send(reverse_str.encode('utf-8'))
            print("disableing reverse")
        elif values[0] == 2:
            delay_str = "5 0 ;"
            s.send(delay_str.encode('utf-8'))
        elif values[0] == 8:
            deylay_str = "5 0.6 ;"
            s.send(delay_str.encode('utf-8'))
```

# Evaluation and further development

The project has satisfied some of the initial aims by providing a functional prototype for a wheelchair enabled for musical interaction of its users.
The sensitivity of the gesture sensor could be enhanced for a better precision.
The project can be further developped in order to integrate other sensors such as the distance sensor on the back of the wheelchair. The idea would then also be to include other loops apart from shortloop.wav and use a sampler in Pure Data. The sensor inputs could then also be used to build a song from different loop elements and layer them over each other. This would need more time in order to classify the readings of the distance sensor & for implementing a sampler into the Pure Data code. Furthermore the LED actuation could be also linked to the Machine learning instead of being regulated with the Arduino Mega. In this way the change of colour or behaviour could be linked to a specific position of the user.
An other element of possible improvement is having a different setting of the sound from the speaker for a louder output.
