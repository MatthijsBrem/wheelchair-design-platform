#include <Arduino.h>
// A small helper
void error(const __FlashStringHelper *err)
{
  Serial.println(err);
  while (1)
    ;
}

/*****************************
      Distance sensor
******************************
*/
#define IR_PIN A0 // Setting up pin to receive voltage from IR

String distanceId = "distancegroup4-5518";

int value, prev_value = -10000;       // int values (read from analog port, both the current and the previous)
int deviation = 0;                    // setting the minimum deviation between the measurements (0 by default)
                                      // up to 512 (although that is pretty useless)
double voltage_value, distance_value; // Converted to Voltage

double convert_to_distance(double voltage)
{
  if (voltage_value < 0.35 || voltage_value > 2.85) // We will ignore values outside the range of measurement, this will happen around 20 - 150cm
    return (0);

  /* General model Exp2:
     f(x) = a*exp(b*x) + c*exp(d*x)
Coefficients (with 95% confidence bounds):
       a =        4498  (-1.731e+04, 2.63e+04)
       b =      -6.351  (-12.72, 0.02085)
       c =       104.9  (61.24, 148.5)
       d =     -0.6928  (-0.9048, -0.4808)

Goodness of fit:
  SSE: 9.915
  R-square: 0.9974
  Adjusted R-square: 0.9955
  RMSE: 1.574

 */

  double a = 4498;
  double b = -6.351;
  double c = 104.9;
  double d = -0.6928;

  return (a * exp(b * voltage) + c * exp(d * voltage));
}

void distance()
{
  value = analogRead(IR_PIN); // reading our analog voltage, careful we only have 10 bit resolution so each
                              // measurement step is only 5V ÷ 1024, so our result will be 0 - 1023

  // if value is within the range of [ previous - σ , previous + σ], ignore it (if value is relatively the same)
  // this will help with having data ocuppy your buffer that is not a significant deviation.
  if (value >= (prev_value - deviation) && value <= (prev_value + deviation))
    return;

  voltage_value = double((value * 5)) / 1023; // converting to voltage [ 0, 5] v.

  distance_value = convert_to_distance(voltage_value); // getting actual distance value(cm) (careful using this, accuracy may not be ideal)
                                                       // due to the functioning of the sensor, once you're closer than around 20 cm, it will
                                                       // start predicting higher distances again. Be careful with this, this is something you can
                                                       // solve with software, however. (if previous results are close to 20 and its going down)
                                                       // then do something.... to ignore results, perhaps.

  if (distance_value < 20 || distance_value > 150) // We will ignore values outside the range of measurement, this will happen around 2.7 -0.4 v
    return;
  /*
    Serial.print("Distance: ");
    Serial.print(value);
    Serial.print(" (0 - 1023) steps,  ");
    Serial.print(voltage_value);
    Serial.print(" (v),  ");
    */
  Serial.println();
  Serial.print(distanceId);
  Serial.print(",");
  Serial.println(distance_value);

  prev_value = value; // Here we have the previous saved variable.
}

/*****************************
      Gesture sensor
******************************
*/

// insert gesture varriables Here
#include "Adafruit_APDS9960.h"
uint8_t gesture = 0;

Adafruit_APDS9960 apds;

String gestureID = "gesturesensorgroup4-cc12";

void gestureSensor()
{

  gesture = apds.readGesture(); // Read gesture into the variable

  // Processing captured gesture, if any.
  if (gesture == APDS9960_DOWN)
  {
    Serial.println();
    Serial.print(gestureID);
    Serial.print(",");
    Serial.println("2");
  }
  if (gesture == APDS9960_UP)
  {
    Serial.println();
    Serial.print(gestureID);
    Serial.print(",");
    Serial.println("8");
  }
  if (gesture == APDS9960_LEFT)
  {
    Serial.println();
    Serial.print(gestureID);
    Serial.print(",");
    Serial.println("4");
  }
  if (gesture == APDS9960_RIGHT)
  {
    Serial.println();
    Serial.print(gestureID);
    Serial.print(",");
    Serial.println("6");
  }
  int gesture_value = 1;
}
/*****************************
      Pressure sensor
******************************
*/

// insert gesture varriables here
#define PRESSURE_PIN1 A1                                  // Setting up pin to receive voltage PRESSURE 1
int value_Pressure_1, prev_value_Pressure_1 = -10000;     // int values (read from analog port, both the current and the previous)
int deviationPressure = 0;                                // setting the minimum deviation between the measurements (0 by default)
                                                          // up to 512 (although that is pretty useless)
double voltage_value_Pressure_1, newton_value_Pressure_1; // Converted to Voltage

#define PRESSURE_PIN2 A2                                  // Setting up pin to receive voltage PRESSURE 2
int value_Pressure_2, prev_value_Pressure_2 = -10000;     // int values (read from analog port, both the current and the previous)
                                                          // up to 512 (although that is pretty useless)
double voltage_value_Pressure_2, newton_value_Pressure_2; // Converted to Voltage

#define PRESSURE_PIN3 A3                                  // Setting up pin to receive voltage PRESSURE 3
int value_Pressure_3, prev_value_Pressure_3 = -10000;     // int values (read from analog port, both the current and the previous)
                                                          // up to 512 (although that is pretty useless)
double voltage_value_Pressure_3, newton_value_Pressure_3; // Converted to Voltage

#define PRESSURE_PIN4 A4                                  // Setting up pin to receive voltage PRESSURE 4
int value_Pressure_4, prev_value_Pressure_4 = -10000;     // int values (read from analog port, both the current and the previous)
                                                          // up to 512 (although that is pretty useless)
double voltage_value_Pressure_4, newton_value_Pressure_4; // Converted to Voltage

String pressureID = "discopressure-5988";

double convert_to_newtons(double voltage)
{
  /* General fitting model Exp2:
     f(x) = a*exp(b*x) + c*exp(d*x)
     Coefficients (with 95% confidence bounds):
       a =     0.01419  (0.01163, 0.01676)
       b =      0.9523  (0.8922, 1.012)
       c =    -0.01461  (-0.02317, -0.006043)
       d =      -2.231  (-6.605, 2.142)
       Goodness of fit:
       SSE: 7.906e-06
       R-square: 0.9999
       Adjusted R-square: 0.9997
       RMSE: 0.001988
   */
  double a = 0.01419;
  double b = 0.9523;
  double c = -0.01461;
  double d = -2.231;

  return ((a * exp(b * voltage) + c * exp(d * voltage)) * 9.81); // the result of the fit is in KgF to convert to newton we simply
                                                                 // multiply by 9.81, if you want data in KgF, remove the final multiplication!
}

void pressure()
{
  value_Pressure_1 = analogRead(PRESSURE_PIN1); // reading our analog voltage, careful we only have 10 bit resolution so each
                                                // measurement step is only 5V ÷ 1024, so our result will be 0 - 1023

  // if value is within the range of [ previous - σ , previous + σ], ignore it (if value is relatively the same)
  // this will help with having data ocuppy your buffer that is not a significant deviation.
  if (value_Pressure_1 >= (prev_value_Pressure_1 - deviationPressure) && value_Pressure_1 <= (prev_value_Pressure_1 + deviationPressure))
    return;

  voltage_value_Pressure_1 = double((value_Pressure_1 * 5)) / 1023;       // converting to voltage [ 0, 5] v.
  newton_value_Pressure_1 = convert_to_newtons(voltage_value_Pressure_1); // getting actual force value (careful using this, accuracy may not be ideal)
                                                                          // sensitivity after 1Kgf and before 0.06kgf is limited, you can lower the deviation

  prev_value_Pressure_1 = value_Pressure_1;

  prev_value_Pressure_2 = value_Pressure_2; // PRESSURE SENSOR 2

  value_Pressure_2 = analogRead(PRESSURE_PIN2); // reading our analog voltage, careful we only have 10 bit resolution so each
                                                // measurement step is only 5V ÷ 1024, so our result will be 0 - 1023

  // if value is within the range of [ previous - σ , previous + σ], ignore it (if value is relatively the same)
  // this will help with having data ocuppy your buffer that is not a significant deviation.
  if (value_Pressure_2 >= (prev_value_Pressure_2 - deviationPressure) && value_Pressure_2 <= (prev_value_Pressure_2 + deviationPressure))
    return;

  voltage_value_Pressure_2 = double((value_Pressure_2 * 5)) / 1023;       // converting to voltage [ 0, 5] v.
  newton_value_Pressure_2 = convert_to_newtons(voltage_value_Pressure_2); // getting actual force value (careful using this, accuracy may not be ideal)
                                                                          // sensitivity after 1Kgf and before 0.06kgf is limited, you can lower the deviation
                                                                          // for some improvements

  prev_value_Pressure_3 = value_Pressure_3;     // PRESSURE SENSOR 3
  value_Pressure_3 = analogRead(PRESSURE_PIN3); // reading our analog voltage, careful we only have 10 bit resolution so each
                                                // measurement step is only 5V ÷ 1024, so our result will be 0 - 1023

  // if value is within the range of [ previous - σ , previous + σ], ignore it (if value is relatively the same)
  // this will help with having data ocuppy your buffer that is not a significant deviation.
  if (value_Pressure_3 >= (prev_value_Pressure_3 - deviationPressure) && value_Pressure_3 <= (prev_value_Pressure_3 + deviationPressure))
    return;

  voltage_value_Pressure_3 = double((value_Pressure_3 * 5)) / 1023;       // converting to voltage [ 0, 5] v.
  newton_value_Pressure_3 = convert_to_newtons(voltage_value_Pressure_3); // getting actual force value (careful using this, accuracy may not be ideal)
                                                                          // sensitivity after 1Kgf and before 0.06kgf is limited, you can lower the deviation
                                                                          // for some improvements

  prev_value_Pressure_4 = value_Pressure_4;     // PRESSURE SENSOR 4
  value_Pressure_4 = analogRead(PRESSURE_PIN4); // reading our analog voltage, careful we only have 10 bit resolution so each
                                                // measurement step is only 5V ÷ 1024, so our result will be 0 - 1023

  // if value is within the range of [ previous - σ , previous + σ], ignore it (if value is relatively the same)
  // this will help with having data ocuppy your buffer that is not a significant deviation.
  if (value_Pressure_4 >= (prev_value_Pressure_4 - deviationPressure) && value_Pressure_4 <= (prev_value_Pressure_4 + deviationPressure))
    return;

  voltage_value_Pressure_4 = double((value_Pressure_4 * 5)) / 1023;       // converting to voltage [ 0, 5] v.
  newton_value_Pressure_4 = convert_to_newtons(voltage_value_Pressure_4); // getting actual force value (careful using this, accuracy may not be ideal)
                                                                          // sensitivity after 1Kgf and before 0.06kgf is limited, you can lower the deviation
                                                                          // for some improvements

  //  Serial.println();
//  Serial.print(pressureID);
  //Serial.print(",");
  Serial.print(newton_value_Pressure_1);
  Serial.print(",");
  Serial.print(newton_value_Pressure_2);
  Serial.print(",");
  Serial.print(newton_value_Pressure_3);
  Serial.print(",");
  Serial.println(newton_value_Pressure_4);
}

/*****************************
      LED and Microphone
******************************
*/

#include <Adafruit_NeoPixel.h> //Library to simplify interacting with the LED strand
#ifdef __AVR__
#include <avr/power.h> //Includes the library for power reduction registers if your chip supports them.
#endif

#define LED_PIN 6    //Pin for the pixel strand. Does not have to be analog.
#define LED_TOTAL 80 //Change this to the number of LEDs in your strand.
#define LED_HALF LED_TOTAL / 2
#define AUDIO_PIN A5 //Pin for the envelope of the sound detector

//////////<Globals>
Adafruit_NeoPixel strand = Adafruit_NeoPixel(LED_TOTAL, LED_PIN, NEO_GRB + NEO_KHZ800); //LED strand objetc

uint16_t gradient = 0; //Used to iterate and loop through each color palette gradually

uint8_t volume = 0; //Holds the volume level read from the sound detector.
uint8_t last = 0;   //Holds the value of volume from the previous loop() pass.

float maxVol = 15; //Holds the largest volume recorded thus far to proportionally adjust the visual's responsiveness.
float avgVol = 0;  //Holds the "average" volume-level to proportionally adjust the visual experience.
float avgBump = 0; //Holds the "average" volume-change to trigger a "bump."

bool bump = false; //Used to pass if there was a "bump" in volume

//////////<Helper Functions>

uint8_t split(uint32_t color, uint8_t i)
{

  //0 = Red, 1 = Green, 2 = Blue

  if (i == 0)
    return color >> 16;
  if (i == 1)
    return color >> 8;
  if (i == 2)
    return color >> 0;
  return -1;
}
//Fades lights by multiplying them by a value between 0 and 1 each pass of loop().
void fade(float damper)
{

  //"damper" must be between 0 and 1, or else you'll end up brightening the lights or doing nothing.
  if (damper >= 1)
    damper = 0.99;

  for (int i = 0; i < strand.numPixels(); i++)
  {

    //Retrieve the color at the current position.
    uint32_t col = (strand.getPixelColor(i)) ? strand.getPixelColor(i) : strand.Color(0, 0, 0);

    //If it's black, you can't fade that any further.
    if (col == 0)
      continue;

    float colors[3]; //Array of the three RGB values

    //Multiply each value by "damper"
    for (int j = 0; j < 3; j++)
      colors[j] = split(col, j) * damper;

    //Set the dampened colors back to their spot.
    strand.setPixelColor(i, strand.Color(colors[0], colors[1], colors[2]));
  }
}
uint32_t Rainbow(unsigned int i)
{
  if (i > 1529)
    return Rainbow(i % 1530);
  if (i > 1274)
    return strand.Color(255, 0, 255 - (i % 255)); //violet -> red
  if (i > 1019)
    return strand.Color((i % 255), 0, 255); //blue -> violet
  if (i > 764)
    return strand.Color(0, 255 - (i % 255), 255); //aqua -> blue
  if (i > 509)
    return strand.Color(0, 255, (i % 255)); //green -> aqua
  if (i > 255)
    return strand.Color(255 - (i % 255), 255, 0); //yellow -> green
  return strand.Color(255, i, 0);                 //red -> yellow
}
//PULSE
//Pulse from center of the strand
void Pulse()
{

  fade(0.75); //Listed below, this function simply dims the colors a little bit each pass of loop()

  //Advances the gradient to the next noticeable color if there is a "bump"
  if (bump)
    gradient += 64;

  //If it's silent, we want the fade effect to take over, hence this if-statement
  if (volume > 0)
  {
    uint32_t col = Rainbow(gradient); //Our retrieved 32-bit color

    //These variables determine where to start and end the pulse since it starts from the middle of the strand.
    //  The quantities are stored in variables so they only have to be computed once.
    int start = LED_HALF - (LED_HALF * (volume / maxVol));
    int finish = LED_HALF + (LED_HALF * (volume / maxVol)) + strand.numPixels() % 2;
    //Listed above, LED_HALF is simply half the number of LEDs on your strand. ↑ this part adjusts for an odd quantity.

    for (int i = start; i < finish; i++)
    {

      //"damp" creates the fade effect of being dimmer the farther the pixel is from the center of the strand.
      //  It returns a value between 0 and 1 that peaks at 1 at the center of the strand and 0 at the ends.
      float damp = float(
                       ((finish - start) / 2.0) -
                       abs((i - start) - ((finish - start) / 2.0))) /
                   float((finish - start) / 2.0);

      //Sets the each pixel on the strand to the appropriate color and intensity
      //  strand.Color() takes 3 values between 0 & 255, and returns a 32-bit integer.
      //  Notice "knob" affecting the brightness, as in the rest of the visuals.
      //  Also notice split() being used to get the red, green, and blue values.
      strand.setPixelColor(i, strand.Color(
                                  split(col, 0) * pow(damp, 2.0),
                                  split(col, 1) * pow(damp, 2.0),
                                  split(col, 2) * pow(damp, 2.0)));
    }
    //Sets the max brightness of all LEDs. If it's loud, it's brighter.
    //  "knob" was not used here because it occasionally caused minor errors in color display.
    strand.setBrightness(255.0 * pow(volume / maxVol, 2));
  }

  //This command actually shows the lights. If you make a new visualization, don't forget this!
  strand.show();
}

//The gradient returns a different, changing color for each multiple of 255
//  This is because the max value of any of the 3 LEDs is 255, so it's
//  an intuitive cutoff for the next color to start appearing.
//  Gradients should also loop back to their starting color so there's no jumps in color.

void LEDstrip()
{                                   //This is where the magic happens. This loop produces each frame of the visual.
  volume = analogRead(AUDIO_PIN);   //Record the volume level from the sound detector
  avgVol = (avgVol + volume) / 2.0; //Take our "average" of volumes.

  //Sets a threshold for volume.
  //  In practice I've found noise can get up to 15, so if it's lower, the visual thinks it's silent.
  //  Also if the volume is less than average volume / 2 (essentially an average with 0), it's considered silent.
  if (volume < avgVol / 2.0 || volume < 15)
    volume = 0;

  //If the current volume is larger than the loudest value recorded, overwrite
  if (volume > maxVol)
    maxVol = volume;

  //This is where "gradient" is reset to prevent overflow.
  if (gradient > 1529)
  {

    gradient %= 1530;

    //Everytime a palette gets completed is a good time to readjust "maxVol," just in case
    //  the song gets quieter; we also don't want to lose brightness intensity permanently
    //  because of one stray loud sound.
    maxVol = (maxVol + volume) / 2.0;
  }

  //If there is a decent change in volume since the last pass, average it into "avgBump"
  if (volume - last > avgVol - last && avgVol - last > 0)
    avgBump = (avgBump + (volume - last)) / 2.0;

  //if there is a notable change in volume, trigger a "bump"
  bump = (volume - last) > avgBump;

  Pulse(); //Calls the visual to be displayed with the globals as they are.

  gradient++; //Increments gradient

  last = volume; //Records current volume for next pass

  delay(30); //Paces visuals so they aren't too fast to be enjoyable
}

//////////</Standard Functions>

//This function simply take a value and returns a gradient color
//  in the form of an unsigned 32-bit integer

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  if (!apds.begin())
  { // Begining the work period of the sensor
    //  Serial.println("Failed to initialize Sensor! Please check your wiring.");
  }
  else
    //    Serial.println("Gesture Sensor initialized!");

    //gesture mode will be entered once proximity mode senses something close
    apds.enableProximity(true); // Enabling proximity detection
  apds.enableGesture(true);     // Enabling Gesture detection

  //  Serial.println("Pressure sensors begin here!");
  pinMode(PRESSURE_PIN1, INPUT); // setting pinmode to read analog value
  pinMode(PRESSURE_PIN2, INPUT);
  pinMode(PRESSURE_PIN3, INPUT);
  pinMode(PRESSURE_PIN4, INPUT);
  // distance sensor_t  pinMode(IR_PIN, INPUT);                // setting pinmode to read analog value

  //  Serial.println("Distance sensor!");
  pinMode(IR_PIN, INPUT); // setting pinmode to read analog value

  deviation = 10; // since there's a bit of a drift in the values if you put the same object over a certain period

  // we ignore a divergence of around 1 percent around the previous value.

  //Led and Microphone set up

  Serial.begin(9600); //Sets data rate for serial data transmission.

  strand.begin(); //Initialize the LED strand object.
  strand.show();  //Show a blank strand, just to get the LED's ready for use.
  delay(100);
}

void loop()
{
  //  gestureSensor();
  //  delay(50);
  //  distance();
  //  delay(50);
  pressure();
  delay(100);

//  LEDstrip();
//  delay(50);
}
