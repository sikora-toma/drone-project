/*
 * FLYSKY FS-i6 controller integration with Arduino Uno and
 * ESC BLDC control using Arduino and the Servo.h library
 * 4 out of 6 channels used
 */

// FastRcReader.h library uses PIN change interrupts to read value changes more precisely
// Download link: https://github.com/timoxd7/FastRCReader
#include <FastRCReader.h>

// Servo library offers an interface for servo motor control
// We will use the writeMicroseconds for precise BLDC controll
// Library link: https://www.arduino.cc/reference/en/libraries/servo/writemicroseconds/
#include <Servo.h>

// Arduino groups pins together into interrupt groups
// Ports to use tells Arduino which group of pins you are going to use for the interrupts
// PORTS_TO_USE:
//  1 for pins D8-D13
//  2 for pins A0-A5
//  3 for pins D0-D7
#define PORTS_TO_USE 3
// pin connected to the throttle on the controller, throttle min and max val
#define THROTTLE_INDEX 4
#define THROTTLE_MIN 1100
#define THROTTLE_MAX 1500

// define the pin to which the ESC signal is connected
#define ESC_PIN 9

// define the number of input channels you are going to use
const byte numInputChannels = 6;

const uint8_t channelPins[numInputChannels] = {2,3,4,5,6,7};
uint8_t* channelPinPointer = channelPins;

// create a global FastRCReader instance to communicate with the controller
FastRCReader RC;

// create a global Servo instance
Servo ESC;

void setup() {
  // initialize the serial connection to print values
  Serial.begin(9600);
  // initialize the RC receiver connection
  RC.begin();
  // set the channel pins in use
  //RC.addChannel(channelPinPointer);
  RC.addChannel((uint8_t)2);
  RC.addChannel((uint8_t)3);
  RC.addChannel((uint8_t)4);
  RC.addChannel((uint8_t)5);
  RC.addChannel((uint8_t)6);
  RC.addChannel((uint8_t)7);


  pinMode(LED_BUILTIN, OUTPUT);

  // initialize the serial connection to print values
  Serial.begin(9600);
  ESC.attach(ESC_PIN);
  ESC.writeMicroseconds(1000);
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(2000);
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)

}

int channelVals[numInputChannels] = {0,0,0,0,0,0};

void loop() {
  // iterate throught all the channels, read the frequency and print it to the Serial monitor and plotter
  for(int i = 0; i<numInputChannels; i++){
    channelVals[i] = map(RC.getFreq((uint8_t)i), 1000, 2000, THROTTLE_MIN, THROTTLE_MAX);
    Serial.print(channelVals[i]);
    if(i < numInputChannels-1)  Serial.print(",");
  }
  Serial.println();
  // set the throttle to the desired value
  if(channelVals[THROTTLE_INDEX]<THROTTLE_MIN)       ESC.writeMicroseconds(THROTTLE_MIN);
  else if(channelVals[THROTTLE_INDEX]>THROTTLE_MAX)  ESC.writeMicroseconds(THROTTLE_MAX);
  else                          ESC.writeMicroseconds((int)channelVals[THROTTLE_INDEX]);
}
