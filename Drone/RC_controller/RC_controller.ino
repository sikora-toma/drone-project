/*
 * FLYSKY FS-i6 controller integration with Arduino Uno
 * 4 out of 6 channels used
 */

// FastRcReader.h library uses PIN change interrupts to read value changes more precisely
// Download link: https://github.com/timoxd7/FastRCReader
#include <FastRCReader.h>

// Arduino groups pins together into interrupt groups
// Ports to use tells Arduino which group of pins you are going to use for the interrupts
// PORTS_TO_USE:
//  1 for pins D8-D13
//  2 for pins A0-A5
//  3 for pins D0-D7
#define PORTS_TO_USE 3
#define NUMBER_OF_PORTS 6

// define the number of input channels you are going to use
const byte numInputChannels = NUMBER_OF_PORTS;

const uint8_t channelPins[numInputChannels] = {0,1,2,3,4,5};
uint8_t* channelPinPointer = channelPins;

// create a global FastRCReader instance to communicate with the controller
FastRCReader RC;

void setup() {
  // initialize the serial connection to print values
  Serial.begin(9600);
  // initialize the RC receiver connection
  RC.begin();
  // set the channel pins in use
  //RC.addChannel(channelPinPointer);
  RC.addChannel((uint8_t)0);
  RC.addChannel((uint8_t)1);
  RC.addChannel((uint8_t)2);
  RC.addChannel((uint8_t)3);
  RC.addChannel((uint8_t)4);
  RC.addChannel((uint8_t)5);
}

void loop() {
  // iterate throught all the channels, read the frequency and print it to the Serial monitor and plotter
  for(int i = 0; i<numInputChannels; i++){
    //Serial.print(i);
    //Serial.print(": ");
    Serial.print(RC.getFreq((uint8_t)i));
    if(i < numInputChannels-1)  Serial.print(",");
  }
  Serial.println();
}
