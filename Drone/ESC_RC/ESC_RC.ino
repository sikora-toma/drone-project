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
#define THROTTLE_INDEX 3
#define THROTTLE_MIN 1100
#define THROTTLE_MAX 1500

#define RECIEVER_MIN 1000
#define RECIEVER_MAX 2000

// define indexes controlling the drone motion
#define PITCH_INDEX 4
#define ROLL_INDEX 5
#define YAW_INDEX 2

// define the pin to which the ESC signal is connected
#define ESC1_PIN 6
#define ESC2_PIN 9
#define ESC3_PIN 10
#define ESC4_PIN 11

// define the number of input channels you are going to use
const byte numInputChannels = 6;

const uint8_t channelPins[numInputChannels] = {0,1,2,3,4,5};
uint8_t* channelPinPointer = channelPins;

// create a global FastRCReader instance to communicate with the controller
FastRCReader RC;

// create a global Servo instance
Servo ESC1, ESC2, ESC3, ESC4;

// throttle values
int throttle1, throttle2, throttle3, throttle4;

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


  pinMode(LED_BUILTIN, OUTPUT);

  // initialize the serial connection to print values
  Serial.begin(9600);
  ESC1.attach(ESC1_PIN);
  ESC2.attach(ESC2_PIN);
  ESC3.attach(ESC3_PIN);
  ESC4.attach(ESC4_PIN);
  ESC1.writeMicroseconds(1000);
  ESC2.writeMicroseconds(1000);
  ESC3.writeMicroseconds(1000);
  ESC4.writeMicroseconds(1000);
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(2000);
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)

}

int channelVals[numInputChannels] = {0,0,0,0,0,0};
int it = 0;
void loop() {
  // iterate throught all the channels, read the frequency and print it to the Serial monitor and plotter
  for(int i = 0; i<numInputChannels; i++){
    channelVals[i] = RC.getFreq((uint8_t)i);
    //Serial.print(channelVals[i]);
    //if(i < numInputChannels-1)  Serial.print(",");
  }
  //Serial.println();

  // calculate the desired throttle values for all motors
  throttle1 = throttle2 = throttle3 = throttle4 = map((int)channelVals[THROTTLE_INDEX], RECIEVER_MIN, RECIEVER_MAX, THROTTLE_MIN, THROTTLE_MAX);

  throttle1 -= (int)channelVals[PITCH_INDEX] - (RECIEVER_MAX+RECIEVER_MIN)/2; throttle2 -= (int)channelVals[PITCH_INDEX] - (RECIEVER_MAX+RECIEVER_MIN)/2;
  throttle3 += (int)channelVals[PITCH_INDEX] - (RECIEVER_MAX+RECIEVER_MIN)/2; throttle4 += (int)channelVals[PITCH_INDEX] - (RECIEVER_MAX+RECIEVER_MIN)/2;

  throttle2 -= (int)channelVals[ROLL_INDEX] - (RECIEVER_MAX+RECIEVER_MIN)/2; throttle4 -= (int)channelVals[ROLL_INDEX] - (RECIEVER_MAX+RECIEVER_MIN)/2;
  throttle1 += (int)channelVals[ROLL_INDEX] - (RECIEVER_MAX+RECIEVER_MIN)/2; throttle3 += (int)channelVals[ROLL_INDEX] - (RECIEVER_MAX+RECIEVER_MIN)/2;

  throttle1 += (int)channelVals[YAW_INDEX] - (RECIEVER_MAX+RECIEVER_MIN)/2; throttle4 += (int)channelVals[YAW_INDEX] - (RECIEVER_MAX+RECIEVER_MIN)/2;
  throttle2 -= (int)channelVals[YAW_INDEX] - (RECIEVER_MAX+RECIEVER_MIN)/2; throttle3 -= (int)channelVals[YAW_INDEX] - (RECIEVER_MAX+RECIEVER_MIN)/2;
// debugging rc control
if(it++%10000==0){
  Serial.print((int)channelVals[PITCH_INDEX]);
  Serial.print(",   ");
  Serial.print(throttle1);
  Serial.print(",");
  Serial.print(throttle2);
  Serial.print(",");
  Serial.print(throttle3);
  Serial.print(",");
  Serial.print(throttle4);
  Serial.println();}
  // set the throttle to the desired value
  if(throttle1<THROTTLE_MIN)       ESC1.writeMicroseconds(THROTTLE_MIN);
  else if(throttle1>THROTTLE_MAX)  ESC1.writeMicroseconds(THROTTLE_MAX);
  else                          ESC1.writeMicroseconds(throttle1);
  if(throttle2<THROTTLE_MIN)       ESC2.writeMicroseconds(THROTTLE_MIN);
  else if(throttle2>THROTTLE_MAX)  ESC2.writeMicroseconds(THROTTLE_MAX);
  else                          ESC2.writeMicroseconds(throttle2);
  if(throttle3<THROTTLE_MIN)       ESC3.writeMicroseconds(THROTTLE_MIN);
  else if(throttle3>THROTTLE_MAX)  ESC3.writeMicroseconds(THROTTLE_MAX);
  else                          ESC3.writeMicroseconds(throttle3);
  if(throttle4<THROTTLE_MIN)       ESC4.writeMicroseconds(THROTTLE_MIN);
  else if(throttle4>THROTTLE_MAX)  ESC4.writeMicroseconds(THROTTLE_MAX);
  else                          ESC4.writeMicroseconds(throttle4);

}
