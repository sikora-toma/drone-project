/*
 * ESC BLDC control using Arduino and the Servo.h library
 */

// Servo library offers an interface for servo motor control
// We will use the writeMicroseconds for precise BLDC controll
// Library link: https://www.arduino.cc/reference/en/libraries/servo/writemicroseconds/
#include <Servo.h>

// define the pin to which the ESC signal is connected
#define ESC_PIN 9

// create a global Servo instance
Servo ESC;

int a = 3000;
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

  // initialize the serial connection to print values
  Serial.begin(9600);
  ESC.attach(9);
  ESC.writeMicroseconds(1000);
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(2000);
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
}

void loop() {
  // iterate throught all the channels, read the frequency and print it to the Serial monitor and plotter
  ESC.writeMicroseconds(2000);
}
