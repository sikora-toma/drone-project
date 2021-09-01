/*
 * ESC BLDC control using Arduino and the Servo.h library
 */

// Servo library offers an interface for servo motor control
// We will use the writeMicroseconds for precise BLDC controll
// Library link: https://www.arduino.cc/reference/en/libraries/servo/writemicroseconds/
#include <Servo.h>

// define the pin to which the ESC signal is connected
#define ESC1_PIN 5
#define ESC2_PIN 6
#define ESC3_PIN 9
#define ESC4_PIN 10

// create a global Servo instance
Servo ESC1, ESC2, ESC3, ESC4;

void setup() {
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

void loop() {
  // set the BLDC motor speed
  ESC1.writeMicroseconds(1400);
  ESC2.writeMicroseconds(1400);
  ESC3.writeMicroseconds(1400);
  ESC4.writeMicroseconds(1400);
}
