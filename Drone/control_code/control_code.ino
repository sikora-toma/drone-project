/*
 * Code for drone stabilization using:
 * MPU-6050 6-DOF gyro and accelerometer used for stabilization
 * FLYSKY FS-i6 controller integration with Arduino Uno, 4 out of 6 channels used
 * ESC BLDC control using Arduino and the Servo.h library
 */

// FastRcReader.h library uses PIN change interrupts to read value changes more precisely
// Download link: https://github.com/timoxd7/FastRCReader
#include <FastRCReader.h>

// Servo library offers an interface for servo motor control
// We will use the writeMicroseconds for precise BLDC controll
// Library link: https://www.arduino.cc/reference/en/libraries/servo/
#include <Servo.h>

// Wire library offers an interface for serial reading of data given by the MPU-6050
// The library uses pins A5 and A4 as SCL and SDA connections by default
// Library link: https://www.arduino.cc/en/Reference/Wire
#include <Wire.h>

// Arduino groups pins together into interrupt groups
// Ports to use tells Arduino which group of pins you are going to use for the interrupts
// PORTS_TO_USE:
//  3 for pins D0-D7
#define PORTS_TO_USE 3
// channel index connected to the throttle on the controller, throttle min and max val
#define THROTTLE_INDEX 4
#define THROTTLE_MIN 1100
#define THROTTLE_MAX 1500

// define the pin to which the ESC signal is connected
#define ESC4_PIN 6
#define ESC1_PIN 9
#define ESC2_PIN 10
#define ESC3_PIN 11

// define the length of gyro calibration loop; CALIBRATION_LOOP_LEN * 3 / 1000 seconds
#define CALIBRATION_LOOP_LEN 2000

// define the number of input channels you are going to use
const byte numInputChannels = 6;

int channelVals[numInputChannels] = {0,0,0,0,0,0};
const uint8_t channelPins[numInputChannels] = {2,3,4,5,6,7};
uint8_t* channelPinPointer = channelPins;

// create a global FastRCReader instance to communicate with the controller
FastRCReader RC;

// variables used for imu signal reading and processing
long gyro_x_cal=0, gyro_y_cal=0, gyro_z_cal=0;
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z;
int temperature;
float acc_total_vector;
float angle_pitch, angle_pitch_acc;
float angle_pitch_output;
float angle_roll, angle_roll_acc;
float angle_roll_output;
boolean mpu_set = false;

// pid calculation variables
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

// create global Servo instances
Servo ESC1, ESC2, ESC3, ESC4;
// esc controll variables
int esc1, es2, esc3, esc4;

unsigned int temp_counter=0;


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
  ESC1.attach(ESC_PIN1);
  ESC1.writeMicroseconds(1000);
  ESC2.attach(ESC_PIN2);
  ESC2.writeMicroseconds(1000);
  ESC3.attach(ESC_PIN3);
  ESC3.writeMicroseconds(1000);
  ESC4.attach(ESC_PIN4);
  ESC4.writeMicroseconds(1000);
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(2000);
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)

  Wire.begin();
  setup_mpu_6050_registers();
  calibrate_gyro();

  temp_counter = micros();
}

void loop() {
  // iterate throught all the channels, read the frequency and print it to the Serial monitor and plotter
  for(int i = 0; i<numInputChannels; i++){
    channelVals[i] = map(RC.getFreq((uint8_t)i), 1000, 2000, THROTTLE_MIN, THROTTLE_MAX);
    Serial.print(channelVals[i]);
    if(i < numInputChannels-1)  Serial.print(",");
  }
  Serial.println();
  // set the throttle values to the desired value
  if(channelVals[THROTTLE_INDEX]<THROTTLE_MIN)       ESC.writeMicroseconds(THROTTLE_MIN);
  else if(channelVals[THROTTLE_INDEX]>THROTTLE_MAX)  ESC.writeMicroseconds(THROTTLE_MAX);
  else                          ESC.writeMicroseconds((int)channelVals[THROTTLE_INDEX]);

  // extract pitch and roll values
  read_mpu_values();
  calculate_pitch_and_roll();

  // PID calculation
  // set throttle for each motor
  /*
  if(esc1<THROTTLE_MIN)       ESC1.writeMicroseconds(THROTTLE_MIN);
  else if(esc1>THROTTLE_MAX)  ESC1.writeMicroseconds(THROTTLE_MAX);
  else                          ESC1.writeMicroseconds(esc1);
  if(esc2<THROTTLE_MIN)       ESC2.writeMicroseconds(THROTTLE_MIN);
  else if(esc2>THROTTLE_MAX)  ESC2.writeMicroseconds(THROTTLE_MAX);
  else                          ESC2.writeMicroseconds(esc2);
  if(esc3<THROTTLE_MIN)       ESC3.writeMicroseconds(THROTTLE_MIN);
  else if(esc3>THROTTLE_MAX)  ESC3.writeMicroseconds(THROTTLE_MAX);
  else                          ESC3.writeMicroseconds(esc3);
  if(esc4<THROTTLE_MIN)       ESC4.writeMicroseconds(THROTTLE_MIN);
  else if(esc4>THROTTLE_MAX)  ESC4.writeMicroseconds(THROTTLE_MAX);
  else                          ESC4.writeMicroseconds(esc4);
   */
  
  while(temp_counter-micros()<4000);
  temp_counter = micros();
}


//function extracts the pitch and roll value from the gyro and accelerometer readings
void calculate_pitch_and_roll(){
  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value

  //Gyro angle calculations
  //0.000061 = 1 / 250 / 131
  angle_pitch += gyro_x * 0.0000305;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000305;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_x/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_pitch_acc -= 1.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the roll angle
  angle_roll_acc -= 1.0;                                              //Accelerometer calibration value for roll
  if(mpu_set){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;     //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll= angle_roll_acc;                                     //Set the gyro roll angle equal to the accelerometer roll angle 
    mpu_set = true;                                            //Set the IMU started flag
  }
  angle_pitch_output = angle_pitch;
  angle_roll_output = angle_roll;
  Serial.print(angle_pitch_output);
  //Serial.print((String)","+angle_pitch_acc);
  Serial.print((String)","+angle_roll_output);
  //Serial.print((String)","+angle_roll_acc);
  Serial.println();  
}
// function performs the initial gyro calibration
void calibrate_gyro(){
  //Serial.println("Calibrating gyro");
  for(int i=0; i<CALIBRATION_LOOP_LEN; i++){
    //if(i%250==0) Serial.println((String)"x="+gyro_x_cal+" y="+gyro_y_cal+" z="+gyro_z_cal);
    read_mpu_values();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;
    delay(3);
  }
  gyro_x_cal/=2000;
  gyro_y_cal/=2000;
  gyro_z_cal/=2000;
}
// function reads the gyro, accelerometer, and temperature values from the imu and stores them
void read_mpu_values(){
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14); 

  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable
}
// function sets up the work modes of the mpu-6050
void setup_mpu_6050_registers(){
  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(0x68);                                     //Start communication with the address found during search.
  Wire.write(0x6B);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                   //End the transmission with the gyro.
  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(0x68);                                     //Start communication with the address found during search.
  Wire.write(0x1B);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(0x68);                                     //Start communication with the address found during search.
  Wire.write(0x1C);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(0x68);                                     //Start communication with the address found during search
  Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                   //End the transmission with the gyro 
}
