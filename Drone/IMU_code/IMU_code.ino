#include <Wire.h>
#define CALIBRATION_LOOP_LEN 2000

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

unsigned int temp_counter=0;

void setup() {
  Serial.begin(9600);
  //Serial.println("Beginning");
  Wire.begin();

  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (250dfs / +/-4g) and start the gyro

  
  // gyro calibration
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

  //Serial.println((String)"Calibrated\nx="+gyro_x_cal+" y="+gyro_y_cal+" z="+gyro_z_cal);

  temp_counter = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  read_mpu_values();
  
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
  // Dampen the pitch angle
  //angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  //if(temp_counter++%250==0){
    //Serial.println();
    //Serial.println((String)"Gyro values:\t\t x="+gyro_x+" y="+gyro_y+" z="+gyro_z);
    //Serial.println((String)"Accelerometer values:\tx="+acc_x+" y="+acc_y+" z="+acc_z);
    //Serial.println((String)"Pitch angle: "+angle_pitch_output);
    Serial.print(angle_pitch_output);
    //Serial.print((String)","+angle_pitch_acc);
    Serial.print((String)","+angle_roll_output);
    //Serial.print((String)","+angle_roll_acc);
    Serial.println();
  //}
  while(temp_counter-micros()<4000);
  temp_counter = micros();
}

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
