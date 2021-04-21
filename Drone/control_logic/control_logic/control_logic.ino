/*
 * This code serves as a mockup for developing the control logic
 * for a drone based on Arduino. When ready, the code will be transfered to the control_code. 
 */

float pitch_goal, roll_goal, yaw_goal;
float pitch_input, roll_input, yaw_input;
int throttle_input;

// setup the gains for all PID values for each control variable
float pid_pitch_p_gain, pid_pitch_i_gain, pid_pitch_d_gain;
float pid_roll_p_gain, pid_roll_i_gain, pid_roll_d_gain;
float pid_yaw_p_gain, pid_yaw_i_gain, pid_yaw_d_gain;
// setup the PID output variables
float pid_pitch_output, pid_roll_output, pid_yaw_output;
// setup max values for PID output
float pid_pitch_max, pid_roll_max, pid_yaw_max;
// setup integral and derivative auxiliary variables
float pid_pitch_i_mem, pid_roll_i_mem, pid_yaw_i_mem, pid_pitch_d_last, pid__pitch_last_error, pid__roll_last_error, pid__yaw_last_error;
float pid_pitch_error, pid_roll_error, pid_yaw_error;
// setup the motor output variables
int motor1_output, motor2_output, motor3_output, motor4_output;

void setup() {

}

void loop() {
	// calculate the difference between the desired and current values
	// calculate the PID values for each motor

    // P calculation
  pid_pitch_error = pitch_input - pitch_goal;
  pid_roll_error = roll_input - roll_goal;
  pid_yaw_error = yaw_input - yaw_goal;
  
    // I calculation
  pid_pitch_i_mem += pid_i_gain_pitch * pid_error_temp;
  if(pid_pitch_i_mem > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_pitch_i_mem < pid_max_pitch * -1)pid_pitch_i_mem = pid_max_pitch * -1;

  pid_roll_i_mem += pid_i_gain_pitch * pid_error_temp;
  if(pid_roll_i_mem > pid_max_pitch)pid_roll_i_mem = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_roll_i_mem = pid_max_pitch * -1;
  
  pid_yaw_i_mem += pid_yaw_i_mem * pid_error_temp;
  if(pid_yaw_i_mem > pid_max_pitch)pid_yaw_i_mem = pid_max_pitch;
  else if(pid_yaw_i_mem < pid_max_pitch * -1)pid_yaw_i_mem = pid_max_pitch * -1;

  // full PID calculation
  pid_pitch_output =  pid_pitch_gain * pid_pitch_error + pid_pitch_gain * pid_pitch_i_mem + pid_pitch_d_gain * (pid_pitch_error-pid_pitch_last_error);
  pid_roll_output =   pid_roll_gain * pid_roll_error   + pid_roll_gain * pid_roll_i_mem   + pid_roll_d_gain * (pid_roll_error-pid_roll_last_error);
  pid_yaw_output =    pid_yaw_gain * pid_yaw_error     + pid_yaw_gain * pid_yaw_i_mem     + pid_yaw_d_gain * (pid_yaw_error-pid_yaw_last_error);


	// calculate the output for each motor
	motor1_output = throttle_input;
	motor2_output = throttle_input;
	motor3_output = throttle_input;
	motor4_output = throttle_input;
  // left front
  if(<THROTTLE_MIN) motor1_output = THROTTLE_MIN;
  if(>THROTTLE_MAX) motor1_output = THROTTLE_MAX;
  else motor1_output = 
  // right front
  if(<THROTTLE_MIN) motor2_output = THROTTLE_MIN;
  if(>THROTTLE_MAX) motor2_output = THROTTLE_MAX;
  else motor2_output = 
  // right back
  if(<THROTTLE_MIN) motor3_output = THROTTLE_MIN;
  if(>THROTTLE_MAX) motor3_output = THROTTLE_MAX;
  else motor3_output = 
  // left back
  if(<THROTTLE_MIN) motor4_output = THROTTLE_MIN;
  if(>THROTTLE_MAX) motor4_output = THROTTLE_MAX;
  else motor4_output = 
  
  // D step  
  pid_pitch_last_error = pid_pitch_error;
  pid_roll_last_error = pid_roll_error;
  pid__yaw_last_error = pid__yaw_error;


  
  if (start == 2){                                                          //The motors are started.
    if (throttle_input > THROTTLE_MAX_CONTROL) throttle_input = THROTTLE_MAX_CONTROL;                                   //We need some room to keep full control at full throttle.
    motor1_output = throttle_input - pid_pitch_output + pid_roll_output - pid_yaw_output; //Calculate the pulse for esc 1 (front-right - CCW)
    motor2_output = throttle_input + pid_pitch_output + pid_roll_output + pid_yaw_output; //Calculate the pulse for esc 2 (rear-right - CW)
    motor3_output = throttle_input + pid_pitch_output - pid_roll_output - pid_yaw_output; //Calculate the pulse for esc 3 (rear-left - CCW)
    motor4_output = throttle_input - pid_pitch_output - pid_roll_output + pid_yaw_output; //Calculate the pulse for esc 4 (front-left - CW)

    if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
       motor1_output += motor1_output * ((1240 - battery_voltage) / (float)3500);              //Compensate the esc-2 pulse for voltage drop.
       motor2_output += motor2_output * ((1240 - battery_voltage) / (float)3500);              //Compensate the esc-1 pulse for voltage drop.
       motor3_output += motor3_output * ((1240 - battery_voltage) / (float)3500);              //Compensate the esc-3 pulse for voltage drop.
       motor4_output += motor4_output * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    } 

    if (motor1_output < 1100) motor1_output = 1100;                                         //Keep the motors running.
    if (motor2_output < 1100) motor2_output = 1100;                                         //Keep the motors running.
    if (motor3_output < 1100) motor3_output = 1100;                                         //Keep the motors running.
    if (motor4_output < 1100) motor4_output = 1100;                                         //Keep the motors running.

    if(motor1_output > 2000) motor1_output = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if (motor2_output > 2000)motor2_output = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if (motor3_output > 2000)motor3_output = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if (motor4_output > 2000)motor4_output = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  }

  else{
      motor1_output = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
      motor2_output = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
      motor3_output = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
      motor4_output = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }

}
