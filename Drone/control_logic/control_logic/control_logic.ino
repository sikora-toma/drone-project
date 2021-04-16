/*
 * This code serves as a mockup for developing the control logic
 * for a drone based on Arduino. When ready, the code will be transfered to the control_code. 
 */

float pitch_goal, roll_goal, yaw_goal;
float pitch_input, roll_input, yaw_input, throttle_input;

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

}
