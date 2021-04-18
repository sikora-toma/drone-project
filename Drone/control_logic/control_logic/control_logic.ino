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


  
  if (start == 2){                                                          //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    } 

    if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  }

  else{
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }

}
