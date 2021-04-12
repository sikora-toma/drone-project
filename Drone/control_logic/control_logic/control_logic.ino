/*
 * This code serves as a mockup for developing the control logic
 * for a drone based on Arduino. When ready, the code will be transfered to the control_code. 
 */

float pitch_goal, roll_goal, yaw_goal;
float pitch_curr, roll_curr, yaw_curr;

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
float pid_pitch_error, pid_roll_error, pid__yaw_last_error;
// setup the motor output variables
int motor1_output, motor2_output, motor3_output, motor4_output;

void setup() {

}

void loop() {
	// calculate the difference between the desired and current values
	// calculate the PID values for each motor

	// calculate the output for each motor
	motor1_output = 0;
	motor2_output = 0;
	motor3_output = 0;
	motor4_output = 0;
}
