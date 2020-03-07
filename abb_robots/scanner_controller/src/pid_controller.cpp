//============================================================================
// Name       : pid_controller.cpp
// Author     : Ali Murtatha Shuman
// Version    : V1.0
// Copyright  : For any questions contact Ali Murtatha Shuman
//				Email: ali.m.shuman@gmail.com
// 				Tel.nr: +46 76 26 55 339
//
// Description: PID-controller class functions deceleration.
//============================================================================


#include "pid_controller.h"

// --------------- Pid class functions deceleration ---------------
Pid::Pid(){
	max_value = 0.0;
	output_value = 0.0;

	current_value = 0.0;
	target_value = 0.0;

	kp = 0.0;
	kd = 0.0;
	ki = 0.0;

	error = 0.0;
	derivator = 0.0;

	integrator = 0.0;
	max_integrator = 0.0;
	min_integrator = 0.0;
}

Pid::Pid(double max, double Kp, double Kd, double Ki){
	max_value = max;
	output_value = 0.0;

	current_value = 0.0;
	target_value = 0.0;

	kp = Kp;
	kd = Kd;
	ki = Ki;

	error = 0.0;
	derivator = 0.0;

	integrator = 0.0;
	max_integrator = max;
	min_integrator = -max;
}

double Pid::calculate(){
	error = target_value - current_value;
	double p_value = kp*error;
	double d_value = kd*(error - derivator);
	derivator = error;

	integrator += error;
	if(integrator > max_integrator){
		integrator = max_integrator;
	}
	else if(integrator < min_integrator){
		integrator = min_integrator;
	}
	double i_value = integrator*ki;

	output_value = p_value + d_value + i_value;
	if(fabs(output_value) > max_value){
		if(output_value > 0){
			output_value = max_value;
		}
		else{
			output_value = -max_value;
		}
	}
	return output_value;
}

void Pid::update_current_value(double new_value){
	current_value = new_value;
}

void Pid::update_target_value(double new_value){
	target_value = new_value;
}

void Pid::reset(){
	error = 0.0;
	derivator = 0.0;

	integrator = 0.0;
}



