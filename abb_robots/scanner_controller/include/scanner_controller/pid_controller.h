//============================================================================
// Name       : pid_controller.h
// Author     : Ali Murtatha Shuman
// Version    : V1.0
// Copyright  : For any questions contact Ali Murtatha Shuman
//				Email: ali.m.shuman@gmail.com
// 				Tel.nr: +46 76 26 55 339
//
// Description: PID-controller class.
//============================================================================

#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include <math.h> // fabs

using namespace std;

// --------------- Pid class ---------------
class Pid{
private:
	double max_value, current_value, output_value;
	double kp, kd, ki;
	double target_value, max_integrator, min_integrator;
	double integrator, derivator, error;

public:
	Pid();
	Pid(double max, double Kp, double Kd, double Ki);
	double calculate();
	void update_target_value(double new_value);
	void update_current_value(double new_value);
	void reset();
};//End of class Pid


#endif //PID_CONTROLLER_H_
