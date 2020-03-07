//============================================================================
// Name       : scanner_controller.h
// Author     : Ali Murtatha Shuman
// Version    : V1.0
// Copyright  : For any questions contact Ali Murtatha Shuman
//				Email: ali.m.shuman@gmail.com
// 				Tel.nr: +46 76 26 55 339
//
// Description: Updating the PID-controllers of the	x,y-translation and the theta angle
//				and keeping track of the translation distance to the target position.
//
//				This node is subscribing to the following topics:
//				"docking_pose" 	: geometry_msgs::Point::ConstPtr&
//									: target position from the scanner
//				"start_dock"	: std_msgs::Bool
//									: for signaling to start the docking/backing
//
//				and publishing the following:
//				"cmd_vel"		: geometry_msgs::Twist
//				"docking_state"	: std_msgs::UInt8
//
//				NOTE: The X-axis equivalent to zero degrees, which is the
// 				front direction of the platform. and Y-axis is orthogonal to
//				the X-axis at the left of the platform.
//
//				NOTE: All parameters used in this program can be modified by
//				changing the config/controller_config.yaml file or by launching
//				the node with the argument: config_file:=new_config_file.yaml.
//============================================================================

#ifndef SCANNER_CONTROLLER_H_
#define SCANNER_CONTROLLER_H_

#include <iostream>
#include <vector> 	// vector
#include <math.h>	// sqrt

//ROS
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"

#include "pid_controller.h"

using namespace std;

#define WAIT_TO_DOCK 0
#define DOCKING 1
#define DOCKING_ENDED 2
#define BACKING 3

#define TARGET_DETCTED true
#define UNDETCTED_TARGET false

#define NO_TASK 0
#define DOCKING_TASK 1
#define BACKING_TASK 2

// --------------- Param class deceleration ---------------
class Param{
public:
	double SCANNER_TO_CENTER_POS;
	double PLATFORM_LENGTH;
	double BACKING_DISTANCE;

	double ACCEPTED_X_ERROR;
	double ACCEPTED_Y_ERROR;
	double ACCEPTED_THETA_ERROR;

	double MAX_LINER_SPEED;
	double MIN_LINER_SPEED;
	double MAX_ANGULAR_SPEED;
	double MIN_ANGULAR_SPEED;

	double TIME_DURATION;

	double X_KP;
	double X_KD;
	double X_KI;

	double Y_KP;
	double Y_KD;
	double Y_KI;

	double THETA_KP;
	double THETA_KD;
	double THETA_KI;

	void read_param();
};//End of class Param

// --------------- Controller class ---------------
class Controller{
private:
	// output data
	geometry_msgs::Twist cmd_vel;

	geometry_msgs::Point current_pos;
	geometry_msgs::Point target_pos;

	std_msgs::UInt8 state;
	bool detection_state;
	int task;

	Pid pid_x;
	Pid pid_y;
	Pid pid_theta;


	void pridect_pos();
	void reset_current_pos();
	void update_pid_current_value();
	void update_pid_target_value();
	void reset_pid();
	void check_control_cmd_limit();
	void calc_target_pos(const geometry_msgs::Point::ConstPtr&);
	void calculate_cmd_vel_data();
	bool dock();
	bool back();

public:
	Param p;

	Controller();

	void toggle_state(const std_msgs::Bool state_toggle);

	void update_navigate_position(const geometry_msgs::Point::ConstPtr&);

	std_msgs::UInt8 get_state();
	bool task_manger();
	geometry_msgs::Twist get_cmd_vel();
};//End of class Controller


// --------------- ControllerSubAndPub class ---------------
class ControllerSubAndPub {
private:
	ros::NodeHandle n_;

	ros::Publisher pub_cmd_vel;
	ros::Publisher pub_docking_state;

	ros::Subscriber sub_docking_pose;
	ros::Subscriber sub_start_dock;

	ros::Timer timer_;

	Controller controller;

public:
	ControllerSubAndPub();
	void callback_navigation_state(const std_msgs::Bool input);
	void callback_navigate_position(const geometry_msgs::Point::ConstPtr&);
	void callback_timer(const ros::TimerEvent&);
};//End of class ControllerSubAndPub

#endif //SCANNER_CONTROLLER_H_
