//============================================================================
// Name       : scanner_controller.cpp
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

#include "scanner_controller.h"

// --------------- Controller class functions deceleration ---------------
void Param::read_param(){
	ros::NodeHandle node("~");
	node.param("SCANNER_TO_CENTER_POS", SCANNER_TO_CENTER_POS, 0.41);
	node.param("PLATFORM_LENGTH_CENTER_TO_EDGE", PLATFORM_LENGTH, 0.5);
	node.param("BACKING_DISTANCE", BACKING_DISTANCE, 1.0);

	node.param("ACCEPTED_X_ERROR", ACCEPTED_X_ERROR, 0.01);
	node.param("ACCEPTED_Y_ERROR", ACCEPTED_Y_ERROR, 0.01);
	node.param("ACCEPTED_THETA_ERROR", ACCEPTED_THETA_ERROR, 0.035);

	node.param("MAX_LINER_SPEED", MAX_LINER_SPEED, 0.2);
	node.param("MIN_LINER_SPEED", MIN_LINER_SPEED, 0.01);
	node.param("MAX_ANGULAR_SPEED", MAX_ANGULAR_SPEED, 0.6);
	node.param("MIN_ANGULAR_SPEED", MIN_ANGULAR_SPEED, 0.02);

	node.param("TIME_DURATION", TIME_DURATION, 0.1);

	node.param("X_KP", X_KP, 0.2);
	node.param("X_KD", X_KD, 0.1);
	node.param("X_KI", X_KI, 0.01);

	node.param("Y_KP", Y_KP, 1.0);
	node.param("Y_KD", Y_KD, 0.3);
	node.param("Y_KI", Y_KI, 0.01);

	node.param("THETA_KP", THETA_KP, 0.5);
	node.param("THETA_KD", THETA_KD, 0.1);
	node.param("THETA_KI", THETA_KI, 0.01);
}

// --------------- Controller class functions deceleration ---------------

// ------- private -------
void Controller::pridect_pos(){
	current_pos.x += cmd_vel.linear.x*p.TIME_DURATION;
	current_pos.y += cmd_vel.linear.y*p.TIME_DURATION;
	current_pos.z += cmd_vel.angular.z*p.TIME_DURATION;
}

void Controller::reset_current_pos(){
	current_pos.x = 0.0;
	current_pos.y = 0.0;
	current_pos.z = 0.0;
}

void Controller::update_pid_current_value(){
	pid_x.update_current_value(current_pos.x);
	pid_y.update_current_value(current_pos.y);
	pid_theta.update_current_value(current_pos.z);
}

void Controller::update_pid_target_value(){
	pid_x.update_target_value(target_pos.x);
	pid_y.update_target_value(target_pos.y);
	pid_theta.update_target_value(target_pos.z);
}

void Controller::reset_pid(){
	pid_x.reset();
	pid_y.reset();
	pid_theta.reset();
}

void Controller::check_control_cmd_limit(){
	if(fabs(cmd_vel.linear.x) < p.MIN_LINER_SPEED){
		if(cmd_vel.linear.x < 0){
			cmd_vel.linear.x = -p.MIN_LINER_SPEED;
		}
		else{
			cmd_vel.linear.x = p.MIN_LINER_SPEED;
		}
	}
	if(fabs(cmd_vel.linear.y) < p.MIN_LINER_SPEED){
		if(cmd_vel.linear.y < 0){
			cmd_vel.linear.y = -p.MIN_LINER_SPEED;
		}
		else{
			cmd_vel.linear.y = p.MIN_LINER_SPEED;
		}
	}
	if(fabs(cmd_vel.angular.z) < p.MIN_ANGULAR_SPEED){
		if(cmd_vel.angular.z < 0){
			cmd_vel.angular.z = -p.MIN_ANGULAR_SPEED;
		}
		else{
			cmd_vel.angular.z = p.MIN_ANGULAR_SPEED;
		}
	}

	if(fabs(target_pos.x-current_pos.x) < p.ACCEPTED_X_ERROR){
		cmd_vel.linear.x = 0.0;
	}
	if(fabs(target_pos.y-current_pos.y) < p.ACCEPTED_Y_ERROR){
		cmd_vel.linear.y = 0.0;
	}
	if(fabs(target_pos.z-current_pos.z) < p.ACCEPTED_THETA_ERROR){
		cmd_vel.angular.z = 0.0;
	}

	if(cmd_vel.linear.x == 0.0 && cmd_vel.linear.y == 0.0){
		if(state.data == DOCKING){
			state.data = DOCKING_ENDED;
		}
		if(state.data == BACKING){
			state.data = WAIT_TO_DOCK;
		}
		task = NO_TASK;
		detection_state = UNDETCTED_TARGET;
		reset_pid();
	}
}

void Controller::calc_target_pos(const geometry_msgs::Point::ConstPtr& input){
	target_pos.x = input->y + p.SCANNER_TO_CENTER_POS - p.PLATFORM_LENGTH;
	target_pos.y = -input->x;
	target_pos.z = input->z;
}

void Controller::calculate_cmd_vel_data(){
	update_pid_target_value();
	update_pid_current_value();

	cmd_vel.linear.x = pid_x.calculate();
	cmd_vel.linear.y = pid_y.calculate();
	cmd_vel.angular.z = pid_theta.calculate();

	check_control_cmd_limit();
	pridect_pos();
}

bool Controller::dock(){
	if(state.data == WAIT_TO_DOCK && detection_state == TARGET_DETCTED){
		ROS_INFO("Processing the docking task!");
		state.data = DOCKING;
		calculate_cmd_vel_data();
		return true;
	}
	else if(state.data == DOCKING){
		calculate_cmd_vel_data();
		return true;
	}
	ROS_INFO("No target detected!");
	return false;
}

bool Controller::back(){
	if(state.data == DOCKING_ENDED){
		ROS_INFO("Processing the backing task!");
		reset_current_pos();
		target_pos.x = -p.BACKING_DISTANCE;// + current_pos.x;
		target_pos.y = 0.0;
		target_pos.z = 0.0;
		detection_state = TARGET_DETCTED;
		state.data = BACKING;
		calculate_cmd_vel_data();
		return true;
	}else if(state.data == BACKING){
		calculate_cmd_vel_data();
		return true;
	}
	return false;
}

// ------- public -------
Controller::Controller(){
	p.read_param();

	pid_x = Pid(p.MAX_LINER_SPEED, p.X_KP, p.X_KD, p.X_KI);
	pid_y = Pid(p.MAX_LINER_SPEED, p.Y_KP, p.Y_KD, p.Y_KI);
	pid_theta = Pid(p.MAX_ANGULAR_SPEED, p.THETA_KP, p.THETA_KD, p.THETA_KI);

	state.data = WAIT_TO_DOCK;
	detection_state = UNDETCTED_TARGET;
	task = NO_TASK;
}

void Controller::toggle_state(const std_msgs::Bool state_toggle){
	if(task == NO_TASK){
		if(state_toggle.data && state.data == WAIT_TO_DOCK){
			task = DOCKING_TASK;
		}else if(!state_toggle.data && state.data == DOCKING_ENDED){
			task = BACKING_TASK;
		}
	}
	else if(task == DOCKING_TASK && !state_toggle.data){
		task = NO_TASK;
		state.data = WAIT_TO_DOCK;
	}
}

void Controller::update_navigate_position(
		const geometry_msgs::Point::ConstPtr& input){
	if(task == DOCKING_TASK){
		detection_state = TARGET_DETCTED;
		calc_target_pos(input);
		reset_current_pos();
	}
}

std_msgs::UInt8 Controller::get_state(){
	return state;
}

bool Controller::task_manger(){
	if(task == DOCKING_TASK){
		return dock();
	}else if(task == BACKING_TASK){
		return back();
	}else{
		ROS_INFO("No task to execute!");
		return false;
	}
}

geometry_msgs::Twist Controller::get_cmd_vel(){
    ROS_INFO("target_pos: x[%f] y[%f] Theta[%f]",
                     target_pos.x,
                     target_pos.y,
                     target_pos.z*180/3.141592);

    ROS_INFO("current_pos: x[%f] y[%f] Theta[%f]",
                     current_pos.x,
                     current_pos.y,
                     current_pos.z*180/3.141592);

    ROS_INFO("PUB: Set speed: x[%f] y[%f] Theta[%f]",
                    cmd_vel.linear.x,
                    cmd_vel.linear.y,
                    cmd_vel.angular.z);

	return cmd_vel;
}

// --------------- ControllerSubAndPub class functions deceleration ---------------
ControllerSubAndPub::ControllerSubAndPub(){
	//Topic you want to publish
	pub_cmd_vel = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	pub_docking_state = n_.advertise<std_msgs::UInt8>("docking_state", 10);
	//Topic you want to subscribe
	sub_docking_pose = n_.subscribe("docking_pose", 10,
			&ControllerSubAndPub::callback_navigate_position, this);
	sub_start_dock = n_.subscribe("start_dock", 10,
			&ControllerSubAndPub::callback_navigation_state, this);
	timer_ = n_.createTimer(ros::Duration(controller.p.TIME_DURATION),
			&ControllerSubAndPub::callback_timer, this);
}

void ControllerSubAndPub::callback_navigation_state(const std_msgs::Bool input){
	controller.toggle_state(input);
}

void ControllerSubAndPub::callback_navigate_position(
		const geometry_msgs::Point::ConstPtr& input){
	controller.update_navigate_position(input);
}

void ControllerSubAndPub::callback_timer(const ros::TimerEvent&){
	pub_docking_state.publish(controller.get_state());
	if(controller.task_manger()){
		pub_cmd_vel.publish(controller.get_cmd_vel());
	}
}
