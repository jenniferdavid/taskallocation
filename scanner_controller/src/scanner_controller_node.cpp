//============================================================================
// Name       : scanner_controller_node.cpp
// Author     : Ali Murtatha Shuman
// Version    : V1.0
// Copyright  : For any questions contact Ali Murtatha Shuman
//				Email: ali.m.shuman@gmail.com
// 				Phone: +46 76 26 55 339
// Description: docking controller node for controlling the platform to make a precision docking
//				using a scanner.
//============================================================================


#include "scanner_controller.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "scanner_controller");
	ControllerSubAndPub controllerSAP;
	ros::spin();

	return 0;
}
