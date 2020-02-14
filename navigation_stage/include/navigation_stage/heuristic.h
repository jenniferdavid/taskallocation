#ifndef HEURISTIC_H__
#define HEURISTIC_H_

#include <fstream>
#include <sstream>
#include <math.h>
#include <iomanip> // needed for setw(int)
#include <string>
#include "stdio.h"
#include <iostream>
#include <cstdlib>
#include <Eigen/Dense>
#include <Eigen/LU> 
#include <vector>
#include <cmath>
#include <algorithm>
#include <time.h>
#include <limits>
#include <cstring>
#include <sys/stat.h>
#include <ctime>
#include <sys/types.h>
#include <locale.h>
#include <wchar.h>
#include <unistd.h>
#include <csignal>
#include <map>

//ROS
#include "ros/ros.h"
#include "navi_msgs/Item.h"
#include "navi_msgs/ItemStruct.h"
#include "navi_msgs/Problem.h"
#include "navi_msgs/Goals.h"
#include "navi_msgs/GoalsList.h"
#include "navi_msgs/nameTasks.h"
#include "navi_msgs/listIntTasks.h"
#include "navi_msgs/listTasks.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Float64MultiArray.h"
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>

using namespace std;

// --------------- Heuristic class ---------------

class Heuristic {

public:
    	int nVehicles;      
	int nTasks;       
	int nDim; 
	int rDim; 

    	Eigen::MatrixXd DeltaMatrix;
	Eigen::VectorXd TVec; 

	Eigen::MatrixXd CostMatrix;
	Eigen::MatrixXd TaskMatrix;
	Eigen::MatrixXd EndMatrix;
	Eigen::MatrixXd StartMatrix;
	Eigen::MatrixXd EndHungMatrix;
	Eigen::MatrixXd StartHungMatrix;
	Eigen::MatrixXd TotalEndHungMatrix;
	Eigen::MatrixXd TotalStartHungMatrix;

	Eigen::VectorXd Edge; 
	Eigen::VectorXd Edge2; 
	Eigen::VectorXd Edge3; 

	Heuristic();
	void compute(int nVehicles, int nTasks, int nDim, int rDim, Eigen::MatrixXd DeltaMatrix);

};//End of class Heuristic


#endif //HEURISTIC_H_




