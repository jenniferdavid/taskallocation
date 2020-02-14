#ifndef TASK_SUBPUB_H_
#define TASK_SUBPUB_H_

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

#include "heuristic.h"
using namespace std;


// --------------- TaskSubPub class ---------------
class TaskSubPub {

public:
    	std::vector<std::pair<std::string,std::tuple<double,double,double>> > totalCoord;
	std::string solStrA;
	std::string solStrB;
	std::string veh_alpha = "ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890";
	std::string task_alpha = "abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@";

	std::vector<char> plotVehicle;
	std::vector<std::vector<char> >plotString;

	double checkTime;
	Eigen::VectorXd cTime;
	Eigen::VectorXd checkTimeVec;
	Eigen::VectorXd sub;

	Heuristic h;
        
	navi_msgs::Goals data;
	navi_msgs::GoalsList msg;
	navi_msgs::listTasks stringD1;
	navi_msgs::listIntTasks intD2;
	navi_msgs::nameTasks stringD3;
	geometry_msgs::PoseArray poseArrayMsg;
	geometry_msgs::Pose poseMsg;
	
	TaskSubPub();
	void deltaMatCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void probCallback(const navi_msgs::Problem::ConstPtr& msg);
	void coordsCallback(const navi_msgs::ItemStruct::ConstPtr& msg);
	std::tuple<std::string, std::string, int, std::vector <std::vector <char>> > displaySolution(Eigen::MatrixXd &VMatrix, Eigen::MatrixXd &DeltaMatrix, Eigen::VectorXd &TVec);
	void publishTasks(int nVehicles, std::vector<std::vector<char> >plotString);

private:
	ros::NodeHandle n;
        ros::Subscriber coordSub;
        ros::Subscriber deltamatSub;
        ros::Subscriber probSub;
        ros::Publisher goalPub;
        ros::Publisher taskPub;

};//End of class TaskSubAndPub


#endif //TASK_SUBPUB_H_




