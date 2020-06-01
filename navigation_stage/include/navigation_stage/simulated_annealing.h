#ifndef SIMULATEDANNEALING_H__
#define SIMULATEDANNEALING_H_

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

using namespace std;

// --------------- SimulatedAnnealing class ---------------

class SimulatedAnnealing {

public:
    	    
int nVehicles;
int nTasks;
int nDim;
int rDim;
double kT_start=100;
double kT_stop=0.01;
double kT_fac=0.9;
double kT;
double lk;
double costValue;
	Eigen::MatrixXd vMatBest; 

Eigen::VectorXd cTime;
Eigen::VectorXd vdVecL;
Eigen::VectorXd vdVecR; 
Eigen::VectorXd rightVec;
Eigen::VectorXd leftVec;
Eigen::MatrixXd vDeltaR;
Eigen::MatrixXd vDeltaL;
Eigen::VectorXd valVec;
Eigen::MatrixXd DeltaMatrix;
Eigen::VectorXd TVec;
Eigen::VectorXd sub;
Eigen::MatrixXd I;
Eigen::VectorXd checkTimeVec;

	SimulatedAnnealing();

bool isValid(Eigen::MatrixXd x);
float calculateLR(Eigen::MatrixXd VMatrix, Eigen::MatrixXd PMatrix, Eigen::MatrixXd DeltaMatrix); //calculate LR from V and PMatrix
	Eigen::MatrixXd compute(int nVehicles, int nTasks, int nDim, int rDim, Eigen::MatrixXd DeltaMatrix);
	Eigen::MatrixXd SA_algo(int &nVehicles, int &nTasks, int &nDim, int &rDim, Eigen::MatrixXd & DeltaMatrix);

};//End of class SimulatedAnnealing


#endif //SIMULATEDANNEALING_H_




