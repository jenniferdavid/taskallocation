#ifndef DETERMINISTICANNEALING_H__
#define DETERMINISTICANNEALING_H_

#include <fstream>
#include <sstream>
#include <math.h>
#include <iomanip> // needed for setw(int)
#include <string>
#include <stdio.h>
#include <stdint.h>
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
#include <limits.h>
#include <math.h>



using namespace std;

// --------------- DeterministicAnnealing class ---------------

class DeterministicAnnealing {

public:
    	int nVehicles;      
	int nTasks;       
	int nDim; 
	int rDim; 
	int NofNorm;
	int PState = 2;
	int TaskState = 1;
	int VState;

	double kT_start = 100.0;
	double kT_stop = 0.01;
	double kT_fac = 0.9;
	double kT_in = 0.9;
	double g = 1;
	double kT;
	double lk;
	double Okappa; //the kappa for factoring Etask
	double kappa = 1;
	double beta = 1;
	double Tkappa = 1; //
	double kT_swfac = 0.0015;
	double kT_fac2;
	double checkTime;

	bool LoopState = true;

	float small = FLT_MIN; //very small	
	float onemsmall = 1 - small; //1
	float lk0 = 1/small - 1; //big number
	float big = FLT_MAX; //big number
	float costValueBest = big; 
    
	Eigen::MatrixXd VMatrix;
	Eigen::MatrixXd DeltaMatrix;
	Eigen::MatrixXd PMatrix;
	Eigen::MatrixXd I;//
        Eigen::MatrixXf::Index maxl, maxr;
	Eigen::MatrixXd NonNormUpdatedVMatrix;
	Eigen::MatrixXd NonNormUpdatedPMatrix;
	Eigen::MatrixXd DeltaVMatrix;
	Eigen::MatrixXd UpdatedVMatrix;
	Eigen::MatrixXd UpdatedWMatrix;
	Eigen::MatrixXd UpdatedPMatrix;
	Eigen::MatrixXd E_task; 
	Eigen::MatrixXd E_loop; 
	Eigen::MatrixXd E_norm; 
	Eigen::MatrixXd E_local; 
	Eigen::MatrixXd vDeltaR;
	Eigen::MatrixXd vDeltaL;
	Eigen::MatrixXd vMatBest; 

	Eigen::VectorXd sumEnorm;
	Eigen::VectorXd TVec; 
	Eigen::VectorXd vdVecL;
	Eigen::VectorXd vdVecR;
	Eigen::VectorXd rightVec;
	Eigen::VectorXd leftVec;
	Eigen::VectorXd sumv;
	Eigen::VectorXd sumw; 
	Eigen::VectorXd sumr; 
	Eigen::VectorXd sumc;
	Eigen::VectorXd sumrow;
	Eigen::VectorXd sumcol;
	Eigen::VectorXd cTime;
	Eigen::VectorXd checkTimeVec;
	Eigen::VectorXd sub;
	Eigen::VectorXd X;

	DeterministicAnnealing();
	Eigen::MatrixXd sinkhorn (Eigen::MatrixXd & VMatrix, int &nVehicles, int &nTasks, int &nDim, int &rDim);
 	Eigen::MatrixXd normalisation (Eigen::MatrixXd & VMatrix);
	Eigen::MatrixXd getVMatrix (int &nVehicles, int &nTasks, int &nDim, int &rDim, Eigen::MatrixXd & DeltaMatrix); 
	Eigen::MatrixXd syncUpdate (Eigen::MatrixXd & E_local, Eigen::MatrixXd & UpdatedVMatrix, int refix, int rDim, int nDim, double kT, int nVehicles); 
	Eigen::MatrixXd calculateE (Eigen::VectorXd &leftVec, Eigen::VectorXd & rightVec, Eigen::MatrixXd & VMatrix, Eigen::MatrixXd & PMatrix, int & TaskState, bool & LoopState, double kappa, double Okappa, int iteration, int nDim, Eigen::MatrixXd & DeltaMatrix, int nVehicles);
	Eigen::MatrixXd updateP (Eigen::MatrixXd &UpdatedVMatrix, Eigen::MatrixXd &VMatrix, Eigen::MatrixXd &PMatrix, int &PState); 
	Eigen::MatrixXd compute(int nVehicles, int nTasks, int nDim, int rDim, Eigen::MatrixXd DeltaMatrix);
	
	std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> NN_algo(int &nVehicles, int &nTasks, int &nDim, int &rDim, Eigen::MatrixXd & DeltaMatrix); 
	std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::MatrixXf::Index, Eigen::MatrixXf::Index, double> calculateLR (Eigen::MatrixXd & VMatrix, Eigen::MatrixXd & PMatrix, Eigen::MatrixXd & DeltaMatrix, int &nVehicles);

	bool validity (Eigen::MatrixXd &Matrix);
};//End of class DeterministicAnnealing


#endif //DETERMINISTICANNEALING_H_




