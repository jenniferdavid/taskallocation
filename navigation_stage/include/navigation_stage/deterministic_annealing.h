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

	double kT_start; //from arguments
	double kT_stop; //from arguments
	double kT_fac;
	double kT_in; //from arguments
	double g;    //from arguments
	double kT;
	double lk;
	double Okappa; //the kappa for factoring Etask set to kappa
	double kappa; 
	double beta;   //from arguments
	double Tkappa = 1; 
	double kT_swfac = 0.0015;
	double kT_fac2;
	double checkTime;

	bool LoopState = true;

	float small = FLT_MIN; //very small	
	float onemsmall = 1 - small; //1
	float lk0 = 1/small - 1; //big number
	//float big = FLT_MAX; //big number
	float big = 10000.00;	
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

	Eigen::MatrixXd Sinkhorn (Eigen::MatrixXd & VMatrix, int nDim, int rDim, int nVehicles); //this function normalises a matrix - makes into doubly stochastic matrix

 	Eigen::MatrixXd normalisation (Eigen::MatrixXd & VMatrix, int &nDim, int &rDim); //this function normalises a matrix - makes into doubly stochastic matrix

	Eigen::MatrixXd getVMatrix (int &nVehicles, int &nTasks, int &nDim, int &rDim, Eigen::MatrixXd & DeltaMatrix); //initialize VMatrix

	Eigen::MatrixXd syncUpdate (Eigen::MatrixXd & E_local, Eigen::MatrixXd & UpdatedVMatrix, int refix, double kT, int nDim, int rDim, int nVehicles); //Updating mean field equations (VMatrix) along row-wise
	
	std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::MatrixXf::Index, Eigen::MatrixXf::Index, double> calculateLR (Eigen::MatrixXd & VMatrix, Eigen::MatrixXd & PMatrix, Eigen::MatrixXd & DeltaMatrix, Eigen::VectorXd & TVec, int nVehicles); //calculate LR from V and PMatrix

	Eigen::MatrixXd calculateE (Eigen::VectorXd &leftVec, Eigen::VectorXd & rightVec, Eigen::MatrixXd & VMatrix, Eigen::MatrixXd & PMatrix, int & TaskState, bool & LoopState, double kappa, double Okappa, double g, double beta, int iteration, int nDim, int rDim, Eigen::MatrixXd DeltaMatrix, Eigen::VectorXd TVec, int nVehicles); //calculate ELocal using L,R and PMatrix

	Eigen::MatrixXd UpdateP (Eigen::MatrixXd &UpdatedVMatrix, Eigen::MatrixXd &VMatrix, Eigen::MatrixXd &PMatrix, int &PState); //calculate Updated P

	bool validity (Eigen::MatrixXd &Matrix); //finds the validity of the matrix

	std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> NN_algo(int nVehicles, int nTasks, int nDim, int rDim, Eigen::MatrixXd DeltaMatrix, double kT_start, double kT_stop, double kT_fac, double g, double beta); //the algorithm

	Eigen::MatrixXd compute(int nVehicles, int nTasks, int nDim, int rDim, Eigen::MatrixXd DeltaMatrix, double kT_start, double kT_stop, double kT_fac, double g, double beta);


};//End of class DeterministicAnnealing


#endif //DETERMINISTICANNEALING_H_




