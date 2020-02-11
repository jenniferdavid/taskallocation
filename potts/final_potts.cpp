/* Trials with Potts_Spin based optimization.
 *
 * Copyright (C) 2014 Jennifer David. All rights reserved.
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
   \file potts_spin.cpp
   
   Trials with running neural network based optimization method for task assignment. 
   
*/
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip> // needed for setw(int)
#include <string>
#include <cstdlib>
#include <limits>
#include <cstring>
#include <ctime>
#include <csignal>

#include "stdio.h"

#include <Eigen/Dense>
#include <Eigen/LU> 
#include <unsupported/Eigen/MatrixFunctions>
#include <time.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <locale.h>
#include <wchar.h>
#include <unistd.h>

#include "gnuplot_iostream.h"

using namespace std;
using namespace Eigen;

int NofNorm;
int nVehicles;
int nTasks;
int nDim;
int rDim;
int PState;
int TaskState;
int VState;

double kT_start;
double kT_stop;
double kT_fac;
double kT_in;
double g;
double kT;
double lk;
double Okappa;
double kappa;
double beta;
double Tkappa;
double kT_swfac = 0.0015;
double kT_fac2;
double checkTime;

bool LoopState;

static const double small = 1e-15; //very small
static const double onemsmall = 1 - small; //1
static const double lk0 = 1/small - 1; //big number
static const double big = 10000000000; //big number
double costValueBest = big; 

std::string outputpath = "/home/jennifer/catkin_ws/src/taskallocation/potts/OUTPUTS/";
std::string inputpath = "/home/jennifer/catkin_ws/src/taskallocation/potts/";
std::string veh_alpha = "ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890";
std::string task_alpha = "abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@";

//std::string task_alpha = "{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghij";

std::string solStrA;
std::string solStrB;

std::vector<char> plotVehicle;
std::vector<std::vector<char>> plotString;
        
ofstream outfile16;
ofstream outfile17;
ofstream outfile18;
    
////////////////////////////////////////////////////////////////////////////////////////

Eigen::MatrixXd VMatrix;
Eigen::MatrixXd DeltaMatrix;
Eigen::MatrixXd PMatrix;
Eigen::MatrixXd I = MatrixXd::Identity(nDim,nDim); 
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
Eigen::VectorXd vdVecL(nDim);
Eigen::VectorXd vdVecR(nDim);
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

std::sig_atomic_t volatile done = 0;
void game_over(int) { done = 1; }
MatrixXf::Index maxl, maxr;


double euclideanDistance (std::pair<double,double> a, std::pair<double,double> b)
	{
		double dist, u, v;
		u = b.first - a.first;
		v = b.second - a.second;
                //cout << "\n x2-x1 is " << u << endl;
                //cout << "\n y2-y1 is " << v << endl;
		dist = u*u + v*v; //xb-xa and yb-ya is 
		//cout << "dist is" << dist <<endl;
		dist = sqrt(dist);
		//cout << "dist is" << dist <<endl;
		return dist;
	}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::MatrixXd Sinkhorn (Eigen::MatrixXd & VMatrix) //this function normalises a matrix - makes into doubly stochastic matrix

{
    Eigen::VectorXd sumd(rDim);
    Eigen::VectorXd c(rDim); 
    Eigen::VectorXd r(rDim);
    Eigen::VectorXd rd(rDim);
    Eigen::VectorXd one = VectorXd::Ones(rDim);
    Eigen::VectorXd cinv(rDim);
    Eigen::VectorXd cd(rDim);
    Eigen::MatrixXd reducedVMat = MatrixXd::Zero(rDim,rDim); 
    
    reducedVMat = VMatrix.block(0,nVehicles,rDim,rDim);
    
    int iter = 1;
    int maxiter = 10000;
    double tol = 0.01;
    
    cout << "\nrow wise is \n" << reducedVMat.rowwise().sum() << endl;
    cout << "\ncol wise is \n" << reducedVMat.colwise().sum() << endl;
    
    sumc = reducedVMat.colwise().sum();
    c = sumc.cwiseInverse(); //c = 1/
    //cout << "\n c is:\n"<< c << endl;

    rd = c.transpose();
    r = reducedVMat * rd;
    r = r.cwiseInverse();
    
    cout << "\n r is:\n"<< r << endl;
    
    while (iter < maxiter)
    {
            iter = iter +1;
            cinv = (r.transpose()) * reducedVMat;
      //      cout << "\n cinv is:\n"<< cinv << endl;
            
            if (((cinv.cwiseProduct(c)-one).cwiseAbs().maxCoeff()) <= tol) //tol
            {cout << "\n ans is:\n"<< ((cinv.cwiseProduct(c)-one).cwiseAbs().maxCoeff()) <<endl;
            break;}
            
            c = cinv.cwiseInverse();
      //      cout << "\n c is:\n"<< c << endl;

            cd = c.transpose();
            r = reducedVMat * cd;
            r = r.cwiseInverse();
      //      cout << "\n r is:\n"<< r << endl;

      //      cout << "rXc\n"<<r*c.transpose() <<endl;
     }
     
     reducedVMat = reducedVMat.cwiseProduct(r*c.transpose());
     //cout << "\n V is:\n"<< reducedVMat << endl;
     cout << "\nrow wise is \n" << reducedVMat.rowwise().sum() << endl;
     cout << "\ncol wise is \n" << reducedVMat.colwise().sum() << endl;
   
     VMatrix.leftCols(nVehicles) *= 0;
     VMatrix.bottomRows(nVehicles) *= 0;
     VMatrix.block(0,nVehicles,rDim,rDim) = reducedVMat;
     
     cout << "\n V is:\n"<< VMatrix << endl;

     return VMatrix;
    
}

  
Eigen::MatrixXd normalisation (Eigen::MatrixXd & VMatrix) //this function normalises a matrix - makes into doubly stochastic matrix
  {
      //Normalising rows of VMatrix
      int y = 0;
      LABEL0:
      sumr = VMatrix.rowwise().sum();
      for (int i = 0; i < nDim; i++)
                {
                    if (sumr(i) == 0.000)
                        { 
                           // cout << "\n Row "  << " is with constraints, so skipping" << endl;
                        }
                    else if (sumr(i) == 1.000)
                        { 
                           // cout << "\n Row "  << " is already normalised" << endl;
                        }
                    else 
                        {   // cout << "\n Row Normalising " << endl;
                            for (int j = 0; j < nDim; j++)
                            {VMatrix(i,j) = VMatrix(i,j)/sumr(i);}
                        }
                 }                  
     //  cout << "\n So finally, the Row Normalised UpdatedVMatrix is \n" << UpdatedVMatrix << endl;
     // cout << "\n row sum after row normalisation is \n" << UpdatedVMatrix.rowwise().sum() << endl;
   
       //Normalising columns of VMatrix
       sumc = VMatrix.colwise().sum();
       for (int i = 0; i < nDim; i++)
                {
                    if (sumc(i) == 0)
                        {//cout << "\n Col " << k << " is with constraints, so skipping" << endl;
                        }
                    else if (sumc(i) == 1)
                        {//cout << "\n Col " << k << " is already normalised" << endl;
                        }   
                    else 
                        {//cout << "\n Column Normalising \n" << endl;
                            for (int j = 0; j < nDim; j++)
                                {VMatrix(j,i) = VMatrix(j,i)/sumc(i);}
                        }
                 }
       y++;
//        sumr = VMatrix.rowwise().sum();
//        sumc = VMatrix.colwise().sum();
//        if (((sumr > 0.999).all()) && ((sumc > 0.999).all()))
//            {
//             cout << "\n Normalised VMatrix is: \n" << VMatrix << endl;
//             cout << "Sum of VMatrix after row normalisation \n" << VMatrix.rowwise().sum() << endl;
//             cout << "Sum of VMatrix after col normalisation \n" << VMatrix.colwise().sum() << endl;
//            }
//       else
//           goto LABEL0;

         int sum_norm = 0;
         int col_norm = 0;
         for (int i = 0; i < nDim; i++)
                {                    
                    if (sumr(i) > 0.97)
                    {sum_norm = sum_norm+1;}
                }

         for (int i = 0; i < nDim; i++)
                {                    
                    if (sumc(i) > 0.97)
                    {col_norm = col_norm+1;}
                }
         
         if ((sum_norm == rDim) && (col_norm == rDim))
             {
            cout << "\n VMatrix is NORMALISED \n" << endl;
          //  cout << "Sum of VMatrix after row normalisation \n" << VMatrix.rowwise().sum() << endl;
          //  cout << "Sum of VMatrix after col normalisation \n" << VMatrix.colwise().sum() << endl;
             }
        else 
            goto LABEL0;
              
//        if (y != NofNorm) //number of normalisation times
//             {goto LABEL0;}
//        else
//             {cout << "\n Normalised VMatrix is: \n" << VMatrix << endl;
//             cout << "Sum of VMatrix after row normalisation \n" << VMatrix.rowwise().sum() << endl;
//             cout << "Sum of VMatrix after col normalisation \n" << VMatrix.colwise().sum() << endl;}
       return VMatrix;
  }
  
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::MatrixXd getVMatrix (int &nVehicles, int &nTasks, int &nDim, int &rDim, Eigen::MatrixXd & DeltaMatrix) //initialize VMatrix
  {
      cout << "\n ///////////////////////////////////////////////////////////////////// " << endl;
      cout << "\n No. of Vehicles M = " << nVehicles << endl;
      cout << "\n No. of Tasks N = " << nTasks << endl;
      cout << "\n Dimension of VMatrix is: " << nDim << endl;
      cout << "\n Total no of Vij: " << nDim*nDim << endl;
        
      VMatrix = MatrixXd::Zero(nDim,nDim);
      double tmp = 1./(nDim-1);
      cout << "\n tmp is: " << tmp << endl;
      std::srand((unsigned int) time(0));

      // intitalising VMatrix with 1/n values
      for (int i = 0; i < nDim; i++)
        {
            for (int j = 0; j < nDim; j++)
            {
                VMatrix(i,j) = tmp + (((double) rand() / (RAND_MAX))/80) ;// + 0.02*(rand() - 0.5);	// +-1 % noise
            }
        }
      cout << "\n VMatrix is \n" << VMatrix << endl;

      // If DeltaMatrix values is huge, then VMatrix values are set to zero
      for (int i = 0; i < nDim; i++)
        {
         for (int j = 0; j < nDim; j++)
            {
            if(DeltaMatrix(i,j) >= big)
                 VMatrix(i,j) = 0;
            else (VMatrix(i,j) = VMatrix(i,j));
            }
        }
      
      cout << "\n VMatrix set according to DeltaMatrix is \n" << VMatrix << endl;
//    adding all the constraints for the vehicles and tasks
   VMatrix.diagonal().array() = 0;
   VMatrix.leftCols(nVehicles) *= 0;
   VMatrix.bottomRows(nVehicles) *= 0;
   VMatrix.topRightCorner(nVehicles,nVehicles) *= 0;  
   /*VMatrix.triangularView<Lower>() *= 0; //for predefined order
   VMatrix.topRows(nDim) *= 0;           //for a=b and b=a ..
   VMatrix.rightCols(nVehicles) *= 0;    //.. setting same values*/
      VMatrix = normalisation(VMatrix);

      return VMatrix;
  }
  
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
Eigen::MatrixXd syncUpdate (Eigen::MatrixXd & E_local, Eigen::MatrixXd & UpdatedVMatrix, int refix) //Updating mean field equations (VMatrix) along row-wise
  {
        if (refix == 1) { cout << "\nUpdating without Nan" << endl; 
        Eigen::VectorXd X = UpdatedVMatrix.rowwise().maxCoeff();}
        
        //in case to use random switch between row and col
        //rn = rand() % 2; cout << "\nrn is: " << rn << endl;      
            
        //Updating the numerator
        Eigen::MatrixXd E = MatrixXd::Zero(nDim,nDim);
        E = ((-1*E_local)/kT);
        
        cout << "\n Updating VMatrix is: \n" << UpdatedVMatrix << endl;

        if (refix == 1) //for Nan type
        {
            for (int i = 0; i < rDim; i++)
             {
               for (int j=nVehicles; j< nDim; j++)
                    {
                        UpdatedVMatrix(i,j) = std::exp (E(i,j)-X(i));
                    }  
             }
        }
        else //for regular ones
        {    
            for (int i = 0; i < rDim; i++)
             {
               for (int j=nVehicles; j< nDim; j++)
                    {
                        UpdatedVMatrix(i,j) = std::exp(E(i,j));
                    }  
             }
        UpdatedWMatrix = UpdatedVMatrix;
        }
        
        cout << "\n Updating VMatrix is: (the numerator is updated) \n" << UpdatedVMatrix << endl;
        
        //Updating the denominator - either row or col wise
        Eigen::VectorXd sumv = UpdatedVMatrix.rowwise().sum(); //row sum
        Eigen::VectorXd sumw = UpdatedWMatrix.colwise().sum(); //col sum

        cout << "Sum row vector of V Matrix is (my ref): \n" << UpdatedVMatrix.rowwise().sum() << endl;
        //cout << "Sum col vector of W Matrix is (my ref): \n" << UpdatedWMatrix.colwise().sum() << endl;

        if (refix == 1) 
        {
            for (int i = 0; i < rDim; i++)
             {                    
               for (int j=nVehicles; j< nDim; j++)
                    {UpdatedVMatrix(i,j) = UpdatedVMatrix(i,j)/(sumv(i)-X(i));}
             }
        }
        else
        {
        for (int i = 0; i < rDim; i++)
             {                    
               for (int j=nVehicles; j< nDim; j++)
                    {
                        UpdatedVMatrix(i,j) = UpdatedVMatrix(i,j)/sumv(i); //dividing by row partition function
                        UpdatedWMatrix(i,j) = UpdatedWMatrix(i,j)/sumw(j); //dividing by col partition function
                    }  
             }     
        }
        //Printing out the VMatrix
//         cout << "\n Updating Row VMatrix is (complete update i.e. dividing by the partition function):  \n" << UpdatedVMatrix << endl;
//         cout << "\n Updating Col WMatrix is (complete update i.e. dividing by the partition function):  \n" << UpdatedWMatrix << endl;
// 
//         cout << "Sum row vector of V Matrix \n" << UpdatedVMatrix.rowwise().sum() << endl;
//         cout << "Sum col vector of V Matrix \n" << UpdatedVMatrix.colwise().sum() << endl;
//         cout << "Sum row vector of W Matrix \n" << UpdatedWMatrix.rowwise().sum() << endl;
//         cout << "Sum col vector of W Matrix \n" << UpdatedWMatrix.colwise().sum() << endl;
        
        //UpdatedVMatrix = 0.5*(UpdatedWMatrix+UpdatedVMatrix);
        //UpdatedVMatrix = (UpdatedWMatrix*UpdatedVMatrix);
        //cout << "\n W*V is: \n" << UpdatedVMatrix << endl;
//         for (int i = 0; i < rDim; i++)
//              {                    
//                for (int j=nVehicles; j< nDim; j++)
//                     {UpdatedVMatrix(i,j) = sqrt((UpdatedWMatrix(i,j)*UpdatedVMatrix(i,j)));}
//              }
//         cout << "\n The Geometric Mean Update is: \n" << UpdatedVMatrix << endl;
        cout << "\n UpdatedVMatrix is: \n" << UpdatedVMatrix << endl;
        return UpdatedVMatrix;
    }
      
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::MatrixXf::Index, Eigen::MatrixXf::Index, double> calculateLR (Eigen::MatrixXd & VMatrix, Eigen::MatrixXd & PMatrix) //calculate LR from V and PMatrix
  {
        cout << "\n VMatrix is: \n" << VMatrix << endl;
        vDeltaL = VMatrix.transpose() * DeltaMatrix;
        cout << "\n vDeltaL is: \n" << vDeltaL << endl;
        vdVecL = vDeltaL.diagonal();
        cout << "\n vdVecL is: \n" << vdVecL << endl;
        leftVec = PMatrix.transpose() * (TVec + vdVecL);
        cout << "\n leftVec is: \n" << leftVec << endl;

        vDeltaR = VMatrix * DeltaMatrix.transpose();
        cout << "\n vDeltaR is: \n" << vDeltaR << endl;
        vdVecR = vDeltaR.diagonal();
        cout << "\n vdVecR is: \n" << vdVecR << endl;
        rightVec = PMatrix * (TVec +vdVecR);
        cout << "\n rightVec is: \n" << rightVec << endl;
        
        cout << "\n Required elements in leftVec are: \n" << leftVec.tail(nVehicles) << endl;
        cout << "\n Required elements in rightVec are: \n" << rightVec.head(nVehicles) << endl;
        
        double maxlVecInd = leftVec.maxCoeff(&maxl);
        double maxrVecInd = rightVec.maxCoeff(&maxr);
        kappa = 0.5 * (leftVec(maxl) + rightVec(maxr));
        cout << "\nKappa is: \n" << kappa << endl;
        
        return std::make_tuple(leftVec, rightVec, maxl, maxr, kappa);
   }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
Eigen::MatrixXd calculateE (Eigen::VectorXd &leftVec, Eigen::VectorXd & rightVec, Eigen::MatrixXd & VMatrix, Eigen::MatrixXd & PMatrix, int & TaskState, bool & LoopState, double kappa, double Okappa, int iteration) //calculate ELocal using L,R and PMatrix
  {
        if (TaskState == 1)
        {
        cout << "\nMax update " << endl;
        cout << "\nmaxl is " <<maxl <<endl;
        cout << "\nmaxr is " <<maxr <<endl;
        cout << "\nkappa is " <<kappa <<endl;

        for (int i = 0; i < nDim; i++)
            {
            for (int j=0; j< nDim; j++)
                {
                        double X = 0; double Y = 0;
                        X = ((leftVec(i) + DeltaMatrix(i,j)) * PMatrix(j,maxl));
                        Y = ((rightVec(j) + DeltaMatrix(i,j)) * PMatrix(maxr,i));
                        E_task(i,j) = (0.5)*(X + Y);  
//                         cout << "\ni is " << i << endl;
//                         cout << "\nj is: " << j << endl;
//                         cout << "\nLeftVec("<<i<<") is: " << leftVec(i) << endl;
//                         cout << "\nPMatrix("<<j<<","<<maxl<<") is: " << PMatrix(j,maxl) << endl;
//                         cout << "\nRightVec("<<j<<") is: " <<rightVec(j) << endl;
//                         cout << "\nPMatrix("<<maxr<<","<<i<<") is: " << PMatrix(maxr,i) << endl;
//                         cout << "\nX is: " << X << endl;
//                         cout << "\nY is: " << Y << endl;
//                         cout << "\nE_task is:\n" << E_task << endl;
//                         cout << "\n*****************************"<<endl;
                }
            }
        }
        else
        {
            cout << "\nSum update " << endl;
            for (int i = 0; i < nDim; i++)
            {
            for (int j=0; j< nDim; j++)
                {
                 double X = 0; double Y = 0;
                 double sumaa = 0; double sumbb=0;
                 
                 for (int l=(nVehicles+nTasks); l<nDim ; l++)
                 {sumaa = sumaa + PMatrix(j,l);}
                 for (int l=0; l< nVehicles; l++)
                 {sumbb = sumbb + PMatrix(l,i);}
                        
                 X = (leftVec(i) + DeltaMatrix(i,j)) * sumaa;
                 Y = (rightVec(i) + DeltaMatrix(i,j)) * sumbb;
                 E_task(i,j) = (0.5/nVehicles)*(X + Y);
                 }
            }
        }
            
    cout << "\nE_task is: \n" << E_task << endl;
        
    E_task.diagonal().array() = 10000000000;
    E_task.leftCols(nVehicles) = DeltaMatrix.leftCols(nVehicles).eval();
    E_task.bottomRows(nVehicles) = DeltaMatrix.bottomRows(nVehicles).eval();
    E_task.topRightCorner(nVehicles,nVehicles) = DeltaMatrix.bottomLeftCorner(nVehicles,nVehicles).eval();   
    
    /////////////////////////////////////Printing out Etask////////////////////////////////////

            double E_taskaverage = 0;
            vector<double> validtaskElements;
            int taskelement = 0;
    
            for (int i = 0; i < nDim; i++)
            {
                for (int j=0; j< nDim; j++)
                {   
                    if ((E_task(i,j) < 100000) && (E_task(i,j)>0))
                    {   validtaskElements.push_back(E_task(i,j));
                        taskelement = taskelement + 1;
                        E_taskaverage = E_taskaverage + E_task(i,j); 
                    }
                }
            }
//             cout << "\nSum of all E_task is: \n" << E_taskaverage << endl;
//             cout << "\nNo of valid element in Etask are: \n" << taskelement << endl;
//             cout << "\nAverage values of valid E_task element is: \n" << E_taskaverage/taskelement << endl;
//             cout << "\nMinimum element in E_task is " << *std::min_element(validtaskElements.begin(),validtaskElements.end()) << endl;
//             cout << "\nLargest element in E_task is " << *std::max_element(validtaskElements.begin(),validtaskElements.end()) << endl;   
//             outfile17 << (E_taskaverage/taskelement);
//     
//             if (iteration == 0)
//                 {
//     //Okappa = E_taskaverage/taskelement;
//                 outfile18 << "\nSum of all E_task is: \n" << E_taskaverage << endl;
//                 outfile18 << "\nNo of valid element in Etask are: \n" << taskelement << endl;
//                 outfile18 << "\nAverage values of valid E_task element is: \n" << E_taskaverage/taskelement << endl;
//                 outfile18 << "\nMinimum element in E_task is " << *std::min_element(validtaskElements.begin(),validtaskElements.end()) << endl;
//                 outfile18 << "\nLargest element in E_task is " << *std::max_element(validtaskElements.begin(),validtaskElements.end()) << endl;   
//                 outfile18 << "\nkappa is: \n" << kappa << endl;
//                 outfile18 << "\n ///////////////////////////////////////////////////////////////////// " << endl;
//                 }
                cout << "\nkappa is: \n" << kappa << endl;
                cout << "\nOkappa is: \n" << Okappa << endl;
   
    E_task = E_task/(Okappa);
    //E_task = E_task/(Okappa/10);
    
                double E_taskaverage2 = 0;
                vector<double> validtaskElements2;
                int taskelement2 = 0;
                for (int i = 0; i < nDim; i++)
                {
                    for (int j=0; j< nDim; j++)
                    {   
                        if ((E_task(i,j) < 100000) && (E_task(i,j)>0))
                        {   validtaskElements2.push_back(E_task(i,j));
                            taskelement2 = taskelement2 + 1;
                            E_taskaverage2 = E_taskaverage2 + E_task(i,j); 
                        }
                    }
                }
//                 if (iteration == 0)
//                 {
//                 outfile18 << "\nSum of all E_task after is: \n" << E_taskaverage2 << endl;
//                 outfile18 << "\nNo of valid elements after in Etask are: \n" << taskelement2 << endl;
//                 outfile18 << "\nAverage values of valid E_task elements after is: \n" << E_taskaverage2/taskelement2 << endl;
//                 outfile18 << "\nMinimum element in E_task after is " << *std::min_element(validtaskElements2.begin(),validtaskElements2.end()) << endl;
//                 outfile18 << "\nLargest element in E_task after is " << *std::max_element(validtaskElements2.begin(),validtaskElements2.end()) << endl;   
//                 outfile18 << "\nkappa is: \n" << kappa << endl;
//                 outfile18 << "\n ///////////////////////////////////////////////////////////////////// " << endl;
//                 }
//    
//                 cout << "\nE_task after applying constraints and dividing by kappa is: \n" << E_task << endl;
//                 cout << "\n ///////////////////////////////////////////////////////////////////// " << endl;

    ///////////////////////////////////////////////////////////////////////////////////
    if (LoopState == true)
    {
    for (int i = 0; i < nDim; i++)
        {
            for (int j=0; j< nDim; j++)
                {   
                        double lk = PMatrix(j,i) / PMatrix(i,i);	// the "zeroed" Pji
                       /* if (lk < onemsmall ) //onesmall ~= 1
                            {lk = lk/(1-lk);} // => the resulting Pji for choice j
                        else
                            {lk = lk0;}  *///lk0 = big
                        E_loop(i,j) = lk;
                }
        }
    }
    else //E_loop = Trace(P)
    {
        Eigen::MatrixXd sqPMatrix = MatrixXd::Zero(nDim,nDim);
        sqPMatrix = PMatrix * PMatrix;
        for (int i = 0; i < nDim; i++)
        {
            for (int j=0; j< nDim; j++)
                {
                E_loop(i,j) = sqPMatrix(j,i);
                }
        }
    }
    
    E_loop.diagonal().array() = 10000000;
    cout << "\nE_loop is: \n" << E_loop << endl;
    
    /////////////////////////////////////Printing out Eloop////////////////////////////////////
/*    
                double E_loopaverage = 0;
                vector<double> validloopElements;
                int loopelement = 0;
    
                for (int i = 0; i < nDim; i++)
                {
                    for (int j=0; j< nDim; j++)
                    {   
                        if ((E_loop(i,j) < 100000) && (E_loop(i,j)>0))
                            {   validloopElements.push_back(E_loop(i,j));
                                loopelement = loopelement + 1;
                                E_loopaverage = E_loopaverage + E_loop(i,j); 
                            }
                    }
                }
                cout << "\nSum of all E_loop is: \n" << E_loopaverage << endl;
                cout << "\nNo of valid element in Eloop are: \n" << loopelement << endl;
                cout << "\nAverage values of valid E_loop element is: \n" << E_loopaverage/loopelement << endl;
                cout << "\nMinimum element in E_loop is " << *std::min_element(validloopElements.begin(),validloopElements.end()) << endl;
                cout << "\nLargest element in E_loop is " << *std::max_element(validloopElements.begin(),validloopElements.end()) << endl; 
    
                if (iteration == 0)
                {
                outfile18 << "\nSum of all E_loop is: \n" << E_loopaverage << endl;
                outfile18 << "\nNo of valid element in Eloop are: \n" << loopelement << endl;
                outfile18 << "\nAverage values of valid E_loop element is: \n" << E_loopaverage/loopelement << endl;
                outfile18 << "\nMinimum element in E_loop is " << *std::min_element(validloopElements.begin(),validloopElements.end()) << endl;
                outfile18 << "\nLargest element in E_loop is " << *std::max_element(validloopElements.begin(),validloopElements.end()) << endl;
                outfile18 << "\n ///////////////////////////////////////////////////////////////////// " << endl;
                }
                cout << "\n ///////////////////////////////////////////////////////////////////// " << endl;*/

    ///////////////////////////////////////////////////////////////////////////////////////
    
    E_norm = -1*VMatrix;/*
//     cout << "\nUpdatedVMatrix is \n" << VMatrix << endl;
//     cout << "\nrow wise is \n" << VMatrix.rowwise().sum() << endl;
//     cout << "\ncol wise is \n" << VMatrix.colwise().sum() << endl;
// 
//     sumEnorm = VMatrix.colwise().sum();
// 
//     for (int i = 0; i < rDim; i++)
//         {
//             for (int j=nVehicles; j< nDim; j++)
//                 {   
//                    // E_norm(i,j) = (sumEnorm(j) - 1); 
//                     E_norm(i,j) = (sumEnorm(j) - VMatrix(i,j) - 1); 
//                     E_norm(i,j) = (sumEnorm(j) - VMatrix(i,j) - 1); 
//                 }
//         }
//         
//     cout << "\nE_norm is: \n" << E_norm << endl;
//     cout << "\nrow wise Enorm is \n" << E_norm.rowwise().sum() << endl;
//     cout << "\ncol wise Enorm is \n" << E_norm.colwise().sum() << endl;*/
//     
            /////////////////////////////////////Printing out Enorm////////////////////////////////////
            double E_normaverage = 0;
            vector<double> validnormElements;
            int normelement = 0;
    
            for (int i = 0; i < nDim; i++)
                {
                    for (int j=0; j< nDim; j++)
                    {   
                        if (E_norm(i,j) < 100000)
                        {   validnormElements.push_back(E_norm(i,j));
                            normelement = normelement + 1;
                            E_normaverage = E_normaverage + E_norm(i,j); 
                        }
                    }
                }
    
//             cout << "\nSum of all E_norm is: \n" << E_normaverage << endl;
//             cout << "\nNo of valid element in Enorm are: \n" << normelement << endl;
//             cout << "\nAverage values of valid E_norm element is: \n" << E_normaverage/normelement << endl;
//             cout << "\nMinimum element in E_norm is " << *std::min_element(validnormElements.begin(),validnormElements.end()) << endl;
//             cout << "\nLargest element in E_norm is " << *std::max_element(validnormElements.begin(),validnormElements.end()) << endl;
//     
//             if (iteration == 0)
//             {
//             outfile18 << "\nSum of all E_norm is: \n" << E_normaverage << endl;
//             outfile18 << "\nNo of valid element in Enorm are: \n" << normelement << endl;
//             outfile18 << "\nAverage values of valid E_norm element is: \n" << E_normaverage/normelement << endl;
//             outfile18 << "\nMinimum element in E_norm is " << *std::min_element(validnormElements.begin(),validnormElements.end()) << endl;
//             outfile18 << "\nLargest element in E_norm is " << *std::max_element(validnormElements.begin(),validnormElements.end()) << endl;
//             outfile18 << "\n ///////////////////////////////////////////////////////////////////// " << endl;
//             }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    E_local = (g*E_loop) + (Tkappa*E_task) + (beta*E_norm); //calculating E matrix
    
            double E_localaverage = 0;
            vector<double> validlocalElements;
            int localelement = 0;
    
            for (int i = 0; i < nDim; i++)
            {
                for (int j=0; j< nDim; j++)
                {   
                    if (E_local(i,j) < 100000)
                    {   validlocalElements.push_back(E_local(i,j));
                        localelement = localelement + 1;
                        E_localaverage = E_localaverage + E_local(i,j); 
                    }
                }
            }
//             cout << "\nSum of all E_local is: \n" << E_localaverage << endl;
//             cout << "\nNo of valid element in Elocal are: \n" << localelement << endl;
//             cout << "\nAverage values of valid E_local element is: \n" << E_localaverage/localelement << endl;
//             cout << "\nMinimum element in E_local is " << *std::min_element(validlocalElements.begin(),validlocalElements.end()) << endl;
//             cout << "\nLargest element in E_local is " << *std::max_element(validlocalElements.begin(),validlocalElements.end()) << endl; 
//             outfile16 << (E_localaverage/localelement);
//     
//             if (iteration == 0)
//             {
//             outfile18 << "\nSum of all E_local is: \n" << E_localaverage << endl;
//             outfile18 << "\nNo of valid element in Elocal are: \n" << localelement << endl;
//             outfile18 << "\nAverage values of valid E_local element is: \n" << E_localaverage/localelement << endl;
//             outfile18 << "\nMinimum element in E_local is " << *std::min_element(validlocalElements.begin(),validlocalElements.end()) << endl;
//             outfile18 << "\nLargest element in E_local is " << *std::max_element(validlocalElements.begin(),validlocalElements.end()) << endl; 
//             outfile18 << "\n ///////////////////////////////////////////////////////////////////// " << endl;
//             }

    return E_local;
    }
    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::MatrixXd UpdateP (Eigen::MatrixXd &UpdatedVMatrix, Eigen::MatrixXd &VMatrix, Eigen::MatrixXd &PMatrix, int &PState) //calculate Updated P
  {
        Eigen::MatrixXd P = MatrixXd::Zero(nDim,nDim);
        if (PState == 1)
        {
            Eigen::MatrixXd dQ = MatrixXd::Zero(nDim,nDim);    
            Eigen::MatrixXd DeltaVMatrix = MatrixXd::Zero(nDim,nDim);
            Eigen::MatrixXd DeltaPMatrix = MatrixXd::Zero(nDim,nDim);
            
            cout << "\n  Previous VMatrix is \n" << VMatrix << endl;
            DeltaVMatrix = (UpdatedVMatrix - VMatrix);
            cout << "\n Delta V Matrix is \n" << DeltaVMatrix << endl;
            Eigen::ColPivHouseholderQR<MatrixXd >lu_decomp(DeltaVMatrix);
            int rank = lu_decomp.rank();
            cout << "\n Rank of Delta V Matrix is: " << rank << endl;
            dQ = DeltaVMatrix * PMatrix;    
            
            cout << "\n dQ:\n" << dQ << endl;
            cout << "\n trace of dQ:\n" << dQ.trace() << endl;
            cout << "\n 1 - (trace of dQ) :\n" << (1 - dQ.trace()) << endl;
            cout << "\n PMatrix old is \n" << PMatrix << endl;             
            
            DeltaPMatrix = (PMatrix * dQ)/(1-(dQ.trace()));
            cout << "\n Delta PMatrix:\n" << DeltaPMatrix << endl;
            UpdatedPMatrix = PMatrix + DeltaPMatrix;
            cout << "\n Updated PMatrix using SM method is:\n" << UpdatedPMatrix << endl;
        }
    else if (PState == 2)
        {
            P = (I-UpdatedVMatrix);
            UpdatedPMatrix = P.inverse();
            cout << "\n Updated PMatrix using exact inverse is:\n" << UpdatedPMatrix << endl;
        }           
        
    else if (PState == 3)
        {
        UpdatedPMatrix = I + (UpdatedVMatrix*PMatrix);
        cout << "\n Updated PMatrix using iterative update as follows:\n" << UpdatedPMatrix << endl;
        }
    
    else 
        {
        cout << "\n No options for updating PMatrix is given" << endl;
        }
    return UpdatedPMatrix;
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
bool validity (Eigen::MatrixXd &Matrix) //finds the validity of the matrix
  {
    sumrow = Matrix.rowwise().sum();
    sumcol = Matrix.colwise().sum();
    int row_sum = 0;
    int col_sum = 0;
    bool invalidSol = false;
    Eigen::MatrixXd pMat;
    
    for (int i = 0; i < nDim; i++)
        {                    
            if (sumrow(i) > 0.7)
            {row_sum = row_sum+1;}
        }

    for (int i = 0; i < nDim; i++)
        {                    
            if (sumcol(i) > 0.7)
            {col_sum = col_sum+1;}
        }
    
    for (int i = 0; i < nDim; i++)
            {for (int j = 0; j < nDim; j++)
                {
                if(Matrix(i,j) > 0.5)
                    Matrix(i,j) = 1;
                else if (Matrix(i,j) < 0.5)
                    Matrix(i,j) = 0;
                else (Matrix(i,j) = 0.5);
                }
            }
            
    if ((row_sum == rDim) && (col_sum == rDim))
        {   
            cout << "\nThe Matrix is valid \n" << endl;
            pMat = (I - Matrix).inverse();
            for (int i = 0; i < nDim; i++)
            {                    
            for (int j = 0; j< nDim; j++)
                 {   
                  if (std::isinf(pMat(i,j)))
                     {
                        cout << "\nIt is a loop solution" << endl;
                        invalidSol= true;
                        return invalidSol;
                      }
                  }
            }
        invalidSol = false;
        }
    else
        {invalidSol = true;}
        return invalidSol;
    }
    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> NN_algo() //the algorithm
  {             
         //calculating initial values L,R, energy for V and P
         Eigen::MatrixXd E_local = MatrixXd::Zero(nDim,nDim);
         Eigen::MatrixXd UpdatedPMatrix = MatrixXd::Zero(nDim,nDim); 
         Eigen::MatrixXd saveV = MatrixXd::Zero(nDim,nDim);
         
         E_task = MatrixXd::Zero(nDim,nDim);
         E_loop = MatrixXd::Zero(nDim,nDim);
         E_norm = MatrixXd::Zero(nDim,nDim);
         vMatBest = MatrixXd::Zero(nDim,nDim);
        
         DeltaVMatrix = MatrixXd::Zero(nDim,nDim);
         NonNormUpdatedVMatrix = MatrixXd::Zero(nDim,nDim);
         NonNormUpdatedPMatrix = MatrixXd::Zero(nDim,nDim);

         leftVec = VectorXd(nDim);
         rightVec = VectorXd(nDim);

         ofstream outfile3;
         std::string createFile3 = "";    //NonNormalised Matrix file
         //createFile3 = outputpath + "/" + "NonVMatrix";   
         createFile3 = "NonVMatrix";          
         outfile3.open(createFile3.c_str());     
        
         ofstream outfile4;   //VMatrix inside folder
         std::string createFile4 = "";    
         createFile4 = outputpath + "/" + "VMatrix";   
         outfile4.open(createFile4.c_str()); 
         
         ofstream outfile0;   //VMatrix for plotting 
         std::string createFile0 = "";    
         createFile0 = "VMatrix";   
         outfile0.open(createFile0.c_str());
         
         ofstream outfile5;
         std::string createFile5 = "";    
         createFile5 = outputpath + "/" + "Eloop" + ".txt";           //Eloop file
         outfile5.open(createFile5.c_str()); 
         
         ofstream outfile6;
         std::string createFile6 = "";    
         createFile6 = outputpath + "/" + "Elocal" + ".txt";          //Elocal file
         outfile6.open(createFile6.c_str()); 
         
         ofstream outfile7;
         std::string createFile7 = "";    
         createFile7 = outputpath + "/" + "Etask" + ".txt";           //Etask file
         outfile7.open(createFile7.c_str()); 
         
         ofstream outfile8;
         std::string createFile8 = "";    
         createFile8 = outputpath + "/" + "Pmatrix" + ".txt";         //PMatrix file
         outfile8.open(createFile8.c_str()); 
         
         ofstream outfile9;
         std::string createFile9 = "";    
         createFile9 = outputpath + "/" + "DeltaVMatrix" + ".txt";     //DeltaVMatrix file   
         outfile9.open(createFile9.c_str()); 
         
         ofstream outfile10;
         std::string createFile10 = "";    
         createFile10 = outputpath + "/" + "leftVec" + ".txt";         //leftVec file
         outfile10.open(createFile10.c_str()); 
         
         ofstream outfile11;
         std::string createFile11 = "";    
         createFile11 = outputpath + "/" + "rightVec" + ".txt";        //rightVec file
         outfile11.open(createFile11.c_str()); 
         
         ofstream outfile12;
         std::string createFile12 = "";    
         createFile12 = outputpath + "/" + "leftMax" + ".txt";         //leftMax file  
         outfile12.open(createFile12.c_str()); 
         
         ofstream outfile13;
         std::string createFile13 = "";    
         createFile13 = outputpath + "/" + "rightMax" + ".txt";        //rightMax file
         outfile13.open(createFile13.c_str()); 
         
         ofstream outfile15;
         std::string createFile15 = "";    
         createFile15 = outputpath + "/" + "kT" + ".txt";              //kT file
         outfile15.open(createFile15.c_str()); 
         
         ofstream outfile20;
         std::string createFile20 = "";    
         createFile20 = outputpath + "/" + "startVeh" + ".txt";              //startVeh Vs file
         outfile20.open(createFile20.c_str()); 
         
         ofstream outfile21;
         std::string createFile21 = "";    
         createFile21 = outputpath + "/" + "endVeh" + ".txt";              //endVeh Vs file
         outfile21.open(createFile21.c_str()); 
         
         ofstream outfile22;
         std::string createFile22 = "";    
         createFile22 = outputpath + "/" + "coreVs" + ".txt";              //coreVs file
         outfile22.open(createFile22.c_str()); 
         
         ofstream outfile23;
         std::string createFile23 = "";    
         createFile23 = outputpath + "/" + "ZeroVs" + ".txt";              //zeroVs file
         outfile23.open(createFile23.c_str()); 
         
         PMatrix = (I - VMatrix).inverse(); //calculates initial P
         cout << "\ninitial PMatrix is \n" << PMatrix << endl;
         cout << "\ninitial VMatrix is \n" << VMatrix << endl;
         
         calculateLR(VMatrix, PMatrix); // calculates LR for initial values
         Okappa = kappa;
         cout << "\ninitial leftVec is: \n" << leftVec << endl;
         cout << "\ninitial rightVec is: \n" << rightVec << endl;
         cout << "\nOriginal kappa \n" << Okappa << endl;
         
         int iteration = 0;
         int FLAG = 1;
         kT = kT_start;
         std::signal(SIGALRM, game_over);
         alarm(30); // this program will self-destruct in 5 seconds
         
         outfile16 << "\n" << iteration << "\t"; //Elocalavg
         outfile17 << "\n" << iteration << "\t"; //Etaskavg

         E_local = calculateE(leftVec,rightVec,VMatrix,PMatrix,TaskState,LoopState,kappa,Okappa,iteration); //calculate initial local energy using VMatrix
         cout << "\ninitial E_local is: \n" << E_local << endl;
         iteration = iteration + 1;
         int refix = 0;
         bool hold = false;
         double old_KT;
         double CT = nVehicles/nTasks;
         
         while (FLAG != 0) //iteration starting  //while (!done)
         {	

            cout << "\n ////////////////////////////////////////////////////////////////////////// " << endl;
            cout << "\n" << iteration << " ITERATION STARTING" << endl;
            cout << "\n kT is " << kT << endl;
            cout << "\n Current PMatrix is \n" << PMatrix << endl;
            cout << "\n Current VMatrix is \n" << VMatrix << endl;
            
            saveV = UpdatedVMatrix;
            UpdatedVMatrix = syncUpdate(E_local, VMatrix, refix); //calculate updatedVMatrix using ELocal
            cout << "\n UpdatedVMatrix now is \n" << UpdatedVMatrix << endl;
            cout << "\n col sum before normalisation is \n" << UpdatedVMatrix.colwise().sum() << endl;
            cout << "\n row sum before normalisation is \n" << UpdatedVMatrix.rowwise().sum() << endl;
            
            for (int i = 0; i < nDim; i++)
                {                    
                    for (int j = 0; j< nDim; j++)
                    {   
                        if ( std::isnan(UpdatedVMatrix(i,j)))// || std::isinf(UpdatedVMatrix(i,j)) )
                        {
                          cout << "\n The matrix has Nan" << endl;
                          refix = 1;
                          goto out;
                        //  UpdatedVMatrix = syncUpdate(E_local, VMatrix, refix); //reupdate V because of Nan
                        }                            
                    }
                }
            
            outfile3 << "\n" << iteration << "\t";  //Non-normalized VMatrix
            outfile4 << "\n" << iteration << "\t";  //Normalized VMatrix
            outfile0 << "\n" << iteration << "\t";  //Normalized VMatrix
            outfile5 << "\n" << iteration << "\t";  //Eloop
            outfile6 << "\n" << iteration << "\t";  //Elocal
            outfile7 << "\n" << iteration << "\t";  //Etask
            outfile8 << "\n" << iteration << "\t";  //PMatrix
            outfile9 << "\n" << iteration << "\t";  //DeltaVMatrix
            outfile10 << "\n" << iteration << "\t"; //leftVec
            outfile11 << "\n" << iteration << "\t"; //rightVec
            outfile12 << "\n" << iteration << "\t"; //maxleftValue
            outfile13 << "\n" << iteration << "\t"; //maxrightValue
            outfile16 << "\n" << iteration << "\t"; //Elocal avg
            outfile17 << "\n" << iteration << "\t"; //Etask avg
            outfile20 << "\n" << iteration << "\t"; //start vehicle Vs
            outfile21 << "\n" << iteration << "\t"; //end vehicle Vs
            outfile22 << "\n" << iteration << "\t"; //core Vs
            outfile23 << "\n" << iteration << "\t"; //zero Vs

            for (int i = 0; i < nDim; i++)
                {                    
                    for (int j = 0; j< nDim; j++)
                    {   
                        outfile3 << UpdatedVMatrix(i,j) << "\t"; //writing out non-normalized VMatrix to a file
                    }
                }
                
            NonNormUpdatedVMatrix = UpdatedVMatrix;
           
            //Normalising till the values along the row/columns is zero
            cout << "\n /////////////////////////////////////////////////////////////////////////////////////// " << endl;
            cout << "\n NORMALISATION BEGINS \n" << endl;
            //UpdatedVMatrix = normalisation(UpdatedVMatrix);
            UpdatedVMatrix = Sinkhorn(UpdatedVMatrix);
            cout << "\n FINAL NORMALISATION COL SUM is \n" << UpdatedVMatrix.colwise().sum() << endl;
            cout << "\n FINAL NORMALISATION ROW SUM is \n" << UpdatedVMatrix.rowwise().sum() << endl;
            cout << "\n Final row and column normalised UpdatedVMatrix is \n" << UpdatedVMatrix << endl;
            cout << "\n NORMALISATION ENDS \n" << endl;
            cout << "\n /////////////////////////////////////////////////////////////////////////////////////// " << endl;
            
            for (int i = 0; i < nDim; i++)
                {                    
                    for (int j = 0; j< nDim; j++)
                    {   
                        outfile4 << UpdatedVMatrix(i,j) << "\t"; //writing out normalized VMatrix to a folder
                        outfile0 << UpdatedVMatrix(i,j) << "\t"; //writing out normalized VMatrix to plot
                    }
                }
            outfile23 << UpdatedVMatrix.leftCols(nVehicles);
            outfile23 << UpdatedVMatrix.bottomRows(nVehicles);
            outfile23 << UpdatedVMatrix.topRightCorner(nVehicles,nVehicles);
            
            for (int i = 0; i < nVehicles; i++)
                {                    
                    for (int j = nVehicles; j< rDim; j++)
                    {   
                        outfile20 << UpdatedVMatrix(i,j) << "\t"; //writing out startVs
                    }
                }
                
            for (int i = nVehicles; i < rDim; i++)
                {                    
                    for (int j = nVehicles; j< rDim; j++)
                    {   
                        outfile21 << UpdatedVMatrix(i,j) << "\t"; //writing out endVs
                    }
                }
                
            for (int i = nVehicles; i < rDim; i++)
                {                    
                    for (int j = rDim; j< nDim; j++)
                    {   
                        outfile22 << UpdatedVMatrix(i,j) << "\t"; //writing out coreVs
                    }
                }

            UpdatedPMatrix = UpdateP(UpdatedVMatrix, VMatrix, PMatrix, PState); //update PMatrix
            for (int i = 0; i < nDim; i++)
                {                    
                    for (int j = 0; j< nDim; j++)
                    {   
                        if ( std::isnan(UpdatedPMatrix(i,j)) )
                        {
                          cout << "\n The Pmatrix has Nan so converging to loop solutions" << endl;
                          UpdatedVMatrix = saveV;
                          goto out;
                        }                            
                    }
                }
            calculateLR(UpdatedVMatrix, UpdatedPMatrix);   //Calculate LR
            E_local = calculateE(leftVec,rightVec,UpdatedVMatrix,UpdatedPMatrix, TaskState, LoopState, kappa,Okappa, iteration); //calculate E
            cout << "\n E_local is: \n " << E_local << endl;

            /////////////////////////////////Writing out solutions to a file////////////////////////////////////////////
            outfile12 << leftVec.maxCoeff();  //leftMax file
            outfile13 << rightVec.maxCoeff(); //rightMax file
            for (int i = 0; i < nDim; i++)
                {                    
                    outfile10 << leftVec(i) << "\t";
                    outfile11 << rightVec(i) << "\t";
                }
            
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////
            //Calculate the costvalue if the updatedVMatrix is above 0.8 and valid, if so, save it as best solution
            int ck = 0;
            for (int i = 0; i < nDim; i++)
                {                    
                    for (int j = 0; j< nDim; j++)
                    {   
                        if(UpdatedVMatrix(i,j) > 0.8)
                            {ck = ck+1;}
                    }
                }
            if (ck == rDim)
            {
                if (validity(UpdatedVMatrix) == false)
                    {
                        calculateLR(UpdatedVMatrix, UpdatedPMatrix);
                        float costValue = kappa;
                        if (costValue < costValueBest)
                            {
                            cout << "\nPresent costValue is " << costValue << endl;
                            cout << "\nBest costValue is " << costValueBest << endl;
                            vMatBest = UpdatedVMatrix;
                            costValueBest = costValue;
                            }   
                    }
            }
            
            DeltaVMatrix = (UpdatedVMatrix - VMatrix);
            saveV = UpdatedVMatrix; //a copy of the updatedVMatrix used in case of Nan
            double maxdV = DeltaVMatrix.maxCoeff();
            double meandV = DeltaVMatrix.mean();
            double meanV = UpdatedVMatrix.mean();
            
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////            
            // if the solutions are close to saturation, then stop            
            
            int check1 = 0; int check2 = 0;
            for (int i = 0; i < nDim; i++)
                {                    
                    for (int j = 0; j< nDim; j++)
                    {   
                        if (NonNormUpdatedVMatrix(i,j) > 0.99)
                            {check1 = check1+1;}
                        if (UpdatedVMatrix(i,j) > 0.99)
                            {check2 = check2+1;}
                    }
                }
            if (check1 == rDim)
            {cout << "\n Normalized matrix is saturated " << endl;}    
            if (check2 == rDim)
            {cout << "\n UpdatedV matrix is saturated " << endl;
             goto out;
            }  
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
            cout << "\nDeltaVMatrix is\n" << DeltaVMatrix << endl;
            cout << "\nmaxdV is " << maxdV << endl;
            cout << "\nmeandV is " << meandV << endl;

            if ((hold == false) && (maxdV > 0.03))
            {hold = true;}
            
            if (hold != true)
                {kT_fac = kT_in;}
            else
                {//kT_fac = kT_in*1.05;
                //kT_fac = kT_in + (maxdV*0.05);
                //kT_fac = kT_in + (0.99 * meandV);
                //kT_fac = kT_in + (1/(1+(std::exp(-(maxdV)))))/30;
                kT_fac = kT_in + 2* (0.99-kT_in) * ( ( 1 / (1+(std::exp(-(maxdV*2)))) ) - (0.5)); 
                //kT_fac = kT_in + (1/(1+(std::exp(-(meandV)))))/30;
                }
                
            if (kT_fac > 0.99) {kT_fac = 0.99;}
            cout << "\nkT_fac is " << kT_fac <<endl;
            
            kT *= kT_fac;
            outfile15 << kT_fac << endl;
            
            cout << "\nNew kT is: " << kT << endl;
            cout << "\n" << iteration << " ITERATION DONE" << endl;
            iteration = iteration + 1;
            VMatrix = UpdatedVMatrix;
            PMatrix = UpdatedPMatrix;
            cout << "\n /*/*/*/*/*/*/*/**/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/ " << endl;
            if (kT < kT_stop) 
            {FLAG = 0;}
            cout << "\n ///////////////////////////////////////////////////////////////////// " << endl;
         }
         out:
         return std::make_tuple(vMatBest, UpdatedVMatrix, NonNormUpdatedVMatrix);
 }  
 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::tuple<std::string, std::string, int,std::vector<std::vector<char>> > displaySolution (Eigen::MatrixXd &VMatrix, Eigen::MatrixXd &DeltaMatrix, Eigen::VectorXd &TVec) //parses the solution
  {
        int indx = 0;
        int indxB = 0;
        cTime = Eigen::VectorXd(nVehicles);
        sub = VectorXd::Ones(nVehicles);
      
        cout << "\nTHE SOLUTION is: \n" << endl;
        cout << VMatrix << endl;
           
           for (int i = 0; i < nVehicles; i++)
            {
            for (int j = 0; j < nDim; j++)
            {if (VMatrix(i,j)==1)
                {indx = j;}
            }
            
            if (i == 0)
            {
                solStrA = std::string("S") + veh_alpha[i] + std::string(" -> ") + task_alpha[indx-nVehicles];
                plotVehicle.push_back((veh_alpha[i]));
                plotVehicle.push_back((task_alpha[indx-nVehicles]));
                solStrB = "max(" + std::to_string(TVec(i)) + std::string(" + ")+ std::to_string(DeltaMatrix(i,indx)) + std::string(" + ") + std::to_string(TVec(indx));
            }
            else
            {
                solStrA = solStrA + std::string(" & S") + veh_alpha[i] + std::string(" -> ") + task_alpha[indx-nVehicles];
                if (!plotVehicle.empty())
                {plotString.push_back(plotVehicle);}
                plotVehicle.clear();
                plotVehicle.push_back((veh_alpha[i]));
                plotVehicle.push_back((task_alpha[indx-nVehicles]));
                solStrB = solStrB + std::string(", ") + std::to_string(TVec(i)) + std::string(" + ")+ std::to_string(DeltaMatrix(i,indx)) + std::string(" + ") + std::to_string(TVec(indx));
            }
            cTime[i] = DeltaMatrix(i,indx) + TVec(indx) + TVec(i);
            
            while (indx <= (nDim-nVehicles-1))
            {
                 for (int j = 0; j < nDim; j++)
                    {if (VMatrix(indx,j)==1)
                        {indxB = j;}
                    }
               
                if (indxB > (nVehicles+nTasks-1))
                {
                    solStrA = solStrA + std::string(" -> E") + veh_alpha[indxB-nVehicles-nTasks];
                    plotVehicle.push_back((veh_alpha[indxB-nVehicles-nTasks]));
                    if (!plotVehicle.empty())
                    {plotString.push_back(plotVehicle);}
                    plotVehicle.clear();
                    solStrB = solStrB + std::string(" + ") + std::to_string(DeltaMatrix(indx,indxB)) + std::string(" + ") +std::to_string(TVec(i));
                    cTime[i] = cTime[i] + DeltaMatrix(indx,indxB) + TVec(i);
                    solStrB = solStrB + std::string(" = ") + std::to_string(cTime[i]);            
                }
                else
                { 
                    solStrA = solStrA + std::string(" -> ") + task_alpha[indxB-nVehicles];
                    plotVehicle.push_back((task_alpha[indxB-nVehicles]));
                    solStrB = solStrB + std::string(" + ") + std::to_string(DeltaMatrix(indx,indxB)) + std::string(" + ") + std::to_string(TVec(indxB));
                    cTime[i] = cTime[i] + DeltaMatrix(indx,indxB) + TVec(indxB);
                }
                indx = indxB;
            }
        }
        solStrB = solStrB + std::string(")");
        checkTimeVec = VectorXd(2*nVehicles);
        checkTimeVec << cTime, cTime + (nVehicles * sub);
        VectorXf::Index maxE;
        checkTime = checkTimeVec.maxCoeff(&maxE) - nVehicles;
        cout << "The tasks are ordered as:\n";
        cout << "\n" <<solStrA << endl;
        cout << "\n" <<solStrB << endl;
        cout << "\n" <<checkTime << endl;
        
        for ( std::vector<std::vector<string>>::size_type i = 0; i < plotString.size(); i++ )
        {for ( std::vector<string>::size_type j = 0; j < plotString[i].size(); j++ )
        {std::cout << plotString[i][j] << ' ';}
        std::cout << std::endl;}  
        return std::make_tuple(solStrA, solStrB, checkTime, plotString);
    }
    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, const char* argv[])
  {
    //setlocale(LC_ALL, "");
    remove("VMatrix");
    remove("NonVMatrix");
    remove("vehCoord.txt");
    remove("taskCoord.txt");
    
    nVehicles = atoi(argv[1]);
    nTasks = atoi(argv[2]);
    kT_start = atof(argv[3]);
    kT_stop = atof(argv[4]);
    kT_in = atof(argv[5]);
    g = atof(argv[6]);
    beta = atof(argv[11]); //eta
    Tkappa =atof(argv[12]); //weighing factor of task function
        
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[40];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer, sizeof(buffer), "%d_%m_%Y_%I_%M_%S", timeinfo);
    clock_t tStart = clock();
    std::vector<std::pair<double,double>> vehCoord;
    std::vector<std::pair<double,double>> taskCoord;
    std::vector<std::pair<double,double>> totalCoord;
    std::map<char, std::pair<double,double>> vehMap;
    std::map<char, std::pair<double,double>> taskMap;
    
    if (argc != 14)
    {
        std::cout<<"\n Less input options... exiting "<<endl;
        return(0);
    }

    std::cout<<"\n nVehicles is "<<nVehicles<<endl;
    std::cout<<"\n nTasks is "<<nTasks<<endl;

    nDim = 2*nVehicles + nTasks;; 
    rDim = nVehicles + nTasks;

    std::cout<<"\n nDim is "<<nDim<<endl;
    std::cout<<"\n rDim is "<<rDim<<endl;
    std::cout<<"\n Gamma is "<<g<<endl;
    std::cout<<"\n Eta is "<<beta<<endl;
    std::cout<<"\n kappa for task function is "<<Tkappa<<endl;
    
    DeltaMatrix = MatrixXd::Ones(nDim,nDim);
    I = MatrixXd::Identity(nDim,nDim); //identity matrix
    TVec = VectorXd(nDim);
    VMatrix = MatrixXd::Zero(nDim,nDim);   
    UpdatedVMatrix = MatrixXd::Zero(nDim,nDim);
    UpdatedWMatrix = MatrixXd::Zero(nDim,nDim);
    std::string data = "M=" + std::to_string(nVehicles) +"_N="+std::to_string(nTasks);
    outputpath.append(std::string(data));
    std::string newDir = "_new";
    std::string at = " AT TIME ";
    outputpath.append(std::string(at));
    outputpath.append(std::string(buffer));
    mkdir(outputpath.c_str(),S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH );
    std::string folder = inputpath + data;
    std::string folder1, folder2, folder3, folder4, folder5, folder6, folder7;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (!strcmp(argv[7],"-random")) //generate random numbers for DeltaMatrix
     {    
        for (int i=0; i<nDim; i++)
           {
               TVec(i) = 1;
           }
        cout << "\n TVec is: \n" << TVec <<endl;
    
        std::srand((unsigned int) time(0));
        DeltaMatrix = MatrixXd::Random(nDim,nDim);
        double HI = 100; // set HI and LO according to your problem.
        double LO = 1;
        double range= HI-LO;
        DeltaMatrix = (DeltaMatrix + MatrixXd::Constant(nDim,nDim,1.)) * range/2;
        DeltaMatrix = (DeltaMatrix + MatrixXd::Constant(nDim,nDim,LO));
        cout << "\n DeltaMatrix is: \n" << DeltaMatrix <<endl;
        
        DeltaMatrix.diagonal().array() = 10000000000;
        DeltaMatrix.leftCols(nVehicles) *= 10000000000;
        DeltaMatrix.bottomRows(nVehicles) *= 10000000000;
        DeltaMatrix.topRightCorner(nVehicles,nVehicles) = DeltaMatrix.bottomRightCorner(nVehicles,nVehicles).eval(); 
        DeltaMatrix.bottomLeftCorner(nVehicles,nVehicles) = DeltaMatrix.bottomRightCorner(nVehicles,nVehicles).eval();       
    
        inputpath.append(std::string (data));
        inputpath.append(std::string (newDir));
        mkdir(inputpath.c_str(),S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH );
        ofstream outfile19;
        ofstream outfile24;
        std::string createFile19 = "";  
        std::string createFile24 = "";    
        createFile19 = inputpath + "/" + "newDeltaMat" + ".txt"; 
        createFile24 = inputpath + "/" + "newTVec" + ".txt";          
        outfile19.open(createFile19.c_str());  
        outfile24.open(createFile24.c_str());     
        outfile19 << DeltaMatrix << std::endl;
        outfile24 << TVec << std::endl;
        outfile19.close();
        outfile24.close();
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    else if (!strcmp(argv[7],"-coordsRandom")) //generate random coordinates and then calculate DeltaMatrix
    {    
        for (int i=0; i<nDim; i++)
           {
               TVec(i) = 1;
           }
        cout << "\n TVec is: \n" << TVec <<endl;
    
        std::srand((unsigned int) time(0));
        
        ofstream outfile26;
        ofstream outfile27;
       
        std::string createFile26 = "";    
        createFile26 = inputpath + "/" + "vehCoord" + ".txt";             
        outfile26.open(createFile26.c_str()); 
        std::string createFile27 = "";    
        createFile27 = inputpath + "/" + "taskCoord" + ".txt";              
        outfile27.open(createFile27.c_str()); 
        
        inputpath.append(std::string (data));
        inputpath.append(std::string (newDir));
        mkdir(inputpath.c_str(),S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH );
        ofstream outfile28;
        ofstream outfile29;
        std::string createFile28 = "";    
        std::string createFile29 = "";    
        createFile28 = inputpath + "/" + "vehCoord" + ".txt";             
        createFile29 = inputpath + "/" + "taskCoord" + ".txt";              
        outfile28.open(createFile28.c_str()); 
        outfile29.open(createFile29.c_str()); 
        
        ////////////////////****************************//////////////////////
        char veh = 65;
        //create a vehicle and task coordinates file and calculate deltamatrix
        for (int i=0; i<nVehicles; i++)
        {
            double item8 = rand() % 300 + (-150.0);
            double item9 = rand() % 300 + (-150.0);
            vehCoord.push_back(std::make_pair(item8,item9));
            vehMap.insert(make_pair(veh,make_pair(item8,item9)));
            outfile26 <<item8<<"\t"<< item9 <<"\n";
            outfile28 <<item8<<"\t"<< item9 <<"\n";
            veh++;
        } 
        
        for (std::map<char,std::pair<double,double>>::iterator it=vehMap.begin(); it!=vehMap.end(); ++it)
        {std::cout <<(it->first) <<"....."<< (it->second).first<<"....."<< (it->second).second << endl;}
        
        ////////////////////****************************//////////////////////
        char tas = 97;
        for (int i=0; i<nTasks; i++)
        {
            double item10 = rand() % 100 + (-50.0);
            double item11 = rand() % 100 + (-50.0);
            totalCoord.push_back(std::make_pair(item10,item11));
            taskCoord.push_back(std::make_pair(item10,item11));
            taskMap[tas]=std::make_pair(item10,item11);
            outfile27 <<item10<<"\t"<< item11 <<"\n";
            outfile29 <<item10<<"\t"<< item11 <<"\n";
            tas++;
        } 
        for (std::map<char,std::pair<double,double>>::iterator it=taskMap.begin(); it!=taskMap.end(); ++it)
        {std::cout <<(it->first) <<"....."<< (it->second).first<<"....."<< (it->second).second << endl;}
        
        ////////////////////****************************//////////////////////
        
        totalCoord.insert(totalCoord.end(), vehCoord.begin(), vehCoord.end());
        totalCoord.insert(totalCoord.begin(), vehCoord.begin(), vehCoord.end());

        for(int i = 0; i < totalCoord.size(); i++) //print all coordinates for delta matrix
        {cout << totalCoord[i].first << ", " << totalCoord[i].second << endl;}
        
        for (int i = 0; i < nDim; i++)
            for (int j = 0; j < nDim; j++)
                {
                    DeltaMatrix(i,j) = euclideanDistance(totalCoord[i],totalCoord[j]);
                }
        cout << "\n DeltaMatrix is: \n" << DeltaMatrix <<endl;
        DeltaMatrix.diagonal().array() = 10000000000;
        DeltaMatrix.leftCols(nVehicles) *= 10000000000;
        DeltaMatrix.bottomRows(nVehicles) *= 10000000000;
        DeltaMatrix.topRightCorner(nVehicles,nVehicles) = DeltaMatrix.bottomRightCorner(nVehicles,nVehicles).eval(); 
        DeltaMatrix.bottomLeftCorner(nVehicles,nVehicles) = DeltaMatrix.bottomRightCorner(nVehicles,nVehicles).eval();       
    
        ofstream outfile19;
        ofstream outfile24;
        
        std::string createFile19 = "";  
        std::string createFile24 = "";  
        
        createFile19 = inputpath + "/" + "coordDeltaMat" + ".txt"; 
        createFile24 = inputpath + "/" + "coordTVec" + ".txt";          
        
        outfile19.open(createFile19.c_str());  
        outfile24.open(createFile24.c_str());     
        
        outfile19 << DeltaMatrix << std::endl;
        outfile24 << TVec << std::endl;
        
        outfile19.close();
        outfile24.close();
        outfile28.close();
        outfile29.close();
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    else if (!strcmp(argv[7],"-read")) //read DeltaMatrix from the given input file
    {
        ifstream file;
        folder1 = folder+ "/tVec.txt"; 
        cout <<"\n"<<folder1<<endl;
        file.open(folder1); 
        if (file.is_open())
        {
           for (int i=0; i<nDim; i++)
           {
               double item;
               file >> item;
               TVec(i) = item;
           }
        }
        else
        {cout <<"\n tVec file not open"<<endl;
         return(0);
        }
        cout << "\n TVec is: \n" << TVec <<endl;
        ifstream file2;
        folder2 = folder+ "/deltaMat.txt"; 
        cout <<"\n"<<folder2<<endl;
        file2.open(folder2); 
        if (file2.is_open())
        {
            for (int i = 0; i < nDim; i++)
                for (int j = 0; j < nDim; j++)
                    {
                        double item2;
                        file2 >> item2;
                        DeltaMatrix(i,j) = item2;
                    }
        } 
        else
        {cout <<"\n Deltamat file not open"<<endl;
        return(0);
        }
        DeltaMatrix.diagonal().array() = 10000000000;
        DeltaMatrix.leftCols(nVehicles) *= 10000000000;
        DeltaMatrix.bottomRows(nVehicles) *= 10000000000;
        DeltaMatrix.topRightCorner(nVehicles,nVehicles) = DeltaMatrix.bottomRightCorner(nVehicles,nVehicles).eval(); 
        DeltaMatrix.bottomLeftCorner(nVehicles,nVehicles) = DeltaMatrix.bottomRightCorner(nVehicles,nVehicles).eval(); 
    
        cout << "\n DeltaMatrix is: \n" << DeltaMatrix <<endl;
        
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    else if (!strcmp(argv[7],"-coordsRead")) //read coordinates from the given input file and then calculate DeltaMatrix
    {
        ifstream file6;  //read tVec from the given file
        folder6 = folder+ "/tVec.txt"; 
        cout <<"\n"<<folder6<<endl;
        file6.open(folder6); 
        if (file6.is_open())
        {
           for (int i=0; i<nDim; i++)
           {
               double item;
               file6 >> item;
               TVec(i) = item;
           }
        }
        else
        {cout <<"\n tVec file not open"<<endl;
         return(0);
        }
        
        ofstream outfile26;
        ofstream outfile27;
       
        std::string createFile26 = "";    
        createFile26 = inputpath + "/" + "vehCoord" + ".txt";             
        outfile26.open(createFile26.c_str()); 
        std::string createFile27 = "";    
        createFile27 = inputpath + "/" + "taskCoord" + ".txt";              
        outfile27.open(createFile27.c_str()); 
        
        ifstream file4; //read vehicle coordinates
        folder4 = folder+ "/vehCoord.txt"; 
        cout <<"\n"<<folder4<<endl;
        file4.open(folder4); 
        if (file4.is_open())
        {
           char veh = 65;
           for (int i=0; i<nVehicles; i++)
           {
                double item4,item6;
                file4 >> item4;
                file4 >> item6;
                vehCoord.push_back(std::make_pair(item4,item6));
                vehMap.insert(make_pair(veh,make_pair(item4,item6)));
                outfile26 <<item4<<"\t"<< item6 <<"\n";
                veh++;
           }
        }
        else
        {cout <<"\n vehCoord file not open"<<endl;
         return(0);
        }
        
        cout << "\n*****************"<<endl;
        for(int i = 0; i < vehCoord.size(); i++)
        {cout << vehCoord[i].first << ", " << vehCoord[i].second << endl;}
        cout << "\n*****************"<<endl;
     
        for(map<char, pair<double,double> >::const_iterator it = vehMap.begin();it != vehMap.end(); ++it)
        {std::cout << it->first << " " << it->second.first << " " << it->second.second << "\n";}
        
        ifstream file5; //read task coordinates
        folder5 = folder+ "/taskCoord.txt"; 
        cout <<"\n"<<folder5<<endl;
        file5.open(folder5); 
        if (file5.is_open())
        {
            char tas=97;
            for (int i=0; i<nTasks; i++)
            {    
                double item5,item7;
                file5 >> item5;
                file5 >> item7;
                taskCoord.push_back(std::make_pair(item5,item7));
                taskMap.insert(make_pair(tas,make_pair(item5,item7)));
                outfile27 <<item5<<"\t"<< item7 <<"\n";
                tas++;
            }
            
        }
        else
        {cout <<"\n taskCoord file not open"<<endl;
         return(0);
        }
        
        cout << "\n*****************"<<endl;
        for(int i = 0; i < taskCoord.size(); i++)
        {cout << taskCoord[i].first << ", " << taskCoord[i].second << endl;}
        cout << "\n*****************"<<endl;
     
        for(map<char, pair<double,double> >::const_iterator it = taskMap.begin();it != taskMap.end(); ++it)
        {std::cout << it->first << " " << it->second.first << " " << it->second.second << "\n";}
        
        totalCoord = taskCoord;
        totalCoord.insert(totalCoord.begin(), vehCoord.begin(), vehCoord.end());
        totalCoord.insert(totalCoord.end(), vehCoord.begin(), vehCoord.end());

        cout << "\n*****************"<<endl;
        for(int i = 0; i < totalCoord.size(); i++)
        {cout << totalCoord[i].first << ", " << totalCoord[i].second << endl;}
        cout << "\n*****************"<<endl;
        
        //calculate DeltaMatrix from the coordinates
        
        for (int i = 0; i < nDim; i++)
                for (int j = 0; j < nDim; j++)
                {
                    DeltaMatrix(i,j) = euclideanDistance(totalCoord[i],totalCoord[j]);
                }
        
        cout <<"\n Delta Matrix generated using the graph coordinates and euclideanDistance \n" << DeltaMatrix << endl; 
        DeltaMatrix.diagonal().array() = 10000000000;
        DeltaMatrix.leftCols(nVehicles) *= 10000000000;
        DeltaMatrix.bottomRows(nVehicles) *= 10000000000;
        DeltaMatrix.topRightCorner(nVehicles,nVehicles) = DeltaMatrix.bottomRightCorner(nVehicles,nVehicles).eval(); 
        DeltaMatrix.bottomLeftCorner(nVehicles,nVehicles) = DeltaMatrix.bottomRightCorner(nVehicles,nVehicles).eval(); 
    
    }
    else 
    {cout << "\n Invalid option: " << argv[7] << "      exiting....\n";
                return(0);
    }
    
    ofstream outfile1;
    std::string createFile1 = "";    
    createFile1 = outputpath + "/" + "tVec" + ".txt";          
    outfile1.open(createFile1.c_str());  
    outfile1.close();
    
    ofstream outfile2;
    std::string createFile2 = "";    
    createFile2 = outputpath + "/" + "deltaMat" + ".txt";          
    outfile2.open(createFile2.c_str());     
    outfile2 << DeltaMatrix << std::endl;
    outfile2.close();
    
    std::string createFile16 = "";    
    createFile16 = outputpath + "/" + "E_local_avg" + ".txt";      
    outfile16.open(createFile16.c_str()); 
    
    std::string createFile17 = "";    
    createFile17 = outputpath + "/" + "E_task_avg" + ".txt";      
    outfile17.open(createFile17.c_str()); 
    
    std::string createFile18 = "";    
    createFile18 = outputpath + "/" + "Analysis" + ".txt";      
    outfile18.open(createFile18.c_str()); 
    
    cout << "\n Updated DeltaMatrix is: \n" << DeltaMatrix << endl;    
    cout << "\n kT_start is "<< kT_start << endl;
    cout << "\n kT_stop is "<< kT_stop << endl;
    cout << "\n kT_in is "<< kT_in << endl;
    cout << "\n ///////////////////////////////////////////////////////////////////// " << endl;
    
    if (!strcmp(argv[8],"-SM"))
       {PState = 1;}
    else if (!strcmp(argv[8],"-exact"))
       {PState = 2;}
    else if (!strcmp(argv[8],"-iterative"))
       {PState = 3;}
    else
       {return(0);}
    
    if (!strcmp(argv[9],"-default"))
       {LoopState = true;}
    else if (!strcmp(argv[9],"-trace"))
       {LoopState = false;}
    else
       {return(0);}
    
    if (!strcmp(argv[10],"-optimal"))
       {VState = 1;}
    else if (!strcmp(argv[10],"-random"))
       {VState = 2;}
    else
       {return(0);}
       
    if (!strcmp(argv[13],"-max"))
       {TaskState = 1;}
    else if (!strcmp(argv[13],"-sum"))
       {TaskState = 2;}
    else
       {return(0);}
          
    if (VState == 1)
    {
    ifstream file3;
    folder3 = folder+ "/Vmat.txt"; 
    file3.open(folder3); 
    if (file3.is_open())
        {
            for (int i = 0; i < nDim; i++)
                for (int j = 0; j < nDim; j++)
                    {
                        double item3;
                        file3 >> item3;
                        VMatrix(i,j) = item3;
                    }
        } 
    else
        {cout <<"\n Vmat file not open"<<endl;
        return(0);}
    }
    
    else if (VState == 2)
    {VMatrix = getVMatrix(nVehicles, nTasks, nDim, rDim, DeltaMatrix); }

    DeltaMatrix.diagonal().array() = 10000000000;
    DeltaMatrix.leftCols(nVehicles) *= 10000000000;
    DeltaMatrix.bottomRows(nVehicles) *= 10000000000;
    DeltaMatrix.topRightCorner(nVehicles,nVehicles) = DeltaMatrix.bottomRightCorner(nVehicles,nVehicles).eval(); 
    DeltaMatrix.bottomLeftCorner(nVehicles,nVehicles) = DeltaMatrix.bottomRightCorner(nVehicles,nVehicles).eval(); 
        
    cout << "\n VMatrix is: \n" << VMatrix <<endl;
    
    NN_algo();    //NN_algo - updating equations
    cout << "\n Annealing done \n" << endl;

    cout << "\n The given DeltaMatrix is: \n" << endl;
    cout << DeltaMatrix << endl;
    cout << "\n //////////////////////////////////////////////////////////////////////////////////////////////////////////// " << endl;
        
    Gnuplot gp;
    Gnuplot gp1;

    gp1 << "Z = `awk 'NR==2 {print NF}' VMatrix` \n";
    gp1 << "unset key \n";
    gp1 << "plot for [i=2:Z] 'VMatrix' using 1:i with linespoints" << endl;

      //  gp1 << "plot for [i=2:Z] \""<< VMatrix <<"\" using 1:i with linespoints";// << endl;
      //  gp << "plot " << filename << " using 1:3:xtic(2) with boxes\n";
      //  gp << "plot \"" << filename << "\" using 1:2:xtic(2) with boxes\n";
    
//     gp2 << "N = `awk 'NR==2 {print NF}' NonVMatrix` \n";
//     gp2 << "unset key \n";
//     gp2 << "plot for [i=2:N] 'NonVMatrix' using 1:i with linespoints" << endl;
     
    cout << "\nThe NonNormalised VMatrix" << endl;
    if (validity(NonNormUpdatedVMatrix) == true)
    {
        cout << "\nInvalid NonNormalised VMatrix" << endl;
    }
    else
    {
        displaySolution(NonNormUpdatedVMatrix, DeltaMatrix, TVec);
        plotString.clear();
    }
    cout << "\n///////////////////////////////////////////////////////////////// " << endl;
    
    cout << "\nThe Last Best VMatrix" << endl;
    if (validity(vMatBest) == true)
    {
        cout << "\nInvalid VMat" << endl;
    }
    else
    {
        displaySolution(vMatBest, DeltaMatrix, TVec);
        plotString.clear();
    }   
    cout << "\n///////////////////////////////////////////////////////////////// " << endl;
    
   // cout << "\nThe VMatrix" << endl;
//     if (validity(UpdatedVMatrix) == true)
//     {
//         cout << "\nInvalid VMatrix" << endl;
//         cout << UpdatedVMatrix << endl;
//     }
//     else
//     {
//         displaySolution(UpdatedVMatrix, DeltaMatrix, TVec);
//         
//         //plot the result
//         if ( (!strcmp(argv[7],"-coordsRead")) || (!strcmp(argv[7],"-coordsRandom")) )
//         {
//             gp << "plot 'taskCoord.txt' using 1:2:(sprintf('(%d, %d)', $1, $2)) with points notitle \n";
//             //create files separate for each vehicle and plot them on the existing plot
//             for (int i = 0; i < nVehicles; i++)
//             {   
//                 ofstream outfile25;
//                 outfile25.open (std::to_string(i) + ".txt");
//                 for ( std::vector<char>::size_type j = 0; j < plotString[i].size(); j++ )
//                 {   
//                     std::map<char,std::pair<double,double>>::iterator it;
//                     it = vehMap.find(plotString[i].at(j));
//                     if (it != vehMap.end() )
//                         { 
//                             outfile25 << (it->second).first << " ";
//                             outfile25 << (it->second).second << endl;
//                         }
//                     it = taskMap.find(plotString[i].at(j));
//                     if (it != taskMap.end() )
//                         { 
//                             outfile25 << (it->second).first << " ";
//                             outfile25 << (it->second).second << endl;
//                         }
//                 }
//             outfile25.close();
//             gp << "replot '" << i <<".txt'" <<" using 1:2:(sprintf('(%d, %d)', $1, $2)) with lines notitle \n";               
//             }
//             gp << "replot 'vehCoord.txt' using 1:2:(sprintf('(%d, %d)', $1, $2)) with points notitle \n";  
//         }
//     }
//     
  //  cout << "\n///////////////////////////////////////////////////////////////// " << endl;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    ofstream outfile14;
    std::string createFile14 = "";    
    createFile14 = outputpath + "/" + "solution" + ".txt";          
    outfile14.open(createFile14.c_str());  
    
    outfile14 << UpdatedVMatrix << std::endl;
    outfile14 << solStrA << std::endl;
    outfile14 << solStrB << std::endl;
    outfile14 << checkTime << std::endl;
    
    cout << "\nThe input parameters are for : " << nVehicles << " vehicles " << nTasks << " tasks taken for " << argv[7] << " values to find the " << argv[13] <<" cost with gamma = " <<g<< " and eta = " << beta << " and Tkappa as " << Tkappa << " with kT_fac as " <<argv[5] << " and kT_start as " << kT_start << " and kT_stop as " << kT_stop << endl;
    printf("\nTotal computational time taken: %.2f\n", (((double)(clock() - tStart)/CLOCKS_PER_SEC)));

    outfile14 << "\nThe input parameters are for : " << nVehicles << " vehicles " << nTasks << " tasks taken for " << argv[7] << " values to find the " << argv[13] <<" cost with gamma = " <<g<< " and eta = " << beta << " and Tkappa as " << Tkappa << " with kT_fac as " <<argv[5] << " and kT_start as " << kT_start << " and kT_stop as " << kT_stop << endl;
    outfile14 << "\nTotal computational time taken:" << ((double)(clock() - tStart)/CLOCKS_PER_SEC) <<endl;
    outfile14.close();
    
    
    }
