/* Simulated Annealing
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
 * 
 * This C++ program is to do task allocation based on Simulated 
 * Annealing with ROS. 
 */
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include <algorithm>
#include <bitset>
#include <limits>
#include <fstream>
#include <math.h>
#include <iomanip> // needed for setw(int)
#include <cstdlib>
#include <Eigen/Dense>
#include <Eigen/LU> 
#include <cmath>
#include <time.h>
#include <limits>
#include <stdlib.h>
#include <cfloat> // for DBL_MAX
#include <cmath>  // for fabs()
#include "simulated_annealing.h"

using namespace std;
using namespace Eigen;

SimulatedAnnealing::SimulatedAnnealing(){}

/////////////////////////////////////////////////////////////////////////////////////////////////

bool SimulatedAnnealing::isValid(Eigen::MatrixXd x)
{
     Eigen::VectorXd sumr = VectorXd::Zero(nDim); 
     sumr = x.rowwise().sum();
     std::cout<<"\n"<<sumr<<endl;
     if (std::isnan(sumr(1)))
             {  
                 std::cout<<"\n has loops so invalid"<<endl;
                 return false;  
             }
    else
             {  
                 std::cout<<"\n no loops so valid"<<endl;
                 return true;
             }
}

/////////////////////////////////////////////////////////////////////////////////////////////////

float SimulatedAnnealing::calculateLR(Eigen::MatrixXd VMatrix, Eigen::MatrixXd PMatrix, Eigen::MatrixXd DeltaMatrix) //calculate LR from V and PMatrix
   {
        vDeltaL = VMatrix.transpose() * DeltaMatrix;
        //cout << "\n vDeltaL is: \n" << vDeltaL << endl;
        vdVecL = vDeltaL.diagonal();
        //cout << "\n vdVecL is: \n" << vdVecL << endl;
        leftVec = PMatrix.transpose() * (TVec + vdVecL);
        //cout << "\n leftVec is: \n" << leftVec << endl;

        vDeltaR = VMatrix * DeltaMatrix.transpose();
        //cout << "\n vDeltaR is: \n" << vDeltaR << endl;
        vdVecR = vDeltaR.diagonal();
        //cout << "\n vdVecR is: \n" << vdVecR << endl;
        rightVec = PMatrix * (TVec +vdVecR);
        //cout << "\n rightVec is: \n" << rightVec << endl;

        MatrixXf::Index imaxl, imaxr;
        double maxleftVec, maxrightVec;
        maxleftVec = leftVec.maxCoeff(&imaxl);
        maxrightVec = rightVec.maxCoeff(&imaxr);


        costValue = 0.5 * (leftVec(imaxl) + rightVec(imaxr));
        cout << "\n ///////////////////////////////////////////////////////////////////// " << endl;
        
        return costValue;
   }


Eigen::MatrixXd SimulatedAnnealing::SA_algo(int &nVehicles, int &nTasks, int &nDim, int &rDim, Eigen::MatrixXd & DeltaMatrix, double &kT_start, double &kT_end, double &kT_step)
    
{
    cout << "\n kT_start is "<< kT_start << endl;
    cout << "\n kT_end is "<< kT_end << endl;
    cout << "\n kT_step is "<< kT_step << endl;
    cout << "\n ///////////////////////////////////////////////////////////////////// " << endl;
    
    std::vector<int> v; 
    double costValueBest = 10000000000; 
   
    for(int i=0;i<rDim;++i)
        {v.push_back(i*1);}              
    
    Eigen::MatrixXd subMat = MatrixXd::Zero(rDim,rDim);
    Eigen::MatrixXd vMat = MatrixXd::Zero(nDim,nDim);
    Eigen::MatrixXd vMatBest = MatrixXd::Zero(nDim,nDim);
    Eigen::MatrixXd vMatInv = MatrixXd::Zero(nDim,nDim);
    Eigen::MatrixXd I = MatrixXd::Identity(nDim,nDim);
    Eigen::VectorXd L(nDim);
    Eigen::VectorXd R(nDim);

    for (int j=0;j<v.size();++j)
        {
            for (int i=0;i<rDim;++i)
                {
                    subMat(i,v[j,i]) = 1;
                }
        }
                
    std::cout<<"\nsubMat is: \n"<<subMat<<endl;
    vMat.block(0,nVehicles,rDim,rDim) = subMat;
    vMat.topRightCorner(nVehicles,nVehicles) *=0;
    vMat.diagonal().array() = 0;
    std::cout<<"\nvMat is: \n"<<vMat<<endl;

    vMatInv = (I - vMat).inverse();
    std::cout<<"\n vMatInv is \n"<< vMatInv << endl;
    if (isValid(vMatInv))
        {
            costValue = calculateLR(vMat, vMatInv, DeltaMatrix);
            if (costValue < costValueBest)
               {vMatBest = vMat;
               costValueBest = costValue;}
        }
    int iteration = 1;
    int indexA =0; int indexB = 0;
    kT = kT_start;
         int FLAG = 1;

    while(FLAG != 0)
      {
          cout << "\n" << iteration << " ITERATION STARTING" << endl;
          cout << "\n kT is " << kT << endl;
          cout<<"\n vMat is \n"<< vMat << endl;
              STEP: 
                  int ran = rand() % 2;
                 if (ran == 1) //change rows to create newvMat
                 {
                     cout << "Row swap" << endl;
                     std::vector<int> rowVector;
                     for (int i=0; i<(nVehicles+nTasks); ++i) 
                     {rowVector.push_back(i);}
                     std::random_shuffle (rowVector.begin(), rowVector.end() );
                     for (std::vector<int>::iterator it=rowVector.begin(); it!=rowVector.end(); ++it)

                     for (int j=0;j<nDim;j++)
                     {
                     if (vMat(rowVector[0],j)==1)
                        {indexA = j;
                        }
                     }
                     vMat(rowVector[0],indexA) = 0;   

                     for (int j=0;j<nDim;j++)
                     {
                     if (vMat(rowVector[1],j)==1)
                        {indexB = j;
                        }
                     }
                     vMat(rowVector[1],indexB) = 0;

                     vMat(rowVector[1],indexA) = 1;
                     vMat(rowVector[0],indexB) = 1;
                 }
                 else
                 {
                     cout << "Column swap" << endl; //change rows to create newvMat
                     std::vector<int> colVector;
                     for (int i=nVehicles; i<nDim; ++i) 
                     {colVector.push_back(i);}
                     std::random_shuffle (colVector.begin(), colVector.end() );
                     for (std::vector<int>::iterator it=colVector.begin(); it!=colVector.end(); ++it)

                     for (int j=0;j<nDim;j++)
                     {
                     if (vMat(j,colVector[0])==1)
                        {indexA = j;
                        }
                     }
                     vMat(indexA,colVector[0]) = 0;   

                     for (int j=0;j<nDim;j++)
                     {
                     if (vMat(j,colVector[1])==1)
                        {indexB = j;
                        }
                     }
                     vMat(indexB,colVector[1]) = 0;

                     vMat(indexA,colVector[1]) = 1;
                     vMat(indexB,colVector[0]) = 1;
                 }
            std::cout<<"\n new vMat is \n"<< vMat << endl;
            vMatInv = (I - vMat).inverse();
            std::cout<<"\n vMatInv is \n"<< vMatInv << endl;
            if (isValid(vMatInv))
                {
                costValue = calculateLR(vMat, vMatInv, DeltaMatrix);
                cout << "\n new costValue is: " << costValue << endl;
                cout << "\n Best costValue is: " << costValueBest << endl;
                if (costValue < costValueBest)
                    {vMatBest = vMat;
                    costValueBest = costValue;
                    cout << "\n better solution found " << endl;
                    }
                kT *= kT_step;
                cout << "\n new kT is: " << kT << endl;
                cout << "\n" << iteration << " ITERATION DONE" << endl;
                iteration = iteration + 1;
                if (kT < kT_end) 
                FLAG = 0;
                }
            else
            {   cout << "\n Invalid solution found.. Rechecking again.. " << endl;
                goto STEP;}
     }    
     cout << costValueBest << endl;
     cout << vMatBest << endl;
     return vMatBest;
}

Eigen::MatrixXd SimulatedAnnealing::compute(int nVehicles, int nTasks, int nDim, int rDim, Eigen::MatrixXd DeltaMatrix, double kT_start, double kT_end, double kT_step)
{
    Eigen::MatrixXd Best = MatrixXd::Zero(nDim,nDim);
    cout << "\nnvehicle is:" << nVehicles<< endl;
    cout << "\ntask is:" << nTasks << endl;
    cout << "\nndim is:" << rDim << endl;
    cout << "\nrdim is:" << nDim << endl;
    
    TVec = VectorXd(nDim);
    for (int i=0; i<nDim; i++)
        {TVec(i) = 1;}
    
    DeltaMatrix.diagonal().array() = 10000000000;
    DeltaMatrix.leftCols(nVehicles) *= 10000000000;
    DeltaMatrix.bottomRows(nVehicles) *= 10000000000;
    DeltaMatrix.topRightCorner(nVehicles,nVehicles) = DeltaMatrix.bottomLeftCorner(nVehicles,nVehicles).eval();       
    
    //subscribe to DeltaMatrix
    cout << "\nDeltaMatrix is:" << DeltaMatrix << endl;

    Best = SA_algo(nVehicles, nTasks, nDim, rDim, DeltaMatrix, kT_start, kT_end, kT_step);    //NN_algo - updating equations
    cout << "\nAnnealing done \n" << endl;

//     Gnuplot gp;
//    
//     gp << "Z = `awk 'NR==2 {print NF}' VMatrix` \n";
//     gp << "unset key \n";
//     gp << "plot for [i=2:Z] 'VMatrix' using 1:i with linespoints" << endl;

    return Best;
}
