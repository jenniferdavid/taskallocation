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
 * 
 * This C++ program is to do task allocation based on Deterministic 
 * Annealing on Potts-spin model with ROS. 
 */

#include <stdlib.h>
#include <cfloat> // for DBL_MAX
#include <cmath>  // for fabs()
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
#include "deterministic_annealing.h"

using namespace std;
using namespace Eigen;

DeterministicAnnealing::DeterministicAnnealing(){}


Eigen::MatrixXd DeterministicAnnealing::Sinkhorn (Eigen::MatrixXd & VMatrix, int nDim, int rDim, int nVehicles) //this function normalises a matrix - makes into doubly stochastic matrix
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::MatrixXd DeterministicAnnealing::normalisation (Eigen::MatrixXd & VMatrix, int &nDim, int &rDim) //this function normalises a matrix - makes into doubly stochastic matrix
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
            {goto LABEL0;}
              
//        if (y != NofNorm) //number of normalisation times
//             {goto LABEL0;}
//        else
//             {cout << "\n Normalised VMatrix is: \n" << VMatrix << endl;
//             cout << "Sum of VMatrix after row normalisation \n" << VMatrix.rowwise().sum() << endl;
//             cout << "Sum of VMatrix after col normalisation \n" << VMatrix.colwise().sum() << endl;}
       return VMatrix;
  }
  
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::MatrixXd DeterministicAnnealing::getVMatrix (int &nVehicles, int &nTasks, int &nDim, int &rDim, Eigen::MatrixXd & DeltaMatrix) //initialize VMatrix
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
      VMatrix = normalisation(VMatrix, nDim, rDim);
	  cout << "VM is done" << endl;
      return VMatrix;
  }
  
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
Eigen::MatrixXd DeterministicAnnealing::syncUpdate (Eigen::MatrixXd & E_local, Eigen::MatrixXd & UpdatedVMatrix, int refix, double kT, int nDim, int rDim, int nVehicles) //Updating mean field equations (VMatrix) along row-wise
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

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::MatrixXf::Index, Eigen::MatrixXf::Index, double> DeterministicAnnealing::calculateLR 
(Eigen::MatrixXd & VMatrix, Eigen::MatrixXd & PMatrix, Eigen::MatrixXd & DeltaMatrix, Eigen::VectorXd & TVec, int nVehicles) //calculate LR from V and PMatrix
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
    
Eigen::MatrixXd DeterministicAnnealing::calculateE (Eigen::VectorXd &leftVec, Eigen::VectorXd & rightVec, Eigen::MatrixXd & VMatrix, 
Eigen::MatrixXd & PMatrix, int & TaskState, bool & LoopState, double kappa, double Okappa, double g, double beta, int iteration, int nDim, 
int rDim, Eigen::MatrixXd DeltaMatrix, Eigen::VectorXd TVec, int nVehicles) //calculate ELocal using L,R and PMatrix
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
        cout << "\nE_task is: \n" << E_task << endl;

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
        cout << "\nE_norm is: \n" << E_norm << endl;
                cout << "\n g is: \n" << g << endl;
                cout << "\n Tkappa is: \n" << Tkappa << endl;
                                cout << "\n eta is: \n" << beta << endl;
        cout << "\nVMatrix is: \n" << VMatrix << endl;


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
    cout << "\nE_local is: \n" << E_local << endl;

    return E_local;
    }
    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::MatrixXd DeterministicAnnealing::UpdateP (Eigen::MatrixXd &UpdatedVMatrix, Eigen::MatrixXd &VMatrix, Eigen::MatrixXd &PMatrix, int &PState) //calculate Updated P
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
    
bool DeterministicAnnealing::validity (Eigen::MatrixXd &Matrix) //finds the validity of the matrix
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

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> DeterministicAnnealing::NN_algo(int nVehicles, int nTasks, int nDim, int rDim, Eigen::MatrixXd DeltaMatrix, double kT_start, double kT_stop, double kT_in, double g, double beta) //the algorithm
  {             
         //calculating initial values L,R, energy for V and P
         Eigen::MatrixXd E_local = MatrixXd::Zero(nDim,nDim);
         Eigen::MatrixXd UpdatedPMatrix = MatrixXd::Zero(nDim,nDim); 
         Eigen::MatrixXd saveV = MatrixXd::Zero(nDim,nDim);
         
         E_task = MatrixXd::Zero(nDim,nDim);
         E_loop = MatrixXd::Zero(nDim,nDim);
         E_norm = MatrixXd::Zero(nDim,nDim);
         vMatBest = MatrixXd::Zero(nDim,nDim);
         I = MatrixXd::Identity(nDim,nDim); 
         DeltaVMatrix = MatrixXd::Zero(nDim,nDim);
         NonNormUpdatedVMatrix = MatrixXd::Zero(nDim,nDim);
         NonNormUpdatedPMatrix = MatrixXd::Zero(nDim,nDim);

         leftVec = VectorXd(nDim);
         rightVec = VectorXd(nDim);
         
         VMatrix = getVMatrix(nVehicles, nTasks, nDim, rDim, DeltaMatrix);
         cout << "\n VMatrix is \n" << VMatrix << endl;

         PMatrix = (I - VMatrix).inverse(); //calculates initial P
         cout << "\ninitial PMatrix is \n" << PMatrix << endl;
         cout << "\ninitial VMatrix is \n" << VMatrix << endl;
         
         calculateLR(VMatrix, PMatrix, DeltaMatrix, TVec, nVehicles); // calculates LR for initial values
         Okappa = kappa;
         cout << "\ninitial leftVec is: \n" << leftVec << endl;
         cout << "\ninitial rightVec is: \n" << rightVec << endl;
         cout << "\nOriginal kappa \n" << Okappa << endl;
         
         int iteration = 0;
         int FLAG = 1;
         kT = kT_start;
         
         E_local = calculateE(leftVec,rightVec,VMatrix,PMatrix,TaskState,LoopState,kappa,Okappa, g,beta,iteration,nDim,rDim,DeltaMatrix,TVec,nVehicles); //calculate initial local energy using VMatrix
         cout << "\ninitial E_local is: \n" << E_local << endl;
         iteration = iteration + 1;
         int refix = 0;
         bool hold = false;
         
         while (FLAG != 0) //iteration starting  //while (!done)
         {	

            cout << "\n ////////////////////////////////////////////////////////////////////////// " << endl;
            cout << "\n" << iteration << " ITERATION STARTING" << endl;
            cout << "\n kT is " << kT << endl;
            cout << "\n Current PMatrix is \n" << PMatrix << endl;
            cout << "\n Current VMatrix is \n" << VMatrix << endl;
            
            saveV = UpdatedVMatrix;
            UpdatedVMatrix = syncUpdate(E_local, VMatrix, refix, kT, nDim, rDim, nVehicles); //calculate updatedVMatrix using ELocal
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
                        //  UpdatedVMatrix = syncUpdate(E_local, VMatrix, refix, kT, nDim, rDim, nVehicles); //reupdate V because of Nan
                        }                            
                    }
                }
           
            NonNormUpdatedVMatrix = UpdatedVMatrix;
           
            //Normalising till the values along the row/columns is zero
            cout << "\n /////////////////////////////////////////////////////////////////////////////////////// " << endl;
            cout << "\n NORMALISATION BEGINS \n" << endl;
            //UpdatedVMatrix = normalisation(UpdatedVMatrix);
            UpdatedVMatrix = Sinkhorn(UpdatedVMatrix, nDim, rDim, nVehicles);
            cout << "\n FINAL NORMALISATION COL SUM is \n" << UpdatedVMatrix.colwise().sum() << endl;
            cout << "\n FINAL NORMALISATION ROW SUM is \n" << UpdatedVMatrix.rowwise().sum() << endl;
            cout << "\n Final row and column normalised UpdatedVMatrix is \n" << UpdatedVMatrix << endl;
            cout << "\n NORMALISATION ENDS \n" << endl;
            cout << "\n /////////////////////////////////////////////////////////////////////////////////////// " << endl;
            
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
            calculateLR(UpdatedVMatrix, UpdatedPMatrix, DeltaMatrix, TVec, nVehicles);   //Calculate LR
            E_local = calculateE(leftVec,rightVec,UpdatedVMatrix,UpdatedPMatrix, TaskState, LoopState, kappa,Okappa, g, beta,iteration, nDim, rDim,DeltaMatrix,TVec,nVehicles); //calculate E
            cout << "\n E_local is: \n " << E_local << endl;

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
                        calculateLR(UpdatedVMatrix, UpdatedPMatrix, DeltaMatrix, TVec, nVehicles);
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
            
            cout << "\n UodatedVMatrix is: \n " << UpdatedVMatrix << endl;
            cout << "\n VMatrix is: \n " << VMatrix << endl;

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

Eigen::MatrixXd DeterministicAnnealing::compute(int nVehicles, int nTasks, int nDim, int rDim, Eigen::MatrixXd DeltaMatrix, double kT_start, double kT_stop, double kT_in, double g, double beta)
  {
    cout << "\nnvehicle is:" << nVehicles<< endl;
    cout << "\ntask is:" << nTasks << endl;
    cout << "\nndim is:" << nDim << endl;
    cout << "\nrdim is:" << rDim << endl;
    
    TVec = VectorXd(nDim);
    for (int i=0; i<nDim; i++)
        {TVec(i) = 1;}
    
    DeltaMatrix.diagonal().array() = 10000000000;
    DeltaMatrix.leftCols(nVehicles) *= 10000000000;
    DeltaMatrix.bottomRows(nVehicles) *= 10000000000;
    DeltaMatrix.topRightCorner(nVehicles,nVehicles) = DeltaMatrix.bottomLeftCorner(nVehicles,nVehicles).eval();       
    
    //subscribe to DeltaMatrix
    cout << "\nDeltaMatrix is:" << DeltaMatrix << endl;
    NN_algo(nVehicles, nTasks, nDim, rDim, DeltaMatrix, kT_start, kT_stop, kT_in, g, beta);    //NN_algo - updating equations
    cout << "\nAnnealing done \n" << endl;

//     Gnuplot gp;
//    
//     gp << "Z = `awk 'NR==2 {print NF}' VMatrix` \n";
//     gp << "unset key \n";
//     gp << "plot for [i=2:Z] 'VMatrix' using 1:i with linespoints" << endl;
    cout << "\n NOW vMatBest is \n" << vMatBest << endl;
    cout << "\n NOW UpdatedVMatrix is \n" << UpdatedVMatrix << endl;

    return UpdatedVMatrix;
}
    
    

    
