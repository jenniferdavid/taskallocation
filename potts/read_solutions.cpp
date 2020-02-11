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

using namespace std;
using namespace Eigen;

int nVehicles;
int nTasks;
int nDim; 
int rDim;
int validMat;
bool sol;

Eigen::VectorXd cTime;
Eigen::VectorXd sub;
Eigen::VectorXd TVec;
Eigen::VectorXd checkTimeVec;
Eigen::MatrixXd vMat;
Eigen::MatrixXd DeltaMatrix;
Eigen::MatrixXd I;

void displaySolution(Eigen::MatrixXd vMat)
{
    std::string veh_alpha = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    std::string task_alpha = "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyz";
    std::string solStrA;
    std::string solStrB;
    
    int indx = 0;
    int indxB = 0;
    int checkTime;
    cTime = VectorXd(nVehicles);
    sub = VectorXd::Ones(nVehicles);

  
        
    for (int i = 0; i < nVehicles; i++)
        {
            for (int j = 0; j < nDim; j++)
            {if (vMat(i,j)==1)
                {indx = j;}
            }
            if (i == 0)
            {
                solStrA = std::string("S") + veh_alpha[i] + std::string(" -> ") + task_alpha[indx-nVehicles];
                solStrB = "max(" + std::to_string(DeltaMatrix(i,indx)) + std::string(" + ") + std::to_string(TVec(indx));
            }

            else
            {
                solStrA = solStrA + std::string(" & S") + veh_alpha[i] + std::string(" -> ") + task_alpha[indx-nVehicles];
                solStrB = solStrB + std::string(", ") + std::to_string(DeltaMatrix(i,indx)) + std::string(" + ") + std::to_string(TVec(indx));
            }
            cTime[i] = DeltaMatrix(i,indx) + TVec(indx);
            
            while (indx <= (nDim-nVehicles-1))
            {
                 for (int j = 0; j < nDim; j++)
                    {if (vMat(indx,j)==1)
                        {indxB = j;}
                    }
               
                if (indxB > (nVehicles+nTasks-1))
                {
                    solStrA = solStrA + std::string(" -> E") + veh_alpha[indxB-nVehicles-nTasks];
                    solStrB = solStrB + std::string(" + ") + std::to_string(DeltaMatrix(indx,indxB));
                    cTime[i] = cTime[i] + DeltaMatrix(indx,indxB);
                    solStrB = solStrB + std::string(" = ") + std::to_string(cTime[i]);            
                }
                else
                { 
                     solStrA = solStrA + std::string(" -> ") + task_alpha[indxB-nVehicles];
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
    checkTime = checkTimeVec.maxCoeff(&maxE);
    
    cout << "\nThis corresponds to the following routing:\n";
    cout << "\n" << solStrA << endl;
    cout << "\n" << solStrB << endl;
    cout << "\n" <<checkTime << endl;
}


int main(int argc, const char* argv[])
{
    nVehicles = atoi(argv[1]);
    nTasks = atoi(argv[2]);
    //std::cout<<"\n argc is "<<argc<<endl;

    if (argc != 3)
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

    DeltaMatrix = MatrixXd::Ones(nDim,nDim);
    vMat = MatrixXd::Ones(nDim,nDim);
    I = MatrixXd::Identity(nDim,nDim); //identity matrix
    TVec = VectorXd(nDim);
           
        ifstream file("tVec.txt");
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
        {cout <<"file not open"<<endl;}
        cout << "\n TVec is: \n" << TVec <<endl;
         
        ifstream file2("deltaMat.txt");
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
        {cout <<"file not open"<<endl;}
        cout << "\n DeltaMatrix is: \n" << DeltaMatrix <<endl;
        
        ifstream file3("sol.txt");
        if (file3.is_open())
        {
            for (int i = 0; i < nDim; i++)
                for (int j = 0; j < nDim; j++)
                    {
                        double item3;
                        file3 >> item3;
                        vMat(i,j) = item3;
                    }
        } 
        else
        {cout <<"file not open"<<endl;}
        cout << "\n vMat is: \n" << vMat <<endl;

       for (int i = 0; i < nDim; i++)
        {for (int j = 0; j < nDim; j++)
        {
            if(vMat(i,j) > 0.7)
                vMat(i,j) = 1;
            else if (vMat(i,j) < 0.3)
                vMat(i,j) = 0;
            else (vMat(i,j) = vMat(i,j));
        }
        }
        
    DeltaMatrix.diagonal().array() = 10000000000;
    DeltaMatrix.leftCols(nVehicles) *= 10000000000;
    DeltaMatrix.bottomRows(nVehicles) *= 10000000000;
    DeltaMatrix.topRightCorner(nVehicles,nVehicles) = DeltaMatrix.bottomLeftCorner(nVehicles,nVehicles).eval();       
    //DeltaMatrix.row(1) += 100* DeltaMatrix.row(0);
    
    cout << "\n Updated DeltaMatrix is: \n" << DeltaMatrix << endl;    
    std::ofstream outfile1 ("tVec.txt");
    std::ofstream outfile2 ("deltaMat.txt");
    std::ofstream outfile3 ("sol.txt");

    outfile1 << TVec << std::endl;
    outfile2 << DeltaMatrix << std::endl;
    outfile3 << vMat << std::endl;

    outfile1.close();
    outfile2.close();
    outfile3.close();

    displaySolution(vMat);//find solutions for each matrix...

    cout << "\nDeltaMatrix is: \n" << DeltaMatrix << endl;    
    cout << "\nTVec is: \n" << TVec << endl;    
    std::cout<<"\n*****************************************************************************"<<endl;
    std::cout<<"*****************************************************************************"<<endl;
}
