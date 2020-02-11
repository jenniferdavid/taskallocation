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
#include <fstream>
#include <algorithm>
#include <time.h>
#include <limits>
#include "gnuplot_iostream.h"
#include <cstring>
#include <sys/stat.h>
#include <ctime>
#include <sys/types.h>
#include <locale.h>
#include <wchar.h>
#include <unistd.h>
#include <csignal>

using namespace std;
using namespace Eigen;

int nVehicles;
int nTasks;
int nDim; 
int rDim;
//int perm = fact(rDim);
int validMat;
bool sol;

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


std::string outputpath = "/home/jendav/Videos/potts_spin_nn/Matrices/inputs/OUTPUTS/";
std::string inputpath = "/home/jendav/Videos/potts_spin_nn/Matrices/inputs/";
std::string veh_alpha = "ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890";
std::string task_alpha = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ`^&*()_+-=[]',./{}|:<>?;~!@#$abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ`^&*()_+-=[]',./{}|:<>?;~!@#$abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ`^&*()_+-=[]',./{}|:<>?;~!@#$abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ`^&*()_+-=[]',./{}|:<>?;~!@#$abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ`^&*()_+-=[]',./{}|:<>?;~!@#$abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ`^&*()_+-=[]',./{}|:<>?;~!@#$abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ`^&*()_+-=[]',./{}|:<>?;~!@#$abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ`^&*()_+-=[]',./{}|:<>?;~!@#$abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ`^&*()_+-=[]',./{}|:<>?;~!@#$abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ`^&*()_+-=[]',./{}|:<>?;~!@#$abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ`^&*()_+-=[]',./{}|:<>?;~!@#$abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ`^&*()_+-=[]',./{}|:<>?;~!@#$abcdef";

/*
int fact(int n)
{
    if (n>1)
        return n*fact(n-1);
    else
        return 1;
}*/

float solutionCheck(Eigen::MatrixXd vMat, Eigen::MatrixXd vMatInv, bool sol)
{
    valVec = VectorXd(2*nVehicles);
    vdVecL = VectorXd(nDim);
    vdVecR = VectorXd(nDim);
    rightVec = VectorXd(nDim);
    leftVec = VectorXd(nDim);

 //  Computing L and R  
    
        vDeltaL = vMat.transpose() * DeltaMatrix;
        vdVecL = vDeltaL.diagonal();
        leftVec = vMatInv.transpose() * (TVec + vdVecL);
                
        vDeltaR = vMat * DeltaMatrix.transpose();
        vdVecR = vDeltaR.diagonal();
        rightVec = vMatInv * (TVec + vdVecR);

        valVec = VectorXd(2*nVehicles);
        valVec << leftVec.tail(nVehicles),rightVec.head(nVehicles);
        cout << "\nvalVec is: \n" << valVec << endl;    

    if (sol == true)
    { 
        MatrixXf::Index imaxl, imaxr;
        double maxleftVec, maxrightVec;
        maxleftVec = leftVec.maxCoeff(&imaxl);
        maxrightVec = rightVec.maxCoeff(&imaxr);
        float maxValue = 0.5 * (leftVec(imaxl) + rightVec(imaxr));
        cout << "\nmaxValue is: \n" << maxValue << endl;    
        return maxValue;
    }
    
    else
    {
        float sumValue = 0.5*(leftVec.sum() + rightVec.sum()); 
        cout << "\nsumValue is: \n" << sumValue << endl;    
        return sumValue;
    }
                
}

float displaySolution(Eigen::MatrixXd vMat)
{
    std::string solStrA;
    std::string solStrB;
    
    int indx = 0;
    int indxB = 0;
    float checkTime;
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
    checkTime = checkTimeVec.maxCoeff(&maxE) - nVehicles;
    
    cout << "\nThis corresponds to the following routing:\n";
    cout << "\n" << solStrA << endl;
    cout << "\n" << solStrB << endl;
    cout << "\n" <<checkTime << endl;
    return checkTime;
}

bool isValid(Eigen::MatrixXd x)
{
     Eigen::VectorXd sumr = VectorXd::Zero(nDim); 
     sumr = x.rowwise().sum();
     //std::cout<<"\n"<<sumr<<endl;
     if (std::isnan(sumr(1)))
             {  
                // std::cout<<"\n has loops so invalid"<<endl;
                 return false;  
             }
    else
             {  
               // std::cout<<"\n no loops so valid"<<endl;
                 return true;
             }
}

int main(int argc, const char* argv[])
{
    int x;
    nVehicles = atoi(argv[1]);
    nTasks = atoi(argv[2]);
    x = atoi(argv[4]);

    time_t rawtime;
    struct tm * timeinfo;
    char buffer[40];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer, sizeof(buffer), "%d_%m_%Y_%I_%M_%S", timeinfo);
    
    if (argc != 5)
    {
        std::cout<<"\n Less input options... exiting "<<endl;
        return(0);
    }

    std::cout<<"\n nVehicles is "<<nVehicles<<endl;
    std::cout<<"\n nTasks is "<<nTasks<<endl;
    std::cout<<"\n No of reqd validsols are "<<x<<endl;

    nDim = 2*nVehicles + nTasks;; 
    rDim = nVehicles + nTasks;

    std::cout<<"\n nDim is "<<nDim<<endl;
    std::cout<<"\n rDim is "<<rDim<<endl;
    
    std::string data = "M=" + std::to_string(nVehicles) +"_N="+std::to_string(nTasks);
    outputpath.append(std::string(data));
    std::string at = " AT TIME ";
    outputpath.append(std::string(at));
    outputpath.append(std::string(buffer));
    mkdir(outputpath.c_str(),S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH );
   
    DeltaMatrix = MatrixXd::Ones(nDim,nDim);
    I = MatrixXd::Identity(nDim,nDim); //identity matrix
    TVec = VectorXd(nDim);
    
    std::string folder = inputpath + data;
    std::string folder1, folder2;

    if (!strcmp(argv[3],"-read"))
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
        cout << "\n DeltaMatrix is: \n" << DeltaMatrix <<endl;
    }
    
    else 
    {cout << "\n Invalid option: " << argv[3] << "      exiting....\n";
                return(0);
    }
    
    DeltaMatrix.diagonal().array() = 10000000000;
    DeltaMatrix.leftCols(nVehicles) *= 10000000000;
    DeltaMatrix.bottomRows(nVehicles) *= 10000000000;
    DeltaMatrix.topRightCorner(nVehicles,nVehicles) = DeltaMatrix.bottomLeftCorner(nVehicles,nVehicles).eval();       
    //DeltaMatrix.row(1) += 100* DeltaMatrix.row(0);
    //DeltaMatrix.triangularView<Lower>() *= 10000000000; //for predefined order
    //DeltaMatrix.topRows(nVehicles) = 0; //for vehicles 
    //DeltaMatrix.rightCols(nVehicles) = 0; //that have same initial cost

    ofstream outfile1;
    std::string createFile1 = "";    
    createFile1 = outputpath + "/" + "tVec" + ".txt";          
    outfile1.open(createFile1.c_str());     
    outfile1 << TVec << std::endl;
    outfile1.close();
    
    ofstream outfile2;
    std::string createFile2 = "";    
    createFile2 = outputpath + "/" + "deltaMat" + ".txt";          
    outfile2.open(createFile2.c_str());     
    outfile2 << DeltaMatrix << std::endl;
    outfile2.close();
    
    cout << "\n Updated DeltaMatrix is: \n" << DeltaMatrix << endl;    
    cout << "\n ///////////////////////////////////////////////////////////////////// " << endl;
    
    std::vector<int> v; 
    std::vector<Eigen::MatrixXd> MVector;
    std::vector <float> costValue; 
    std::vector<Eigen::MatrixXd> PVector; 
    validMat = 0;
    int y = 0;
    for(int i=0;i<rDim;++i)
        {v.push_back(i*1);}
    std::srand((unsigned int) time(0));
    std::random_shuffle (v.begin(), v.end() );
    do{
                y++;
                std::cout<<"\n********************************************************************************"<<endl;
                std::cout<<"\n********************************************************************************"<<endl;
                std::cout<<"\n ITERATION NO: "<< y <<endl;
                if (validMat==x) //sometimes the validmats are not x because of the permutation of random sequences
                {break;}
                std::srand((unsigned int) time(0));
                std::random_shuffle (v.begin(), v.end());
                Eigen::MatrixXd subMat = MatrixXd::Zero(rDim,rDim);
                Eigen::MatrixXd vMat = MatrixXd::Zero(nDim,nDim);
                Eigen::MatrixXd vMatInv = MatrixXd::Zero(nDim,nDim);

                for (int i=0;i<rDim;++i)
                    {std::cout<<"\n"<<v[i]<<endl;}
                for (int j=0;j<v.size();++j)
                    {
                        for (int i=0;i<rDim;++i)
                            {
                                subMat(i,v[j,i]) = 1;
                            }
                    }
              //  std::cout<<"\n"<<subMat<<endl;
                vMat.block(0,nVehicles,rDim,rDim) = subMat;
                vMat.topRightCorner(nVehicles,nVehicles) *=0;
                vMat.diagonal().array() = 0;
                std::cout<<"\n"<<vMat<<endl;
                std::cout<<"\n/////////////////////////////////////////////////////"<<endl;

                int check = 0;
                for (int i = 0; i < nDim; i++)
                {                    
                    for (int j = 0; j< nDim; j++)
                    {   
                        if (vMat(i,j)==1)
                        {
                          check = check+1;
                        }
                    }
                }
                
                if (check == rDim)
                {
                    vMatInv = (I - vMat).inverse();
                    //std::cout<<"\n vMatInv is \n"<< vMatInv << endl;
                    if (isValid(vMatInv))
                        {
                        validMat = validMat+1;
                        std::cout<<"\n validMat no is "<< validMat << endl;
                        std::cout<<"\n validMat is \n"<< vMat << endl;
                        MVector.push_back(vMat);
                        PVector.push_back(vMatInv);
                        }
                }
                else
                {
                    std::cout<<"\n XXXXXXXX invalidMat XXXXXXXX \n"<< endl;
                }
          }
      while(std::next_permutation(v.begin(),v.end()));
      std::cout<<"\n ITERATIONS DONE "<<endl;
      std::cout<<"\n********************************************************************************"<<endl;
      ofstream outfile3; 
      std::string createFile3 = "";    
      createFile3 = outputpath + "/" + "validsols" + ".txt";          
      outfile3.open(createFile3.c_str());    
       
      for (int i = 0;i <validMat;i++)
      {
       std::cout<<"\nValid solution no "<< i+1 << endl;
       std::cout<<"\n"<<MVector[i]<<endl;
    //  std::cout<<"\nCorresponding Propagator is " << endl;
    //  std::cout<<"\n"<<PVector[i]<<endl;
       float cost = displaySolution(MVector[i]);//find solutions for each matrix...
       outfile3 << cost << endl;
       std::cout<<"\n********************************************************************************"<<endl;
      }
    
       std::cout<<"*****************************************************************************"<<endl;
}
