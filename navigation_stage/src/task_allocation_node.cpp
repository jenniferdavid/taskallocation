/* Task Allocation Node
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
 * INPUT: One of the methods - DA or SA or Heuristic with arguments
 * OUTPUT: Task allocation solution
 */

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
#include "navi_msgs/OdomArray.h"
#include "nav_msgs/Odometry.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Float64MultiArray.h"
#include <tf/transform_datatypes.h>
#include "heuristic.h"
#include "deterministic_annealing.h"
#include "simulated_annealing.h"


using namespace std;
using namespace Eigen;

std::vector<std::pair<std::string,std::tuple<double,double,double>> > robotCoord;
std::string solStrA;
std::string solStrB;
std::string veh_alpha = "ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890";
std::string task_alpha = "abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@";

std::vector<std::vector<char> >plotString;
double checkTime;
Eigen::VectorXd cTime;
Eigen::VectorXd checkTimeVec;
Eigen::VectorXd sub;
std::vector<char> plotVehicle;
clock_t tStart = clock();

int nVehicles;      
int nTasks;       
int nDim; 
int rDim; 
Eigen::MatrixXd DeltaMatrix;
Eigen::VectorXd TVec; 
Eigen::MatrixXd VMatrix;

navi_msgs::Goals goalMsg;
navi_msgs::GoalsList goalListMsg;
navi_msgs::listTasks stringD1;
navi_msgs::listIntTasks intD2;
navi_msgs::nameTasks stringD3;
navi_msgs::Problem probmsg;
navi_msgs::OdomArray coordsmsg;

std_msgs::Float64MultiArray deltaMatMsg;

ros::Publisher taskPub;   

boost::shared_ptr<navi_msgs::OdomArray const> coordsmsgptr;
boost::shared_ptr<navi_msgs::Problem const> probmsgptr;
boost::shared_ptr<std_msgs::Float64MultiArray const> deltamsgptr;

std::vector<std::vector<char> > displaySolution(Eigen::MatrixXd &VMatrix, Eigen::MatrixXd &DeltaMatrix, Eigen::VectorXd &TVec, int &nVehicles, int &nTasks, int &nDim, int &rDim)
    {   
    
        int indx = 0;
        int indxB = 0;
        cTime = VectorXd::Ones(nVehicles);
        sub = VectorXd::Ones(nVehicles);

        cout << "\nTHE SOLUTION is: \n" << endl;
        cout << VMatrix << endl;
        
        for (int i = 0; i < nDim; i++)
        {
         for (int j = 0; j < nDim; j++)
            {
            if(VMatrix(i,j) > 0.8)
                { VMatrix(i,j) = 1;}
            else if (VMatrix(i,j) < 0.2)
                {VMatrix(i,j) = 0;}
            else
                {VMatrix(i,j) = 0.5;}
            }
        }
        cout << "\nTHE approximated SOLUTION is: \n" << endl;
        cout << VMatrix << endl;
        
        for (int i = 0; i < nVehicles; i++)
            {
                for (int j = 0; j < nDim; j++)
                    {if (VMatrix(i,j)==1)
                        {indx = j;}}
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

                while (indx <= ((nDim-nVehicles)-1))
                {
                    for (int j = 0; j < nDim; j++)
                        {if (VMatrix(indx,j)==1)
                            {indxB = j;}}
               
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
        cout << "\nThe tasks are ordered as:\n";
        cout << "\n" <<solStrA << endl;
        cout << "\n" <<solStrB << endl;
        cout << "\n" <<checkTime << endl;
        printf("\nTotal computational time taken: %.2f\n", (((double)(clock() - tStart)/CLOCKS_PER_SEC)));

        for ( std::vector<std::vector<char>>::size_type i = 0; i < plotString.size(); i++ )
            {for ( std::vector<char>::size_type j = 0; j < plotString[i].size(); j++ )
                {std::cout << plotString[i][j] << ' ';}
                std::cout << std::endl;} 
       return plotString;
    }

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "TaskAllocationNode"); 
    ros::NodeHandle n;
    
    std::string method;
    double kT_start, kT_end, kT_step, gamma, eta;
    
    n.getParam("/TaskAllocationNode/method", method);

    kT_start = atof(argv[1]);
    kT_end = atof(argv[2]);
    kT_step = atof(argv[3]);
    gamma = atof(argv[4]);
    eta = atof(argv[5]);
    
    // The inputs can be
    // rosrun navigation_stage task_allocation_node _method:=DeterministicAnnealing 100 1 0.9 1 1
    // rosrun navigation_stage task_allocation_node _method:=SimulatedAnnealing 100 1 0.9 0 0 // gamma and eta not applicable
    // rosrun navigation_stage task_allocation_node _method:=Heuristic 0 0 0 0 0 // eta is 0 (DA), 1 (SA) and 2 (Heuristic)
    
    cout << "1: " << kT_start << endl;
    cout << "2: " << kT_end << endl;
    cout << "3: " << kT_step << endl;
    cout << "4: " << gamma << endl;
    cout << "5: " << eta << endl;
        
    //ros::Duration(5).sleep();
    probmsgptr = ros::topic::waitForMessage<navi_msgs::Problem>("/problem",ros::Duration(10));
        if (probmsgptr == NULL)
            {ROS_INFO("No problem input messages received");}
        else
            {
                probmsg = * probmsgptr; 
                nVehicles = probmsg.nRobots;
                rDim = probmsg.nModels;     
                int t = rDim - nVehicles;
                nTasks = t;
                nDim = 2*nVehicles + t;
            }
    
    //Wait till the task list available
    deltamsgptr = ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/deltamat",ros::Duration(10));
        if (deltamsgptr == NULL)
            {ROS_INFO("No deltamat messages received");}
        else
            {deltaMatMsg = * deltamsgptr; 
            DeltaMatrix = MatrixXd::Ones(nDim,nDim);
            int k = 0;
            for (int i=0;i<nDim;i++){
                for (int j=0;j<nDim;j++){   
                    DeltaMatrix(i,j) = deltaMatMsg.data[k];
                    k++;}}
            }
            
    coordsmsgptr = ros::topic::waitForMessage<navi_msgs::OdomArray>("/coords",ros::Duration(10));
        if (coordsmsgptr == NULL)
            {ROS_INFO("No coords messages received");}
        else
            {coordsmsg = * coordsmsgptr; }
            double x,y,z; std::string name;
        for (int i = 0; i < nDim; i++)
                {
                    name = coordsmsg.coords[i].header.frame_id;
                    x = coordsmsg.coords[i].pose.pose.position.x;
                    y = coordsmsg.coords[i].pose.pose.position.y;
                    z = coordsmsg.coords[i].pose.pose.orientation.w;
                    robotCoord.push_back(std::make_pair(name,std::make_tuple(x,y,z)));
                }
            
    VMatrix = MatrixXd::Zero(nDim,nDim);
    
    if (method == "Heuristic")
    {Heuristic h;
	VMatrix = h.compute(nVehicles,nTasks,nDim,rDim,DeltaMatrix, eta);}

    else if (method == "DeterministicAnnealing")
    {DeterministicAnnealing h;
    VMatrix = h.compute(nVehicles,nTasks,nDim,rDim,DeltaMatrix,kT_start, kT_end, kT_step, gamma, eta);}
    
    else
    {SimulatedAnnealing h;
    VMatrix = h.compute(nVehicles,nTasks,nDim,rDim,DeltaMatrix,kT_start, kT_end, kT_step);}
    
    TVec = VectorXd(nDim);
    for (int i=0; i<nDim; i++) {TVec(i) = 1;}
    
    plotString = displaySolution(VMatrix, DeltaMatrix, TVec, nVehicles, nTasks, nDim, rDim);
    
    ros::Publisher taskPub = n.advertise<navi_msgs::GoalsList>("/tasks", 1000);
    ros::Rate loop_rate(10);
   
    int q = 0;

    while (q < nVehicles)
    {
        goalListMsg.input.nRobots = nVehicles;
        goalListMsg.input.nTasks = nTasks;
        goalListMsg.input.nModels = rDim;
        goalListMsg.input.nDropoffs = nVehicles;
        goalListMsg.input.nTotalmodels = nDim;
    
        goalMsg.robotName = q;
        goalMsg.name = "robot"+std::to_string(q); 
        
        //publish no of tasks
        goalMsg.tasks = plotString[q].size() - 1; 

        for (int j=1; j< plotString[q].size();j++)       //for each robot -> the list of tasks
             {   
                 string s(1,plotString[q][j]);           //convert char to string
                 stringD1.ltask = s;  
                 /*publish string of task*/
                 goalMsg.lT.push_back(stringD1);  
                 int x = (int)plotString[q][j];          //int of all tasks

                if (j != plotString[q].size() -1)
                {    
                    int y = x - 96 - 1;                      //int of all tasks
                    intD2.liTask = y;
                    /*publish int of tasks*/
                    goalMsg.lIT.push_back(intD2);  

                    std::string t = "/task_"+std::to_string(y)+"/odom";
                    stringD3.namTask = t;     
                    /*publish string+int tasks*/
                    goalMsg.nT.push_back(stringD3);
                    
                    for(int i = nVehicles; i<robotCoord.size(); i++)  //loop through all the poses of tasks to find the right one 
                    {    
                        if (t.compare(robotCoord[i].first) != 0)
                            {
                                //nothing
                            }
                        else
                            {
                                nav_msgs::Odometry msg;
                               /* msg.pose.pose.position.x = std::get<0>(robotCoord[i].second);
                                msg.pose.pose.position.y = std::get<1>(robotCoord[i].second);
                                msg.pose.pose.orientation.z = std::get<2>(robotCoord[i].second);
                               */ 
                                msg.pose.pose.position.x = coordsmsg.coords[i].pose.pose.position.x;
                                msg.pose.pose.position.y = coordsmsg.coords[i].pose.pose.position.y;
                                msg.pose.pose.position.z = coordsmsg.coords[i].pose.pose.position.z;
                                msg.pose.pose.orientation.x = coordsmsg.coords[i].pose.pose.orientation.x;
                                msg.pose.pose.orientation.y = coordsmsg.coords[i].pose.pose.orientation.y;
                                msg.pose.pose.orientation.z = coordsmsg.coords[i].pose.pose.orientation.z;
                                msg.pose.pose.orientation.w = coordsmsg.coords[i].pose.pose.orientation.w;
                    
                                msg.twist.twist.linear.x = coordsmsg.coords[i].twist.twist.linear.x;
                                msg.twist.twist.linear.y = coordsmsg.coords[i].twist.twist.linear.y;
                                msg.twist.twist.linear.z = coordsmsg.coords[i].twist.twist.linear.z;
                                msg.twist.twist.angular.x = coordsmsg.coords[i].twist.twist.angular.x;
                                msg.twist.twist.angular.y = coordsmsg.coords[i].twist.twist.angular.y;
                                msg.twist.twist.angular.z = coordsmsg.coords[i].twist.twist.angular.z;
                                
                                goalMsg.pList.coords.push_back(msg);
                            }
                      }
                }
                else
                {
                    int z = x - 64 + nTasks - 1; //int of all tasks                    
                    std::string dt = "/dropoff_"+std::to_string(z)+"/odom";
                    stringD3.namTask = dt;
                    goalMsg.nT.push_back(stringD3);

                    //pose of the dropoff
                    for (int i = nTasks; i<robotCoord.size(); i++)
                    {
                        if (dt.compare(robotCoord[i].first) == 0)
                            {
                                nav_msgs::Odometry msg;
//                                 msg.pose.pose.position.x = std::get<0>(robotCoord[i].second);
//                                 msg.pose.pose.position.y = std::get<1>(robotCoord[i].second);
//                                 msg.pose.pose.orientation.z = std::get<2>(robotCoord[i].second);
                                msg.pose.pose.position.x = coordsmsg.coords[i].pose.pose.position.x;
                                msg.pose.pose.position.y = coordsmsg.coords[i].pose.pose.position.y;
                                msg.pose.pose.position.z = coordsmsg.coords[i].pose.pose.position.z;
                                msg.pose.pose.orientation.x = coordsmsg.coords[i].pose.pose.orientation.x;
                                msg.pose.pose.orientation.y = coordsmsg.coords[i].pose.pose.orientation.y;
                                msg.pose.pose.orientation.z = coordsmsg.coords[i].pose.pose.orientation.z;
                                msg.pose.pose.orientation.w = coordsmsg.coords[i].pose.pose.orientation.w;
                    
                                msg.twist.twist.linear.x = coordsmsg.coords[i].twist.twist.linear.x;
                                msg.twist.twist.linear.y = coordsmsg.coords[i].twist.twist.linear.y;
                                msg.twist.twist.linear.z = coordsmsg.coords[i].twist.twist.linear.z;
                                msg.twist.twist.angular.x = coordsmsg.coords[i].twist.twist.angular.x;
                                msg.twist.twist.angular.y = coordsmsg.coords[i].twist.twist.angular.y;
                                msg.twist.twist.angular.z = coordsmsg.coords[i].twist.twist.angular.z;
                                goalMsg.pList.coords.push_back(msg);
                            }
                    }
                }
             goalMsg.pList.header.stamp = ros::Time::now(); // timestamp of creation of the msg
             goalMsg.pList.header.frame_id = "map" ;
            }
    
     goalListMsg.list.push_back(goalMsg);
     goalMsg.lT.clear();  
     goalMsg.lIT.clear();
     goalMsg.nT.clear();
     goalMsg.pList.coords.clear();
     q++;
    }
     while (ros::ok())
        {
            taskPub.publish(goalListMsg); //publishes
            loop_rate.sleep();
        }
    return 0;
}


    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
