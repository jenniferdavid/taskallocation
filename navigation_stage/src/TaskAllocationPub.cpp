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
//#include "gnuplot_iostream.h"
#include <cstring>
#include <sys/stat.h>
#include <ctime>
#include <sys/types.h>
#include <locale.h>
#include <wchar.h>
#include <unistd.h>
#include <csignal>
#include <map>
#include "Hungarian.h"
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

using namespace std;
using namespace Eigen;


class SubPub
{
    public:
    std::vector<std::pair<std::string,std::tuple<double,double,double>> > totalCoord;
std::string solStrA;
std::string solStrB;
std::string veh_alpha = "ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890";
std::string task_alpha = "abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@";

std::vector<char> plotVehicle;
std::vector<std::vector<char> >plotString;

int nVehicles;      
int nTasks;       
int nDim; 
int rDim; 
    
double checkTime;
Eigen::VectorXd cTime;
Eigen::VectorXd checkTimeVec;
Eigen::VectorXd sub;
Eigen::MatrixXd DeltaMatrix;
Eigen::MatrixXd CostMatrix;
Eigen::MatrixXd TaskMatrix;
Eigen::VectorXd TVec; 

navi_msgs::Goals data;
navi_msgs::GoalsList msg;
navi_msgs::listTasks stringD1;
navi_msgs::listIntTasks intD2;
navi_msgs::nameTasks stringD3;
geometry_msgs::PoseArray poseArrayMsg;
geometry_msgs::Pose poseMsg;

    //constructor function that publishes and subscribes
    SubPub()
    {
        probSub = n.subscribe("/problem", 100, &SubPub::probCallback,this);
        deltamatSub = n.subscribe("/deltamat", 100, &SubPub::deltaMatCallback,this);
        coordSub = n.subscribe("/coords", 100, &SubPub::coordsCallback,this);
        ros::Duration(10).sleep();
        goalPub = n.advertise<navi_msgs::Goals>("/goals", 100);
        taskPub = n.advertise<navi_msgs::GoalsList>("/tasks", 100);
    }
        
void deltaMatCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {   
        DeltaMatrix = MatrixXd::Ones(nDim,nDim);
        int k = 0;
        for (int i=0;i<nDim;i++)
        {
            for (int j=0;j<nDim;j++)
            {   
                DeltaMatrix(i,j) = msg->data[k];
                k++;
            }
        }
    }

void probCallback(const navi_msgs::Problem::ConstPtr& msg)
    {
        nVehicles = msg->nRobots;
        nTasks= msg->nTasks;
        nDim = msg->nTotalmodels;
        rDim = msg->nModels;
    }
        
    //callback for subscribing /coords 
void coordsCallback(const navi_msgs::ItemStruct::ConstPtr& msg)
    {
        for (int i=0; i< nDim; ++i) //msg->coords.size()
        {
            const navi_msgs::Item &data = msg->coords[i];
            totalCoord.push_back(std::make_pair(data.name,(std::make_tuple(data.x, data.y, data.yaw))));
        }
    }

    //parses the solution
std::tuple<std::string, std::string, int, std::vector <std::vector <char>> > displaySolution(Eigen::MatrixXd &VMatrix, Eigen::MatrixXd &DeltaMatrix, Eigen::VectorXd &TVec) 
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
            
                while (indx <= (nDim-nVehicles-1))
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
        cout << "The tasks are ordered as:\n";
        cout << "\n" <<solStrA << endl;
        cout << "\n" <<solStrB << endl;
        cout << "\n" <<checkTime << endl;
        
        for ( std::vector<std::vector<char>>::size_type i = 0; i < plotString.size(); i++ )
            {for ( std::vector<char>::size_type j = 0; j < plotString[i].size(); j++ )
                {std::cout << plotString[i][j] << ' ';}
                std::cout << std::endl;} 
       // publish(nVehicles, plotString);
        return std::make_tuple(solStrA, solStrB, checkTime, plotString);    
    }

void publish(int nVehicles, std::vector<std::vector<char> >plotString)
{
    int q = 0;
    msg.input.nRobots = nVehicles;
    msg.input.nTasks = nTasks;
    msg.input.nModels = rDim;
    msg.input.nTotalmodels = nDim;
    msg.input.nDropoffs = nVehicles;

    while (q < nVehicles)
    {
        data.robotName = q;
        //publish robot name
        data.name = "robot"+std::to_string(q); 
        std::cout << data.name << " - robot name" << endl;
        
        //publish no of tasks
        data.tasks = plotString[q].size() - 1; 
        std::cout << data.tasks << " - no of tasks" << endl;

        for (int j=1; j< plotString[q].size();j++)       //for each robot -> the list of tasks
             {   
                 string s(1,plotString[q][j]);           //convert char to string
                 stringD1.ltask = s;  
                 /*publish string of task*/
                 data.lT.push_back(stringD1);  
                 int x = (int)plotString[q][j];          //int of all tasks
               
                if (j != plotString[q].size() -1)
                {    
                    int y = x - 96;                      //int of all tasks
                    intD2.liTask = y;
                    /*publish int of tasks*/
                    data.lIT.push_back(intD2);  
                    
                    std::string t = "task"+std::to_string(y);
                    stringD3.namTask = t;     
                    /*publish string+int tasks*/
                    data.nT.push_back(stringD3);
                    
                    for(int i = nVehicles; i<totalCoord.size(); i++)  //pose of all tasks 
                    {    
                        if (t.compare(totalCoord[i].first) != 0)
                            {
                                //nothing
                            }
                        else
                            {
                                poseMsg.position.x = std::get<1>(totalCoord[i].second);
                                poseMsg.position.y = -1 * (std::get<0>(totalCoord[i].second));
                                
                                double radians = (std::get<2>(totalCoord[i].second)) * (M_PI/180);
                                tf::Quaternion quaternion;
                                quaternion = tf::createQuaternionFromYaw(radians);
                                geometry_msgs::Quaternion qMsg;
                                tf::quaternionTFToMsg(quaternion, qMsg);
                                poseMsg.orientation = qMsg;
                            }
                      }
                }
                else
                {
                    int z = x - 64 + nTasks; //int of all tasks                    
                    std::string dt = "dropoff"+std::to_string(z);
                    stringD3.namTask = dt;
                    data.nT.push_back(stringD3);
                    
                    //pose of the dropoff
                    for (int i = nTasks; i<totalCoord.size(); i++)
                    {
                        if (dt.compare(totalCoord[i].first) == 0)
                            {
                                poseMsg.position.x = std::get<1>(totalCoord[i].second);
                                poseMsg.position.y = -1 * (std::get<0>(totalCoord[i].second));
                                
                                double radians = (std::get<2>(totalCoord[i].second)) * (M_PI/180);
                                tf::Quaternion quaternion;
                                quaternion = tf::createQuaternionFromYaw(radians);
                                geometry_msgs::Quaternion qMsg;
                                tf::quaternionTFToMsg(quaternion, qMsg);
                                poseMsg.orientation = qMsg;
                            }
                    }
                }
             poseArrayMsg.header.stamp = ros::Time::now(); // timestamp of creation of the msg
             poseArrayMsg.header.frame_id = "map" ;
             poseArrayMsg.poses.push_back(poseMsg);
            }
     data.pList= poseArrayMsg;
     goalPub.publish(data);
     msg.list.push_back(data);
     q++;       
     data.lT.clear();  
     data.lIT.clear();
     data.nT.clear();
     poseArrayMsg.poses.clear();
    }
    taskPub.publish(msg);
}

    private:
        ros::NodeHandle n;
        ros::Subscriber coordSub;
        ros::Subscriber deltamatSub;
        ros::Subscriber probSub;
        ros::Publisher goalPub;
        ros::Publisher taskPub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TaskAllocationNode"); 
    SubPub sp;
    ros::spin();

    return 0;
}

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
