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


class TaskAllocationSub
{
    public:
    std::vector<std::pair<std::string,std::tuple<double,double,double>> > totalCoord;
int nVehicles;      
int nTasks;       
int nDim; 
int rDim; 
Eigen::MatrixXd DeltaMatrix;

    //constructor function that publishes and subscribes
    TaskAllocationSub()
    {
        probSub = n.subscribe("/problem", 100, &TaskAllocationSub::probCallback,this);
        deltamatSub = n.subscribe("/deltamat", 100, &TaskAllocationSub::deltaMatCallback,this);
        coordSub = n.subscribe("/coords", 100, &TaskAllocationSub::coordsCallback,this);
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
    
    private:
        ros::NodeHandle n;
        ros::Subscriber coordSub;
        ros::Subscriber deltamatSub;
        ros::Subscriber probSub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TaskAllocationSubNode"); 
    TaskAllocationSub sub;
    ros::spin();

    return 0;
}

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
