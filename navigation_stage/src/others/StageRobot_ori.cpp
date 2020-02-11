#include "ros/ros.h"
#include "libplayerc++/playerc++.h"
#include <iostream>
#include <fstream>
#include "navi_msgs/Item.h"
#include "navi_msgs/ItemStruct.h"
#include "navi_msgs/Problem.h"

using namespace std;
using namespace PlayerCc;

struct Item
    {
        char name[16];
        int id;
        char A;
        double x;
        double y;
        double yaw;
    }typedef item_t;
    
int numRobots;
int numTasks;
navi_msgs::Problem details;
char taskL = 'a';
char dropL = 'A';
char robotL = 'A';
////////////////////////////////////////////////////////////////////////////////////////////////

//lists out coordinates of all the models from player using SimulationProxy
Item totalModelList(item_t *modelList, SimulationProxy &simProxy, int numRobots, int numTasks)
{
    
int numDropoffs = numRobots;
int numModels = numTasks + numDropoffs; 
int totalModels = numRobots + numModels;  

    //get the poses of the task
    for(int i=0;i<numTasks;i++)
        {
            char taskStr[] = "task%d";
            sprintf(modelList[i].name, taskStr, i+1);
            modelList[i].id = i+101;
            modelList[i].A = taskL; 
            simProxy.GetPose2d(modelList[i].name, modelList[i].x, modelList[i].y, modelList[i].yaw);
            ++taskL;
        }
        
    //get the poses of the dropoff
    for(int j =numTasks;j<numModels;j++)
        {
            char dropoff[] = "dropoff%d";
            sprintf(modelList[j].name, dropoff, j+1);
            modelList[j].id = j+1001;
            modelList[j].A = dropL; 
            simProxy.GetPose2d(modelList[j].name, modelList[j].x, modelList[j].y, modelList[j].yaw);
            ++dropL;
        }
    return modelList[numModels];
}

/////////////////////////////////////////////////////////////////////////////////////////////////

class Robot
{
public:
    Item allModels(ros::NodeHandle n, int numRobots, int numTasks);
          
    navi_msgs::Item data;
    navi_msgs::ItemStruct msg;
        
    ros::Publisher coords_pub;
    ros::Publisher problem_pub;

protected:
    PlayerClient *robot;
    Position2dProxy *p2dProxy;
    SimulationProxy *simProxy;
};

Item Robot::allModels(ros::NodeHandle n, int numRobots, int numTasks)
{
        
        int numDropoffs = numRobots;
        int numModels = numTasks + numDropoffs; 
        int totalModels = numRobots + numModels;     
        item_t robotList[numRobots];
        robot = new PlayerClient("localhost", 6665);
        Position2dProxy **position = new Position2dProxy*[numRobots];
        
            
        for (int k = 0; k<numRobots; k++)
            {
                position[k] = new Position2dProxy(robot, k);
                robot->Read();

                //receive all robots pose
                char robotStr[] = "robot%d";
                sprintf(robotList[k].name, robotStr, k);
                robotList[k].id = k;
                robotList[k].A = robotL; 
                robotList[k].x = position[k]->GetXPos(); 
                robotList[k].y = position[k]->GetYPos(); 
                robotList[k].yaw = position[k]->GetYaw(); 
                ++robotL;
            }
    
        //receive all models coords in itemList
        SimulationProxy simProxy(robot,0);
        item_t dupList[numModels];
        totalModelList(dupList, simProxy, numRobots, numTasks); 
    
        //create a finalList of all coordinates 
        item_t finalList[totalModels];
        std::copy(dupList,dupList+numModels, std::copy(robotList,robotList+numRobots,finalList));
    
        //totalList of robots and models stacked - printed out
        for (int k = 0; k < totalModels; k++) {
        std::cout << finalList[k].name << ' ' << finalList[k].id << ' ' << finalList[k].A << ' ' <<finalList[k].x << ' ' << finalList[k].y << ' '<< finalList[k].yaw << endl;
	}
        coords_pub = n.advertise<navi_msgs::ItemStruct>("/coords", 100);
        problem_pub = n.advertise<navi_msgs::Problem>("/problem", 100);

        while (ros::ok())
            {
            for (int k = 0; k< totalModels; k++)
                {
                    {string str(finalList[k].name);
                    data.name = str;}
                    data.id = finalList[k].id;
                    data.x = finalList[k].x;
                    data.y = finalList[k].y;
                    data.yaw = finalList[k].yaw;

                    msg.coords.push_back(data);
                    coords_pub.publish(msg);
                }
            details.nRobots = numRobots;
            details.nTasks = numTasks;
            details.nModels = numModels;
            details.nDropoffs = numDropoffs;
            details.nTotalmodels = totalModels;
            problem_pub.publish(details);
            }
            delete []position;
            return finalList[totalModels];
    }

/////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "stage_robot_node");
    ros::NodeHandle n;
    n.getParam("/stage_robot_node/numRobots", numRobots);
    n.getParam("/stage_robot_node/numTasks", numTasks);

    Robot robot;
    robot.allModels(n,numRobots,numTasks);
    //ros::spin();
    return 0;
}


























