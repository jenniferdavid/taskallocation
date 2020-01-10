#include "ros/ros.h"
#include "libplayerc++/playerc++.h"
#include <iostream>
#include <fstream>
#include "navi_msgs/Item.h"
#include "navi_msgs/ItemStruct.h"

using namespace std;
using namespace PlayerCc;

struct Item
    {
        char name[16];
        int id;
        double x;
        double y;
        double yaw;
    }typedef item_t;
    
static size_t const numRobots(7);
static size_t const numTasks(18);
static size_t const numDropoffs(5);
static size_t const numModels(numTasks + numDropoffs); //23
static size_t const totalModels(numRobots + numModels);  //30      

////////////////////////////////////////////////////////////////////////////////////////////////

//lists out coordinates of all the models from player using SimulationProxy
Item totalModelList(item_t *modelList, SimulationProxy &simProxy) 
{
    int i;
    //get the poses of the task
    for(i=0;i<numTasks;i++)
        {
            char taskStr[] = "task%d";
            sprintf(modelList[i].name, taskStr, i+1);
            modelList[i].id = i+100;
            simProxy.GetPose2d(modelList[i].name, modelList[i].x, modelList[i].y, modelList[i].yaw);
        }
        
    //get the poses of the dropoff
    for(i=numTasks;i<numModels;i++)
        {
            char dropoff[] = "dropoff%d";
            sprintf(modelList[i].name, dropoff, i+1);
            modelList[i].id = i+1000;
            simProxy.GetPose2d(modelList[i].name, modelList[i].x, modelList[i].y, modelList[i].yaw);
        }
        
    return modelList[numModels];
}

/////////////////////////////////////////////////////////////////////////////////////////////////

class Robot
{
public:
    Item allModels(int i);
          
    navi_msgs::Item data;
    navi_msgs::ItemStruct msg;
        
    ros::NodeHandle n;
    ros::Publisher coords_pub;
    
protected:
    PlayerClient *robot;
    Position2dProxy *p2dProxy;
    SimulationProxy *simProxy;
};

Item Robot::allModels(int i)
    {
        item_t robotList[i];
        robot = new PlayerClient("localhost", 6665);
        Position2dProxy **position = new Position2dProxy*[i];
        
        for (int k = 0; k<i; k++)
            {
                position[k] = new Position2dProxy(robot, k);
                robot->Read();

                //receive all robots pose
                char robotStr[] = "robot%d";
                sprintf(robotList[k].name, robotStr, k);
                robotList[k].id = k;
                robotList[k].x = position[k]->GetXPos(); 
                robotList[k].y = position[k]->GetYPos(); 
                robotList[k].yaw = position[k]->GetYaw(); 
            }
    
        //receive all models coords in itemList
        SimulationProxy simProxy(robot,0);
        item_t dupList[numModels];
        totalModelList(dupList, simProxy); 
    
        //create a finalList of all coordinates 
        item_t finalList[totalModels];
        std::copy(dupList,dupList+numModels, std::copy(robotList,robotList+numRobots,finalList));
    
        //totalList of robots and models stacked - printed out
        for (int k = 0; k < totalModels; k++) {
        std::cout << finalList[k].name << ' ' << finalList[k].id << ' ' << finalList[k].x << ' ' << finalList[k].y << ' '<< finalList[k].yaw << endl;
	}
        coords_pub = n.advertise<navi_msgs::ItemStruct>("/coords", 1);
    
     while (ros::ok())
            {
            for (int k = 0; k< totalModels; k++)
                {
                    data.id = finalList[k].id;
                    data.x = finalList[k].x;
                    data.y = finalList[k].y;
                    data.yaw = finalList[k].yaw;

                    msg.coords.push_back(data);
                    coords_pub.publish(msg);
                }
            }
            delete []position;
            return finalList[totalModels];
    }

/////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "stage_robot");

    Robot robot;
    robot.allModels(numRobots);

    return 0;
}


























