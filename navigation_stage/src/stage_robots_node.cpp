/* stage_robots_node.cpp
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
 * This C++ program is part of the Task Allocation framework with Stage.
 * It connects with the player configuration file, reads the position 
 * data about robots, tasks and dropoffs and publishes them as ROS topics. 
 * INPUT: Player configuration
 * OUTPUT: Two ROS topics - 1. All coordinates 2. Problem details
 */
#include "ros/ros.h"
#include "libplayerc++/playerc++.h"
#include <iostream>
#include <fstream>
#include "navi_msgs/Item.h"
#include "navi_msgs/ItemStruct.h"
#include "navi_msgs/Problem.h"

using namespace std;
using namespace PlayerCc;

int main(int argc, char *argv[])
{
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
    
    ros::init(argc, argv, "StageRobotsNode");
    ros::NodeHandle n;
    n.getParam("/StageRobotsNode/numRobots", numRobots);
    n.getParam("/StageRobotsNode/numTasks", numTasks);
   
    int numDropoffs = numRobots;
    int numModels = numTasks + numDropoffs; 
    int totalModels = numRobots + numModels;     

    item_t robotList[numRobots];
    item_t modelList[numModels];
    item_t finalList[totalModels];
    
    char taskL = 'a';
    char dropL = 'A';
    char robotL = 'A';
    
    navi_msgs::Item data;
    navi_msgs::ItemStruct msg;
    navi_msgs::Problem details;

    PlayerClient *robot;
    robot = new PlayerClient("localhost", 6665);
    
    Position2dProxy **position = new Position2dProxy*[numRobots];
    SimulationProxy simProxy(robot,0);

    //receive all robots pose
    for (int k = 0; k<numRobots; k++)
        {
            position[k] = new Position2dProxy(robot, k);
            robot->Read();
            char robotStr[] = "robot%d";
            sprintf(robotList[k].name, robotStr, k);
            robotList[k].id = k;
            robotList[k].A = robotL; 
            robotList[k].x = position[k]->GetXPos(); 
            robotList[k].y = position[k]->GetYPos(); 
            robotList[k].yaw = position[k]->GetYaw(); 
            ++robotL;
        }
            
    //get the poses of the task
    for(int i=0;i<numTasks;i++)
        {
            char taskStr[] = "task%d";
            sprintf(modelList[i].name, taskStr, i);
            modelList[i].id = i+101;
            modelList[i].A = taskL; 
            simProxy.GetPose2d(modelList[i].name, modelList[i].x, modelList[i].y, modelList[i].yaw);
            ++taskL;
        }
        
    //get the poses of the dropoff
    for(int j =numTasks;j<numModels;j++)
        {
            char dropoff[] = "dropoff%d";
            sprintf(modelList[j].name, dropoff, j);
            modelList[j].id = j+1001;
            modelList[j].A = dropL; 
            simProxy.GetPose2d(modelList[j].name, modelList[j].x, modelList[j].y, modelList[j].yaw);
            ++dropL;
        }
    
    //create a finalList of all coordinates 
    std::copy(modelList,modelList+numModels, std::copy(robotList,robotList+numRobots,finalList));
    
    //totalList of robots and models stacked - printed out
    for (int k = 0; k < totalModels; k++) {
        std::cout << finalList[k].name << ' ' << finalList[k].id << ' ' << finalList[k].A << ' ' <<finalList[k].x << ' ' << finalList[k].y << ' '<< finalList[k].yaw << endl;
	}
    
    delete []position;

    ros::Publisher coords_pub = n.advertise<navi_msgs::ItemStruct>("/allcoords", 10);
    ros::Publisher problem_pub = n.advertise<navi_msgs::Problem>("/problem", 10);
    ros::Rate loop_rate(10);

    while (ros::ok() )
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
                }
            coords_pub.publish(msg);
            ROS_INFO("Publishing coords data");

            details.nRobots = numRobots;
            details.nTasks = numTasks;
            details.nModels = numModels;
            details.nDropoffs = numDropoffs;
            details.nTotalmodels = totalModels;
            problem_pub.publish(details);
            ROS_INFO("Publishing problem data");

            ros::spinOnce();
            loop_rate.sleep();
        }
    return 0;
}


























