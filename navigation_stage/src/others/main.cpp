#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include "/home/jendav/player-3.0.2/client_libs/libplayerc++/playerc++.h"
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
#include <math.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <locale.h>
#include <wchar.h>
#include <unistd.h>
#include "/home/jendav/catkin_ws/src/navigation_tutorials/navigation_stage/include/RefreshItemList.hh"
/*#include "/home/jendav/catkin_ws/src/multiple_robots_stage/include/CostMatrix.hh"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
*/
int main(int argc, char *argv[])
{
    using namespace PlayerCc;
    using namespace std;
    
    PlayerClient robot("localhost", 6665);
    RangerProxy sprox1(&robot,1);
    RangerProxy sprox2(&robot,2);
    RangerProxy sprox3(&robot,3);
    RangerProxy sprox4(&robot,4);
    RangerProxy sprox5(&robot,5);

    //robots
    Position2dProxy robot1(&robot,1);
    Position2dProxy robot2(&robot,2);
    Position2dProxy robot3(&robot,3);
    Position2dProxy robot4(&robot,4);
    Position2dProxy robot5(&robot,5);
           
    robot.Read();
    
    //receive all robots pose
    item_t robotList[6];
    for(int i=0;i<6;i++){
            char robotStr[] = "robot%d";
            sprintf(robotList[i+1].name, robotStr, i+1);
    }
    //for now
    robotList[1].x = robot1.GetXPos(); 
    robotList[1].y = robot1.GetYPos(); 
    robotList[1].yaw = robot1.GetYaw(); 
    
    robotList[2].x = robot2.GetXPos(); 
    robotList[2].y = robot2.GetYPos(); 
    robotList[2].yaw = robot2.GetYaw(); 
    
    robotList[3].x = robot3.GetXPos(); 
    robotList[3].y = robot3.GetYPos(); 
    robotList[3].yaw = robot3.GetYaw(); 
    
    robotList[4].x = robot4.GetXPos(); 
    robotList[4].y = robot4.GetYPos(); 
    robotList[4].yaw = robot4.GetYaw(); 
    
    robotList[5].x = robot5.GetXPos(); 
    robotList[5].y = robot5.GetYPos(); 
    robotList[5].yaw = robot5.GetYaw(); 
    
    //to be done: using vector of classes for multiple robots
        
    //shared Simultion proxy...
    SimulationProxy simProxy(&robot,0);
    //list to store all models coords    
    item_t itemList[23];
    
    RefreshItemList A; //class object
    //receive all models coords in itemList
    A.totalList(itemList, simProxy); 
    
    //create a matrix 
    int m = sizeof(robotList)/sizeof(robotList[0]);
    int n = sizeof(itemList)/sizeof(itemList[0]);
    item_t finalList[m+n];
    std::copy(itemList,itemList+n, std::copy(robotList,robotList+m,finalList));
    
    //totalList of robots and models stacked
    for (int i = 1; i < m + n; i++) {
        std::cout << finalList[i].name << ' ' << finalList[i].x << ' ' << finalList[i].y << ' '<< finalList[i].yaw << endl;
	}
    
    //run A* for any two points to get path length
    //read the map which will be 0 or 100 or -1
    //localize the robots in the map
    
    
    
                        //create a CostMatrix
                        //publish CostMatrix
    srand(time(NULL));
    
}





