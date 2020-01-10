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
#include "/home/jendav/catkin_ws/src/multiple_robots_stage/include/CostMatrix.hh"
#include "ros/ros.h"

using namespace std;
    
CostMatrix::CostMatrix(){   };
    
void CostMatrix::publishCoords()    
        { 
             //Advertise a new publisher for the simulated robot's velocity command topic                
            coordsPub = node.advertise<geometry_msgs::Point>("",10);
            geometry_msgs::Point msg;    //The default constructor will set all commands to 0  
            msg.x = ;
            msg.y = ;
            coordsPub.publish(msg);
        }

void CostMatrix::constructMatrix()
    {                
        ros::Rate rate(10);                
        ROS_INFO("Starting..");
        while (ros::ok())    
        {                          
            publishCoords();                                
            ros::spinOnce();    //Need to call this function often to allow ROS to process incoming messages                                
            rate.sleep();                
        }
        
    }
    
