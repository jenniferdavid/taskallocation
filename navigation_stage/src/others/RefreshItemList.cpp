#include <stdio.h>
#include <unistd.h>
#include <time.h>
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
#include "RefreshItemList.hh"
#include "ros/ros.h"
#include <navi_msgs/Item.h>
#include <navi_msgs/ItemStruct.h>

using namespace std;

//constructor
RefreshItemList::RefreshItemList(){}; 

void RefreshItemList::returnList(item_t *finalList)
    {
     for (int k = 0; k < 30; k++) {
        std::cout << finalList[k].name << ' ' << finalList[k].id << ' ' << finalList[k].x << ' ' << finalList[k].y << ' '<< finalList[k].yaw << endl;
	}
    }
    
//function to receive all the initial coords of the models from player using SimulationProxy
Item RefreshItemList::totalList(item_t *itemList, SimulationProxy &simProxy)
    {
        int i;
        //get the poses of the task
        for(i=0;i<18;i++)
        {
            char taskStr[] = "task%d";
            sprintf(itemList[i].name, taskStr, i+1);
            itemList[i].id = i+100;
            simProxy.GetPose2d(itemList[i].name, itemList[i].x, itemList[i].y, itemList[i].yaw);
            //cout << "Name is " <<itemList[i].name << ": x is "<< itemList[i].x << " and y is " << itemList[i].y << endl;
        }
        //get the poses of the dropoff
        for(i=18;i<23;i++)
        {
            char dropoff[] = "dropoff%d";
            sprintf(itemList[i].name, dropoff, i+1);
            itemList[i].id = i+1000;
            simProxy.GetPose2d(itemList[i].name, itemList[i].x, itemList[i].y, itemList[i].yaw);
            //cout << "Name is " <<itemList[i].name << ": x is "<< itemList[i].x << " and y is " << itemList[i].y << endl;
        }
        
        return itemList[23];
    }








    
