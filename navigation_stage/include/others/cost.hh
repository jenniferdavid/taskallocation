#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include "libplayerc++/playerc++.h"
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

using namespace PlayerCc;

class CostMatrix    

{
    
 public:
        CostMatrix(); //constructor
        void constructMatrix(); //construct the cost matrix
        void calculatePath();  //calculate path distance between two points

 private:                
        ros::NodeHandle node;
        ros::Publisher coordsPub;  //Publish the coords
        ros::Publisher costMatrixPub;  //Publish the cost matrix
};

