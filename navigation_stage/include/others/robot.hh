#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED
#include <iostream>
#include <fstream>
#include <sstream>
#include <climits>
#include <iomanip>
#include <time.h>
#include <string>
#include "libplayerc++/playerc++.h"
#include "RefreshItemList.hh"

using namespace PlayerCc;
using namespace std;

class Robot
{

public:
        Robot(int);
        ~Robot(void){};
        void read();
                    
private:
        int portNumber;
        
protected:
        PlayerClient *robot;
        PlayerClient *planner;
        Position2dProxy *p2dProxy;
        RangerProxy *rangerProxy;
        SimulationProxy *simProxy;
};

//end class
#endif // ROBOT_H_INCLUDED
