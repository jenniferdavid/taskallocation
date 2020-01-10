#include <iostream>
#include <fstream>
#include "/home/jendav/player-3.0.2/client_libs/libplayerc++/playerc++.h"
#include "/home/jendav/catkin_ws/src/navigation_stage/include/robot.hh"
#include "/home/jendav/catkin_ws/src/navigation_stage/include/RefreshItemList.hh"

using namespace std;
using namespace PlayerCc;

int main(int argc, char *argv[])
{

    int num = 7;
    Robot robot (num);
    robot.task(num);
    return 0;
}
