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

using namespace std;

struct Item
    {
        char name[16];
        int id;
        double x;
        double y;
        double yaw;
    }typedef item_t;
    
using namespace PlayerCc;
    
class RefreshItemList

{  
    item_t itemList[23];
    
public:
    
    RefreshItemList(); //constructor
    void returnList(item_t *itemList);
    Item totalList(item_t *itemList, SimulationProxy &simProxy); //lists out coordinates of all the models from player using SimulationProxy
    
};

