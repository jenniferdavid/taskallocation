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
#include "ros/ros.h"
#include "navi_msgs/Item.h"
#include "navi_msgs/ItemStruct.h"
#include "navi_msgs/Problem.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU> 
#include "std_msgs/Float64MultiArray.h"
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

using namespace Eigen;
using namespace std;

//calculates eulcidean distance in an empty map
double euclideanDistance (std::pair<double,double> a, std::pair<double,double> b)
{
    double dist, u, v;
    u = b.first - a.first;
    v = b.second - a.second;
    dist = u*u + v*v; //xb-xa and yb-ya is 
    dist = sqrt(dist);
    return dist;
}

class SubPub
{
    public:
        Eigen::MatrixXd DeltaMatrix;
        std_msgs::Float64MultiArray deltaMat;
        int totalModels;

        //constructor function that publishes and subscribes
        SubPub()
        {
            pub_ = n_.advertise<std_msgs::Float64MultiArray>("/deltamat",100);
            coordSub_ = n_.subscribe("/coords", 100, &SubPub::coordCallback, this);
            probSub_ = n_.subscribe("/problem", 100, &SubPub::probCallback, this);
        }
        
        void probCallback(const navi_msgs::Problem::ConstPtr& msg)
        {
            totalModels = msg->nTotalmodels;
        }

        //callback for subscribing /coords and publishing /EDdeltaMat
        void coordCallback(const navi_msgs::ItemStruct::ConstPtr& msg)
        {
            std::vector<std::pair<double,double> > totalCoord;
            for (int i=0; i< totalModels; ++i) //msg->coords.size()
                {
                const navi_msgs::Item &data = msg->coords[i];
                ROS_INFO_STREAM("Model ID: " << data.id << " x: " << data.x << " y: " << data.y << " yaw: " << data.yaw);
                totalCoord.push_back(std::make_pair(data.x, data.y));
                }
            DeltaMatrix = MatrixXd::Ones(totalCoord.size(),totalCoord.size());
            for(int i = 0; i < totalCoord.size(); i++)
                {cout << totalCoord[i].first << ", " << totalCoord[i].second << endl;}
            for (int i = 0; i < totalCoord.size(); i++)
                for (int j = 0; j < totalCoord.size(); j++)
                    {DeltaMatrix(i,j) = euclideanDistance(totalCoord[i],totalCoord[j]);} //calculates Eulcidean distance in an empty map

            cout <<"\n"<< DeltaMatrix <<endl;
            tf::matrixEigenToMsg(DeltaMatrix,deltaMat); // convert EigenMatrix to Float64MultiArray ROS format to publish
            pub_.publish(deltaMat);
        }
        
private:
        ros::NodeHandle n_;
        ros::Publisher pub_;
        ros::Subscriber coordSub_;
        ros::Subscriber probSub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DistCostNode"); 
    SubPub sp;

    ros::spin();
    return 0;
}
