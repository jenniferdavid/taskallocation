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
#include "navi_msgs/OdomArray.h"
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
        int totalModels, totalRobots, Models;
        std::vector<std::pair<double,double> > robotCoord;
        std::vector<std::pair<double,double> > modelCoord;

        //constructor function that publishes and subscribes
        SubPub()
        {
            modelcoordSub_ = n_.subscribe("/modelcoords", 100, &SubPub::modelCallback, this);
            robotcoordSub_ = n_.subscribe("/robotcoords", 100, &SubPub::robotCallback, this);
            probSub_ = n_.subscribe("/problem", 100, &SubPub::probCallback, this);
            
            pub_ = n_.advertise<std_msgs::Float64MultiArray>("/deltamat",100);
            ros::Duration(1.0).sleep();
            robotCoord.insert( robotCoord.end(), modelCoord.begin(), modelCoord.end());            
            DeltaMatrix = MatrixXd::Ones(robotCoord.size(),robotCoord.size());
            for(int i = 0; i < robotCoord.size(); i++)
                {cout << robotCoord[i].first << ", " << robotCoord[i].second << endl;}
            for (int i = 0; i < robotCoord.size(); i++)
                for (int j = 0; j < robotCoord.size(); j++)
                    {DeltaMatrix(i,j) = euclideanDistance(robotCoord[i],robotCoord[j]);} //calculates Eulcidean distance in an empty map

            cout <<"\n"<< DeltaMatrix <<endl;
            tf::matrixEigenToMsg(DeltaMatrix,deltaMat); // convert EigenMatrix to Float64MultiArray ROS format to publish
            pub_.publish(deltaMat);
        }
        
        void probCallback(const navi_msgs::Problem::ConstPtr& msg)
        {
            totalModels = msg->nTotalmodels;
            totalRobots = msg->nRobots;
            Models = msg->nModels;
        }

        //callback for subscribing /robotcoords and publishing /EDdeltaMat
        void robotCallback(const navi_msgs::OdomArray::ConstPtr& msg)
        {
            for (int i=0; i< totalRobots; ++i) //msg->coords.size()
                {
                    double x,y;
                    x = msg->coords[i].pose.pose.position.x;
                    y = msg->coords[i].pose.pose.position.y;
                    robotCoord.push_back(std::make_pair(x,y));
                }
        }
        
        //callback for subscribing /modelcoords and publishing /EDdeltaMat
        void modelCallback(const navi_msgs::OdomArray::ConstPtr& msg2)
        {
            for (int i=0; i< Models; ++i) //msg->coords.size()
                {
                    double x2,y2;
                    x2 = msg2->coords[i].pose.pose.position.x;
                    y2 = msg2->coords[i].pose.pose.position.y;
                    modelCoord.push_back(std::make_pair(x2,y2));
                }
        }
        
private:
        ros::NodeHandle n_;
        ros::Publisher pub_;
        ros::Subscriber robotcoordSub_;
        ros::Subscriber modelcoordSub_;
        ros::Subscriber probSub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "EuclideanCostNode"); 
    SubPub sp;

    ros::spin();
    return 0;
}
