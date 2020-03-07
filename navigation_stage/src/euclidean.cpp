#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
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

using namespace Eigen;
using namespace std;
using namespace message_filters;
typedef navi_msgs::OdomArray myMsg;

double euclideanDistance (std::pair<double,double> a, std::pair<double,double> b)
{
    double dist, u, v;
    u = b.first - a.first;
    v = b.second - a.second;
    dist = u*u + v*v; //xb-xa and yb-ya is 
    dist = sqrt(dist);
    return dist;
}

class Node
{

public:
    Eigen::MatrixXd DeltaMatrix;
    std_msgs::Float64MultiArray deltaMat;
    int totalModels, Robots, Models;
    std::vector<std::pair<double,double> > robotCoord;
    std::vector<std::pair<double,double> > modelCoord;
    navi_msgs::Problem probmsg;
    boost::shared_ptr<navi_msgs::Problem const> probmsgptr;

    Node()
	{
        deltapub_ = node.advertise<std_msgs::Float64MultiArray>("/deltamat",10);
        robotcoordpub_.subscribe(node, "robotcoords", 1);
        modelcoordpub_.subscribe(node, "modelcoords", 1);
        sync.reset(new Sync(MySyncPolicy(10), robotcoordpub_, modelcoordpub_));   
        sync->registerCallback(boost::bind(&Node::callback, this, _1, _2));
    }
    
    void callback(const navi_msgs::OdomArrayConstPtr &in1, const navi_msgs::OdomArrayConstPtr &in2)
    {
        probmsgptr = ros::topic::waitForMessage<navi_msgs::Problem>("/problem",ros::Duration(10));
        if (probmsgptr == NULL)
            {ROS_INFO("No problem input messages received");}
        else
            {
                probmsg = * probmsgptr; 
                Robots = probmsg.nRobots;
                Models = probmsg.nModels;     
    
                for (int i=0; i< Robots; ++i) 
                {
                    double x1,y1;
                    x1 = in1->coords[i].pose.pose.position.x;
                    y1 = in1->coords[i].pose.pose.position.y;
                    robotCoord.push_back(std::make_pair(x1,y1));
                }
                
                for (int i=0; i< Models; ++i) 
                {
                    double x2,y2;
                    x2 = in2->coords[i].pose.pose.position.x;
                    y2 = in2->coords[i].pose.pose.position.y;
                    modelCoord.push_back(std::make_pair(x2,y2));
                }
            }      
        robotCoord.insert( robotCoord.end(), modelCoord.begin(), modelCoord.end());            
        DeltaMatrix = MatrixXd::Ones(robotCoord.size(),robotCoord.size());
        /*for(int i = 0; i < robotCoord.size(); i++)
                {cout << robotCoord[i].first << ", " << robotCoord[i].second << endl;}
        */for (int i = 0; i < robotCoord.size(); i++)
            for (int j = 0; j < robotCoord.size(); j++)
                {DeltaMatrix(i,j) = euclideanDistance(robotCoord[i],robotCoord[j]);} //calculates Eulcidean distance in an empty map

        cout <<"\n"<< DeltaMatrix <<endl;
        tf::matrixEigenToMsg(DeltaMatrix,deltaMat); // convert EigenMatrix to Float64MultiArray ROS format to publish
        deltapub_.publish(deltaMat);
    }  
    

private:

    ros::NodeHandle node;
    ros::Publisher deltapub_;

    message_filters::Subscriber<myMsg> robotcoordpub_;
    message_filters::Subscriber<myMsg> modelcoordpub_;
    
    typedef sync_policies::ApproximateTime<myMsg,myMsg> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "euclidean");
    Node synchronizer;
    ros::spin();
    return 0;
}
