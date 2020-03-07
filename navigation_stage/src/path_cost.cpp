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
   
    double goalTolerance = 0.1;
    nav_msgs::GetPlan srv;

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
        string service_name = "robot_0/move_base_node/make_plan";
            while (!ros::service::waitForService(service_name, ros::Duration(3.0))) 
            {ROS_INFO("Waiting for service move_base/make_plan to become available");}

            serviceClient = node.serviceClient<nav_msgs::GetPlan>(service_name, true);
            if (!serviceClient) 
            {   ROS_FATAL("Could not initialize get plan service from %s", serviceClient.getService().c_str());}
		
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
        for (int i = 0; i < robotCoord.size(); i++)
            for (int j = 0; j < robotCoord.size(); j++)
                {DeltaMatrix(i,j) = pathDistance(robotCoord[i],robotCoord[j]);} //calculates Eulcidean distance in an empty map

        cout <<"\n"<< DeltaMatrix <<endl;
        tf::matrixEigenToMsg(DeltaMatrix,deltaMat); // convert EigenMatrix to Float64MultiArray ROS format to publish
        deltapub_.publish(deltaMat);
    }  
    
    void fillPathRequest(nav_msgs::GetPlan::Request &request, std::pair<double,double> a, std::pair<double,double> b)
    {
        srv.request.start.header.frame_id = "map";
        srv.request.start.pose.position.x = a.second;
        srv.request.start.pose.position.y = -1*(a.first);
        srv.request.start.pose.orientation.w = 1.0;

        srv.request.goal.header.frame_id = "map";
        srv.request.goal.pose.position.x = b.second;
        srv.request.goal.pose.position.y = -1*(b.first);
        srv.request.goal.pose.orientation.w = 1.0;

        srv.request.tolerance = goalTolerance;
    }

    double callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv)
    {
        if (serviceClient.call(srv)) 
            {if (!srv.response.plan.poses.empty()) 
                    {   /* forEach(const geometry_msgs::PoseStamped &p, srv.response.plan.poses) 
                        {ROS_INFO("x = %f, y = %f", p.pose.position.x, p.pose.position.y);}*/
                        //calculate path length between two pose responses and add them to get total length
                        double dist = 0.0;
                        for (int i = 0; i<srv.response.plan.poses.size()-1 ;i++){
                             double old_dist = 0.0;
                             double u = 0.0;
                             double v = 0.0;
                             u = srv.response.plan.poses[i+1].pose.position.x - srv.response.plan.poses[i].pose.position.x;
                             v = srv.response.plan.poses[i+1].pose.position.y - srv.response.plan.poses[i].pose.position.y;
                             old_dist = u*u + v*v; 
                             dist = dist + sqrt(old_dist);
                        }
                        // std::cout << dist << endl;
                        return dist;
                    }
                else 
                    {ROS_WARN("Got empty plan");}}
        else 
            {ROS_ERROR("Failed to call service %s - is the robot moving?", serviceClient.getService().c_str());}
    }
        
    //calculates path length in the given map
    double pathDistance (std::pair<double,double> a, std::pair<double,double> b) //a is start and b is goal
    {  
            //std::cout << "\n The distance between " << a.first <<","<<a.second<< " and " << b.first << "," << b.second << " is " << endl;
            fillPathRequest(srv.request,a,b);
            if (!serviceClient) 
            {ROS_FATAL("Persistent service connection to %s failed", serviceClient.getService().c_str());return -1;}
            return callPlanningService (serviceClient, srv);
    }  

private:

    ros::NodeHandle node;
    ros::Publisher deltapub_;
    ros::ServiceClient serviceClient;

    message_filters::Subscriber<myMsg> robotcoordpub_;
    message_filters::Subscriber<myMsg> modelcoordpub_;
    
    typedef sync_policies::ApproximateTime<myMsg,myMsg> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathcost");
    Node synchronizer;
    ros::spin();
    return 0;
}


 
         













































