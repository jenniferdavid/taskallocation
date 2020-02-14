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

class SubPub
{
    public:
        
    Eigen::MatrixXd DeltaMatrix;
    std_msgs::Float64MultiArray deltaMat;
    int totalModels;    
    double goalTolerance = 0.1;
    nav_msgs::GetPlan srv;

    //constructor function that publishes and subscribes
    SubPub()
        {
             // Init service query for make plan
            string service_name = "robot_0/move_base_node/make_plan";
            while (!ros::service::waitForService(service_name, ros::Duration(3.0))) 
            {ROS_INFO("Waiting for service move_base/make_plan to become available");}

            serviceClient = nh_.serviceClient<nav_msgs::GetPlan>(service_name, true);
            if (!serviceClient) 
            {   ROS_FATAL("Could not initialize get plan service from %s", serviceClient.getService().c_str());}
		
            pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/deltamat",100);
            sub_ = nh_.subscribe("/coords", 100, &SubPub::callback, this);
            probSub_ = nh_.subscribe("/problem", 100, &SubPub::probCallback, this);
        }
        
    void probCallback(const navi_msgs::Problem::ConstPtr& msg)
        {
            totalModels = msg->nTotalmodels;
        }
        
    //callback for subscribing /coords and publishing /EDdeltaMat
    void callback(const navi_msgs::ItemStruct::ConstPtr& msg)
        {
            std::vector<std::pair<double,double> > totalCoord;
            for (int i=0; i< totalModels; ++i) //msg->coords.size()
                {
                const navi_msgs::Item &data = msg->coords[i];
                ROS_INFO_STREAM("Model ID: " << data.id << " x: " << data.x << " y: " << data.y << " yaw: " << data.yaw);
                totalCoord.push_back(std::make_pair(data.x, data.y));
                }
                        
            DeltaMatrix = MatrixXd::Ones(totalCoord.size(),totalCoord.size());
            
            /*for(int i = 0; i < totalCoord.size(); i++)
                {cout << totalCoord[i].first << ", " << totalCoord[i].second << endl;}*/
                cout << "\nComputing CostMatrix...."<< endl;

                //int k = 1;
            for (int i = 0; i < totalCoord.size(); i++)
                { for (int j = 0; j < totalCoord.size(); j++)
                        {  if (i==j)
                            {DeltaMatrix(i,j) = 0;}
                            else
                            {DeltaMatrix(i,j) =
                            pathDistance(totalCoord[i],totalCoord[j]);}//calculates path distance in the given map
                        } 
                }
            cout << "\nPublishing CostMatrix...."<< endl;
            cout <<"\n"<< DeltaMatrix <<endl;
            tf::matrixEigenToMsg(DeltaMatrix,deltaMat); // convert EigenMatrix to Float64MultiArray ROS format to publish
            pub_.publish(deltaMat);
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
        {if (serviceClient.call(srv)) 
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
 
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Subscriber probSub_;
    ros::ServiceClient serviceClient;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PathCostNode"); 
    SubPub sp;
    ros::spin();
    return 0;
}










































