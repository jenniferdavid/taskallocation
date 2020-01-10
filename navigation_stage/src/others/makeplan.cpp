/*
 * MakePlan.cpp
 *
 *  Modified on: Oct 28, 2019
 *      Author: jendav
 */

#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>

#include "navi_msgs/Item.h"
#include "navi_msgs/ItemStruct.h"
#include <string>
using std::string;

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

double g_GoalTolerance = 0.5;
string g_WorldFrame = "map";
static size_t const totalModels(30);  //30    
            nav_msgs::GetPlan srv;

void fillPathRequest(nav_msgs::GetPlan::Request &req, std::pair<double,double> a, std::pair<double,double> b);
void callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv);

void callback(const navi_msgs::ItemStruct::ConstPtr& msg)
        {
            std::vector<std::pair<double,double> > totalCoord;
            for (int i=0; i< totalModels; ++i) //msg->coords.size()
                {
                const navi_msgs::Item &data = msg->coords[i];
                ROS_INFO_STREAM("Model ID: " << data.id << " x: " << data.x << " y: " << data.y << " yaw: " << data.yaw);
                totalCoord.push_back(std::make_pair(data.x, data.y));
                }
            fillPathRequest(srv.request,totalCoord[0],totalCoord[1]);
        }
        
int main(int argc, char** argv)
{
	ros::init(argc, argv, "make_plan_nodes");
	ros::NodeHandle nh;

	// Init service query for make plan
	string service_name = "robot_2/move_base_node/make_plan";
	while (!ros::service::waitForService(service_name, ros::Duration(3.0))) {
		ROS_INFO("Waiting for service move_base/make_plan to become available");
	}

	ros::ServiceClient serviceClient = nh.serviceClient<nav_msgs::GetPlan>(service_name, true);
	if (!serviceClient) {
		ROS_FATAL("Could not initialize get plan service from %s", serviceClient.getService().c_str());
		return -1;
	}
        
        ros::Subscriber sub_ = nh.subscribe("/coords", 1, callback);
//        nav_msgs::GetPlan srv;
//	fillPathRequest(srv.request);

	if (!serviceClient) {
		ROS_FATAL("Persistent service connection to %s failed", serviceClient.getService().c_str());
		return -1;
	}

	callPlanningService(serviceClient, srv);
}

void fillPathRequest(nav_msgs::GetPlan::Request &request, std::pair<double,double> a, std::pair<double,double> b)
{
	request.start.header.frame_id = g_WorldFrame;
	request.start.pose.position.x = 8;
	request.start.pose.position.y = 6;
	request.start.pose.orientation.w = 1.0;

	request.goal.header.frame_id = g_WorldFrame;
	request.goal.pose.position.x = 12;
	request.goal.pose.position.y = 15;
	request.goal.pose.orientation.w = 1.0;

	request.tolerance = g_GoalTolerance;
}

void callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv)
{
	 // Perform the actual path planner call
	if (serviceClient.call(srv)) {
		if (!srv.response.plan.poses.empty()) {
			forEach(const geometry_msgs::PoseStamped &p, srv.response.plan.poses) {
				ROS_INFO("x = %f, y = %f", p.pose.position.x, p.pose.position.y);
			}
		}
		else {
			ROS_WARN("Got empty plan");
		}
	}
	else {
		ROS_ERROR("Failed to call service %s - is the robot moving?", serviceClient.getService().c_str());
	}
}

