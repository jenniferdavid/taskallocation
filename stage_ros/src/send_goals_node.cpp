/*
 * SendGoals.cpp
 *
 *  Created on: Dec 7, 2013
 *      Author: roiyeho
 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include "navi_msgs/GoalsList.h"
#include "navi_msgs/Problem.h"
#include <thread>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <string>

using namespace std;

void taskNavigation(int a, navi_msgs::GoalsList task_msg)
{
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    std::string name1 = "robot_", name2 = "/move_base";
    std::string robotName= name1+std::to_string(a)+name2;
    std::cout <<robotName <<std::endl;
    MoveBaseClient ac(robotName, true);

    // Wait 60 seconds for the action server to become available
    ROS_INFO("Waiting for the move_base action server");
    ac.waitForServer(ros::Duration(60));
    ROS_INFO("Connected to move base server");
    
    move_base_msgs::MoveBaseGoal goal;
    int numTasks = task_msg.list[a].tasks;

    for (int i=0; i<numTasks; i++)
        {   
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();	
            goal.target_pose.pose.position.x = task_msg.list[a].pList.poses[i].position.x;
            goal.target_pose.pose.position.y = task_msg.list[a].pList.poses[i].position.y;

            double radians = task_msg.list[a].pList.poses[i].position.z * (M_PI/180);
            tf::Quaternion quaternion;
            quaternion = tf::createQuaternionFromYaw(radians);
            geometry_msgs::Quaternion qMsg;
            tf::quaternionTFToMsg(quaternion, qMsg);
            goal.target_pose.pose.orientation = qMsg;

            ROS_INFO("Sending robot_%d to: x = %f, y = %f, theta = %f", a,task_msg.list[a].pList.poses[i].position.x,task_msg.list[a].pList.poses[i].position.y,task_msg.list[a].pList.poses[i].position.z );    
            ac.sendGoal(goal);
            ac.waitForResult();
    
            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {   
                    ROS_INFO("The robot_%d REACHED the %d goal!",a,i+1);
                    if (i == (numTasks-1))
                    {   
                        ROS_INFO("FINISHED TASKS FOR ROBOT %d",a);
                        break;                            
                    }
                }    
            else
                ROS_INFO("The base failed for some reason");
        } 
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "SendGoalsNode");
    ros::NodeHandle nh_;
    ros::Duration(5).sleep();
    
    //Wait till the task list available
    navi_msgs::GoalsList task_msg;
    boost::shared_ptr<navi_msgs::GoalsList const> task_msgptr;
    task_msgptr = ros::topic::waitForMessage<navi_msgs::GoalsList>("/tasks",ros::Duration(10));
    if (task_msgptr == NULL)
        {ROS_INFO("No task messages received");}
    else
        {
    task_msg = * task_msgptr;
    //create multithreads to run all robots concurrently
    ROS_INFO("----------STARTING THREADS-----------");
    std::vector<std::thread> threadsTask;
    int nVehicles = task_msg.input.nRobots;
    cout << nVehicles <<endl;
    for(int a = 0; a < nVehicles; a++) 
    {threadsTask.push_back(std::thread(taskNavigation, a, task_msg));}
        
    for (auto& t : threadsTask)
    t.join();
    ROS_INFO("----------TASK ALLOCATION COMPLETED-----------");
        }
    return 0;
}
    
































