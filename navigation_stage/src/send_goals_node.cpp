/* Send Goals node
 *
 * Copyright (C) 2014 Jennifer David. All rights reserved.
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * This C++ program is to send paths to each of the robots after getting
 * the task allocation details with ROS.
 * INPUT: ROS topic with task allocation solution
 * OUTPUT: ROS topic with send navigation goals to each robot
 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include "navi_msgs/GoalsList.h"
#include "navi_msgs/Problem.h"
#include "navi_msgs/OdomArray.h"

#include <thread>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <string>
#include <stage.hh>
#include <algorithm>

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
    
    /////////////////////////////////////////////////
    std::vector<std::pair<double,double> >taskposes;
    std::vector<Stg::Model *> modelmodels;
    struct StageModel
    {Stg::Model* modelmodel;};
    std::vector<StageModel const *> modelmodels_;
    Stg::Pose resetpose;
    resetpose.x = 0.0;
    resetpose.y = 0.0;
    resetpose.z = 0.0;
    resetpose.a = 0.0;
    /////////////////////////////////////////////////

    navi_msgs::OdomArray modcoords_msg;
    boost::shared_ptr<navi_msgs::OdomArray const> modelcoords_msgptr;
    modelcoords_msgptr = ros::topic::waitForMessage<navi_msgs::OdomArray>("/modelcoords",ros::Duration(10));
    if (modelcoords_msgptr == NULL)
        {ROS_INFO("No modelcoords messages received");}
    else
        {modcoords_msg = * modelcoords_msgptr;
 		for (int i = 0; i < modcoords_msg.coords.size(); i++)
                {taskposes.push_back(std::make_pair(modcoords_msg.coords[i].pose.pose.position.x,modcoords_msg.coords[i].pose.pose.position.y));}
	}

    for (int i=0; i<numTasks; i++)
        {   
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();	
            goal.target_pose.pose.position.x = task_msg.list[a].pList.coords[i].pose.pose.position.x;
            goal.target_pose.pose.position.y = task_msg.list[a].pList.coords[i].pose.pose.position.y;
            
            double radians = task_msg.list[a].pList.coords[i].pose.pose.position.z * (M_PI/180);
            tf::Quaternion quaternion;
            quaternion = tf::createQuaternionFromYaw(radians);
            geometry_msgs::Quaternion qMsg;
            tf::quaternionTFToMsg(quaternion, qMsg);
            goal.target_pose.pose.orientation = qMsg;
            
            ROS_INFO("Sending robot_%d to: x = %f, y = %f, theta = %f", a,task_msg.list[a].pList.coords[i].pose.pose.position.x,task_msg.list[a].pList.coords[i].pose.pose.position.y,task_msg.list[a].pList.coords[i].pose.pose.orientation.z );    
            ac.sendGoal(goal);
            ac.waitForResult();
    
            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {   
                    ROS_INFO("The robot_%d REACHED the %d goal!",a,i+1);
                    {   
                       // if the goal == one of the modelcoords, then remove the task from the simulator
                        auto p = std::make_pair(task_msg.list[a].pList.coords[i].pose.pose.position.x,task_msg.list[a].pList.coords[i].pose.pose.position.y);
                        if (std::find(taskposes.begin(), taskposes.end(), p) != taskposes.end())
                         { 
                             //set the poses of the corresponding model in minus
                         }
                    }
                    
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
    //ros::Duration(5).sleep();
    
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
    
