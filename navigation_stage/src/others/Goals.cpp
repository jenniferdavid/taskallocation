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
#include <geometry_msgs/Pose2D.h>
#include <thread>
#include <string>

static size_t const totalModels(32);  //5+18+5   
double goalTolerance = 0.1;

//subscribe to Pose2D msgs
void callback(const geometry_msgs::Pose2D::ConstPtr& msg)
        {
                //callback
        }
        
void sendTask(std::vector<geometry_msgs::Pose2D> robotPoses, int a)
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
            int i = 0;

            while (!robotPoses[i].empty())
            {
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();	
                goal.target_pose.pose.position.x = robotPoses[i].x;
                goal.target_pose.pose.position.y = robotPoses[i].y;

                double radians = robotPoses[i].theta * (M_PI/180);
                tf::Quaternion quaternion;
                quaternion = tf::createQuaternionFromYaw(radians);
                geometry_msgs::Quaternion qMsg;
                tf::quaternionTFToMsg(quaternion, qMsg);
                goal.target_pose.pose.orientation = qMsg;

                ROS_INFO("Sending robot_%d to: x = %f, y = %f, theta = %f", a, robotPoses[i].x, robotPoses[i].x, robotPoses[i].theta);    
                ac.sendGoal(goal);
                ac.waitForResult();
    
                if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {ROS_INFO("The robot_%d reached the %d goal!",a,i);}    
                else if (i==robotPoses[i].size())
                    {ROS_INFO("The robot_%d finished its task",a);
                    return;}
                else
                    {ROS_INFO("The base failed for some reason");}
            }
        }

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "send_goals_node");
    ros::Subscriber sub_ = nh_.subscribe("/goals", 100, &Goals::callback, this);
    ros::spin();
    return 0;
}
    
    /*//poses for first robot
    std::vector<geometry_msgs::Pose2D> robotPose0;
    double xx0 [] = {54,46,47,54};
    double yy0 [] = {35,22,15,5};
    double tt0 [] = {0,0,0,0};
    for (int i = 0; i<4; i++)
    {robotPose0[i].x = xx0[i];
     robotPose0[i].y = yy0[i];
     robotPose0[i].theta = tt0[i];
    }
          
    //poses for second robot
    std::vector<geometry_msgs::Pose2D> robotPose1;
    double xx1 [] = {12,16,10,8};
    double yy1 [] = {51,36,30,38};
    double tt1 [] = {0,0,0,0};
    for (int i = 0; i<4; i++)
    {robotPose1[i].x = xx1[i];
     robotPose1[i].y = yy1[i];
     robotPose1[i].theta = tt1[i];
    }  
    
    std::vector<geometry_msgs::Pose2D> robotPoses;
    
    std::vector<std::thread> threads;
    for (int i = 0; i <2; ++i)
    {threads.emplace_back(sendTask,robotPoses,i); }

    for(auto& i: threads)
    i.join();
    
    //poses for third robot
    /*double xx2 [] = {30,38,40,47,55};
    double yy2 [] = {28,24,31,37,48};
    double tt2 [] = {0,0,0,0,0};
    std::vector <double> x2;
    std::vector <double> y2;
    std::vector <double> theta2;
    for (int i = 0; i<5; i++)
    {x2.push_back(xx2[i]);
    y2.push_back(yy2[i]);
    theta2.push_back(tt2[i]);}
     
    //poses for fourth robot
    double xx3 [] = {26,30,32};
    double yy3 [] = {39,50,40};
    double tt3 [] ={0,0,0};
    std::vector <double> x3;
    std::vector <double> y3;
    std::vector <double> theta3;
    for (int i = 0; i<3; i++)
    {x3.push_back(xx3[i]);
    y3.push_back(yy3[i]);
    theta3.push_back(tt3[i]);}
     
    //poses for fifth robot
    double xx4 [] = {38,45};
    double yy4 [] = {18,18};
    double tt4 [] = {0,0};
    std::vector <double> x4;
    std::vector <double> y4;
    std::vector <double> theta4;
    for (int i = 0; i<2; i++)
    {x4.push_back(xx4[i]);
    y4.push_back(yy4[i]);
    theta4.push_back(tt4[i]);}
     
    //poses for sixth robot
    double xx5 [] = {5,7,10};
    double yy5 [] = {24,5,10};
    double tt5 [] = {0,0,0};
    std::vector <double> x5;
    std::vector <double> y5;
    std::vector <double> theta5;
    for (int i = 0; i<3; i++)
    {x5.push_back(xx5[i]);
    y5.push_back(yy5[i]);
    theta5.push_back(tt5[i]);}
     
    //poses for seventh robot
    double xx6 [] = {23,29,24,22};
    double yy6 [] = {22,12,2,12};
    double tt6 [] = {0,0,0,0};
    std::vector <double> x6;
    std::vector <double> y6;
    std::vector <double> theta6;
    for (int i = 0; i<4; i++)
    {x6.push_back(xx6[i]);
     y6.push_back(yy6[i]);
     theta6.push_back(tt6[i]);}
    
    // create the action client	
    MoveBaseClient ac0("robot_0/move_base", true);
    MoveBaseClient ac1("robot_1/move_base", true);
    MoveBaseClient ac2("robot_2/move_base", true);
    MoveBaseClient ac3("robot_3/move_base", true);
    MoveBaseClient ac4("robot_4/move_base", true);
    MoveBaseClient ac5("robot_5/move_base", true);
    MoveBaseClient ac6("robot_6/move_base", true);

    // Wait 60 seconds for the action server to become available
    ROS_INFO("Waiting for the move_base action server");
    ac0.waitForServer(ros::Duration(60));
    ac1.waitForServer(ros::Duration(60));
    ac2.waitForServer(ros::Duration(60));
    ac3.waitForServer(ros::Duration(60));
    ac4.waitForServer(ros::Duration(60));
    ac5.waitForServer(ros::Duration(60));
    ac6.waitForServer(ros::Duration(60));

    ROS_INFO("Connected to move base server");

    // Send a goal to move_base
    move_base_msgs::MoveBaseGoal goal0;
    move_base_msgs::MoveBaseGoal goal1;
    move_base_msgs::MoveBaseGoal goal2;
    move_base_msgs::MoveBaseGoal goal3;
    move_base_msgs::MoveBaseGoal goal4;
    move_base_msgs::MoveBaseGoal goal5;
    move_base_msgs::MoveBaseGoal goal6;

    for (int i = 0; i<5; i++)
    {
        goal0.target_pose.header.frame_id = "map";
        goal0.target_pose.header.stamp = ros::Time::now();	
        goal0.target_pose.pose.position.x = x0[i];
        goal0.target_pose.pose.position.y = y0[i];
    
        goal1.target_pose.header.frame_id = "map";
        goal1.target_pose.header.stamp = ros::Time::now();	
        goal1.target_pose.pose.position.x = x1[i];
        goal1.target_pose.pose.position.y = y1[i];

        goal2.target_pose.header.frame_id = "map";
        goal2.target_pose.header.stamp = ros::Time::now();	
        goal2.target_pose.pose.position.x = x2[i];
        goal2.target_pose.pose.position.y = y2[i];
        
        goal3.target_pose.header.frame_id = "map";
        goal3.target_pose.header.stamp = ros::Time::now();	
        goal3.target_pose.pose.position.x = x3[i];
        goal3.target_pose.pose.position.y = y3[i];
        
        goal4.target_pose.header.frame_id = "map";
        goal4.target_pose.header.stamp = ros::Time::now();	
        goal4.target_pose.pose.position.x = x4[i];
        goal4.target_pose.pose.position.y = y4[i];
        
        goal5.target_pose.header.frame_id = "map";
        goal5.target_pose.header.stamp = ros::Time::now();	
        goal5.target_pose.pose.position.x = x5[i];
        goal5.target_pose.pose.position.y = y5[i];
        
        goal6.target_pose.header.frame_id = "map";
        goal6.target_pose.header.stamp = ros::Time::now();	
        goal6.target_pose.pose.position.x = x6[i];
        goal6.target_pose.pose.position.y = y6[i];
        
        // Convert the Euler angle to quaternion
        double radians0 = theta0[i] * (M_PI/180);
        tf::Quaternion quaternion0;
        quaternion0 = tf::createQuaternionFromYaw(radians0);
        geometry_msgs::Quaternion qMsg0;
        tf::quaternionTFToMsg(quaternion0, qMsg0);
        goal0.target_pose.pose.orientation = qMsg0;
    
        double radians1 = theta1[i] * (M_PI/180);
        tf::Quaternion quaternion1;
        quaternion1 = tf::createQuaternionFromYaw(radians1);
        geometry_msgs::Quaternion qMsg1;
        tf::quaternionTFToMsg(quaternion1, qMsg1);
        goal1.target_pose.pose.orientation = qMsg1;
        
        double radians2 = theta2[i] * (M_PI/180);
        tf::Quaternion quaternion2;
        quaternion2 = tf::createQuaternionFromYaw(radians2);
        geometry_msgs::Quaternion qMsg2;
        tf::quaternionTFToMsg(quaternion2, qMsg2);
        goal2.target_pose.pose.orientation = qMsg2;
        
        double radians3 = theta3[i] * (M_PI/180);
        tf::Quaternion quaternion3;
        quaternion3 = tf::createQuaternionFromYaw(radians3);
        geometry_msgs::Quaternion qMsg3;
        tf::quaternionTFToMsg(quaternion3, qMsg3);
        goal3.target_pose.pose.orientation = qMsg3;
        
        double radians4 = theta4[i] * (M_PI/180);
        tf::Quaternion quaternion4;
        quaternion4 = tf::createQuaternionFromYaw(radians4);
        geometry_msgs::Quaternion qMsg4;
        tf::quaternionTFToMsg(quaternion4, qMsg4);
        goal4.target_pose.pose.orientation = qMsg4;
        
        double radians5 = theta5[i] * (M_PI/180);
        tf::Quaternion quaternion5;
        quaternion5 = tf::createQuaternionFromYaw(radians5);
        geometry_msgs::Quaternion qMsg5;
        tf::quaternionTFToMsg(quaternion5, qMsg5);
        goal5.target_pose.pose.orientation = qMsg5;
        
        double radians6 = theta6[i] * (M_PI/180);
        tf::Quaternion quaternion6;
        quaternion6 = tf::createQuaternionFromYaw(radians6);
        geometry_msgs::Quaternion qMsg6;
        tf::quaternionTFToMsg(quaternion6, qMsg6);
        goal6.target_pose.pose.orientation = qMsg6;

        // Send the goal command
        ROS_INFO("Sending robot_0 to: x = %f, y = %f, theta = %f", x0[i], y0[i], theta0[i]);
        ac0.sendGoal(goal0);
    
        ROS_INFO("Sending robot_1 to: x = %f, y = %f, theta = %f", x1[i], y1[i], theta1[i]);
        ac1.sendGoal(goal1);
        
        ROS_INFO("Sending robot_2 to: x = %f, y = %f, theta = %f", x2[i], y2[i], theta2[i]);
        ac2.sendGoal(goal2);
        
        ROS_INFO("Sending robot_3 to: x = %f, y = %f, theta = %f", x3[i], y3[i], theta3[i]);
        ac3.sendGoal(goal3);
        
        ROS_INFO("Sending robot_4 to: x = %f, y = %f, theta = %f", x4[i], y4[i], theta4[i]);
        ac4.sendGoal(goal4);
        
        ROS_INFO("Sending robot_5 to: x = %f, y = %f, theta = %f", x5[i], y5[i], theta5[i]);
        ac5.sendGoal(goal5);
        
        ROS_INFO("Sending robot_6 to: x = %f, y = %f, theta = %f", x6[i], y6[i], theta6[i]);
        ac6.sendGoal(goal6);

        // Wait for the action to return
        ac0.waitForResult();
        if (ac0.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Robot reached the goal!");
        else
            ROS_INFO("The base failed for some reason");
        
        ac1.waitForResult();
        if (ac1.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Robot reached the goal!");
        else
            ROS_INFO("The base failed for some reason");
    
        ac2.waitForResult();
        if (ac2.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Robot reached the goal!");
        else
            ROS_INFO("The base failed for some reason");
        
        ac3.waitForResult();
        if (ac3.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Robot reached the goal!");
        else
            ROS_INFO("The base failed for some reason");
              
        ac4.waitForResult();
        if (ac4.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Robot reached the goal!");
        else
            ROS_INFO("The base failed for some reason");
              
        ac5.waitForResult();
        if (ac5.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Robot reached the goal!");
        else
            ROS_INFO("The base failed for some reason");
        
        ac6.waitForResult();
        if (ac6.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Robot reached the goal!");
        else
            ROS_INFO("The base failed for some reason");
}*/  
    

































