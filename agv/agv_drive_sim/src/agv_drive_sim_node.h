#ifndef agv_drive_sim_node_H
#define agv_drive_sim_node_H

//Eigen includes
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

//STD/STL cpp
#include <iostream>
#include <math.h>

//standard ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

//cargo ants ROS 
#include "cargo_ants_msgs/ReferenceTrajectory.h"
/** \brief Implementation of AGV inverse kinematics
 *
 * This package implements a ROS node which receives platform velocity commands (dot(x), dot(theta)) and translates them
 * to control inputs for low-level steering and wheel transmission PID's. 
 * Therefore it implements inverese kinematics of AGV, specifically the zero side-slip model. 
 * All along the class letters fl, fr, bl, br mean respectively front-left, front-right, back-left, back-right
 * 
 */
class AgvDriveNode
{
    protected: 
        //ros node handle
        ros::NodeHandle nh_;
               
        //trajectory subscriber
        ros::Subscriber trajectory_subscriber_;

        //trajectory subscriber
        ros::Subscriber teleop_subscriber_;
        
        //low level command publishers
        ros::Publisher fl_steer_publisher_;
        ros::Publisher fr_steer_publisher_;
        ros::Publisher bl_steer_publisher_;
        ros::Publisher br_steer_publisher_;        
        ros::Publisher fl_wheel_rate_publisher_;
        ros::Publisher fr_wheel_rate_publisher_;
        ros::Publisher bl_wheel_rate_publisher_;
        ros::Publisher br_wheel_rate_publisher_;
        
        //wished update rate of the main loop, [hz]        
        double rate_;
        
        //trajectory
        std::vector<Eigen::Vector3d> trajectory_; 
        
        //teleop command
        double teleop_v_;
        double teleop_w_;
        
        //time to execute each trajectory point
        double dt_; 
        
        //flag indicating new trajectory/command has arrived
        //bool new_commands_flag_; 
        
        //current point index. Current point is retrived as trajectory_.at(current_idx_)
        unsigned int current_idx_; 
        
        //time marking the end of the current command
        ros::Time end_command_time_;
        
        //wheel steering values
        double sfl_, sfr_, sbl_, sbr_; 

        //wheel rate values
        double wfl_, wfr_, wbl_, wbr_; 
        
        //vehicle kinematic parameters 
        double wheel_radius_, axis_width_, axes_separation_; 
                
    public:
        /** \brief Default constructor
        * 
        * This constructor initializes the node. 
        * 
        */
        AgvDriveNode();

        /** \brief Destructor
        * 
        * This destructor frees all necessary dynamic memory allocated within this class.
        */
        ~AgvDriveNode();
        
        /** \brief Returns rate_
         * 
         * Returns rate_
         * 
         **/
        double getRate() const; 

        /** \brief Main process 
        * 
        * Main process flow
        * 
        **/
        void process();
                    
        /** \brief Fill output messages
        * 
        * Fills main output and debug messages
        */
        void publish() const;
        
    protected: 
        
        /** \brief Inverse kinematics for the AGV 
         * 
         * Inverse kinematics for the AGV. Zero side-slip model
         * Given a goal expressed as (x-dot,y-dot,theta-dot), updates steering 
         * and wheel rate commands (class members: sfl_, sfr_, sbl_, sbr_ and wfl_, wfr_, wbl_, wbr_)
         * 
         **/
        void invKinematics(const Eigen::Vector3d & _pt); 
        
        /** \brief stops platform
         * 
         * Sets all steering and wheel rate commands to 0
         * 
         **/
        void stop(); 

        /** \brief  Trajectory subscriber callback
         * 
         * Trajectory subscriber callback
         * 
         **/
        void trajectoryCallback(const cargo_ants_msgs::ReferenceTrajectory::ConstPtr& _msg);


        /** \brief  TeleOp subscriber callback
         * 
         * TeleOp subscriber callback
         * 
         **/
        void teleopCallback(const geometry_msgs::Twist::ConstPtr& _msg);
        
};
#endif

