#ifndef agv_odom_sim_node_H
#define agv_odom_sim_node_H

//Eigen includes
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

//STD/STL cpp
#include <iostream>
#include <math.h>

//standard ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

//This ROS package. Dynamic configure
#include <agv_odom_sim/agv_odom_sim_paramsConfig.h>



/** \brief Implementation of AGV simulated odometry
 *
 * Implementation of AGV simulated odometry
 * 
 */
class AgvOdomSimNode
{
    protected: 
        //ros node handle
        ros::NodeHandle nh_;
               
        //trajectory subscriber
        ros::Subscriber ground_truth_sub_;
        
        //output odometry publishers
        ros::Publisher odom_pub_;
        
        //odometry transform broadcatser
        tf::TransformBroadcaster tfb_;
        
        //wished update rate of the main loop, [hz]        
        double rate_;
        
        //flag indicating new integration values
        bool odom_flag_;
        
        //integrated 2D position
        double x_, y_, th_; 
        
        //last callback time stamp
        ros::Time last_callback_stamp_;
        
        //noisy versions of received ground truth linear speed and rotational rate. To be published as the current twist
        double v_, w_;
        
        //noise parameters. ratios of odometry readings to compute noise std deviations.         
        double rstdev_v;
        double rstdev_w; 
        
        //random generator and distribution for noise addition
        std::default_random_engine rand_generator_;
        std::normal_distribution<double> noise_dist_;
        
    public:
        /** \brief Default constructor
        * 
        * This constructor initializes the node. 
        * 
        */
        AgvOdomSimNode();

        /** \brief Destructor
        * 
        * This destructor frees all necessary dynamic memory allocated within this class.
        */
        ~AgvOdomSimNode();
        
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
//         void process();
                    
        /** \brief Fill output messages
        * 
        * Fills main output and debug messages
        */
        void publish();
        
    protected: 
        
        /** \brief  Trajectory subscriber callback
         * 
         * Trajectory subscriber callback
         * 
         **/
        void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& _msg);
        
};
#endif

