#include "agv_odom_sim_node.h"

AgvOdomSimNode::AgvOdomSimNode() :
    nh_(ros::this_node::getName()), 
    rate_(50), //default at 50Hz
    odom_flag_(false),
    x_(0),
    y_(0),
    th_(0),
    noise_dist_(0,1) //Gaussian noise. Normalized
{
    //set up subscribers with callbacks
    ground_truth_sub_ = nh_.subscribe("ground_truth", 100, &AgvOdomSimNode::groundTruthCallback, this);
    
    //advertise publishers
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 100);
    
    //create a normal distribution. TODO: Get params from yaml file thorugh ros node handler
    nh_.getParam("odom_noise_stddev_ratio_v", rstdev_v);
    nh_.getParam("odom_noise_stddev_ratio_w", rstdev_w);
    
    //init time management variable
    last_callback_stamp_ = ros::Time::now();
}

AgvOdomSimNode::~AgvOdomSimNode()
{
    //
}

double AgvOdomSimNode::getRate() const
{
    return rate_;
}

// void AgvOdomSimNode::process()
// {
//     //to do 
// }
// 

void AgvOdomSimNode::publish()
{
    //publish only in case of new odometry
    if (odom_flag_) 
    {
        //reset odom_flag_
        odom_flag_ = false; 

        //fill & publish output message
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = last_callback_stamp_; 
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "agv_footprint";
        
        //pose xyz
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0;
        
        //pose orientation
        tf::Quaternion qt = tf::createQuaternionFromYaw(th_);
        odom_msg.pose.pose.orientation.x = qt.getX();
        odom_msg.pose.pose.orientation.y = qt.getY();
        odom_msg.pose.pose.orientation.z = qt.getZ();
        odom_msg.pose.pose.orientation.w = qt.getW();
        
        //twist
        odom_msg.twist.twist.linear.x = v_; //TODO: backward displacement case not published
        odom_msg.twist.twist.angular.z = w_;
        
        //publish
        odom_pub_.publish(odom_msg);
        
        //fill and broadcast transform
        tf::Transform odom2base;
        odom2base.setOrigin( tf::Vector3(x_, y_, 0.0) );
        odom2base.setRotation(qt);
        tfb_.sendTransform( tf::StampedTransform(odom2base, last_callback_stamp_, "odom", "agv_footprint") );  
    }
}

void AgvOdomSimNode::groundTruthCallback(const nav_msgs::Odometry::ConstPtr& _msg)
{
    //aux values
    double dT; //time increment
    ros::Duration dT_ros; //time between consecutive odometry callbacks
    //double v_th; //linear velocity orientation (on the gorund)
    
    //get dT and set last_callback_stamp_ to be used in the next callback
    dT_ros = _msg->header.stamp - last_callback_stamp_; 
    last_callback_stamp_ = _msg->header.stamp; 
    
    //compute v linear wrt vehicle forward axis (X axis), assuming no slipping (no Y axis velocity)
    v_ = sqrt(_msg->twist.twist.linear.x*_msg->twist.twist.linear.x + _msg->twist.twist.linear.y*_msg->twist.twist.linear.y);
    //v_th = atan2( _msg->twist.twist.linear.y,_msg->twist.twist.linear.x);
    
    //add noise to input odometry linear and rotational velocities. rstdev_x are ratios of odometry reading to get noise stddev
    v_ = v_ + noise_dist_(rand_generator_)*rstdev_v*v_;
    w_ = _msg->twist.twist.angular.z + noise_dist_(rand_generator_)*rstdev_w*w_;
    
    //integrates noisy velocities to get new position
    dT = dT_ros.toSec();
    th_ = th_ + 0.5*dT*w_; 
    x_ = x_ + v_*dT*cos(th_); //TODO: backward displacement case not integrated
    y_ = y_ + v_*dT*sin(th_); //TODO: backward displacement case not integrated
    th_ = th_ + 0.5*dT*w_; 
    
    //Bound th_ within -pi,pi (assumes within -2pi,2pi before bounding, so less than pi increment in a single iteration)
    if ( th_ > M_PI ) th_ = -2*M_PI + th_;
    if ( th_ < -M_PI ) th_ = 2*M_PI + th_;
    
    //debugging
//     std::cout << "v: " << v_ << std::endl;
//     std::cout << "w: " << w_ << std::endl;
//     std::cout << "th: " << th_ << std::endl;
//     std::cout << "-----------------------" << std::endl;
    
    
    //rise odom_flag_
    odom_flag_ = true; 
    
}