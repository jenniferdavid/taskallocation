
//ros dependencies
#include "ros/ros.h" //required
#include <cargo_ants_msgs/ArtsArray.h> //out message

//visp dependencies
//...

//std dependencies
//...

//constants. Do not necessay if waiting for VISP is a blocking instruction
const double LOOP_RATE = 2; //Hz

//node main
int main(int argc, char **argv)
{
    //init ros
    ros::init(argc, argv, "arts_driver");
    
    //ros node handle
    ros::NodeHandle nh;

    // publisher
    ros::Publisher arts_publisher;
    
    //output message
    cargo_ants_msgs::ArtsArray arts_msg;    
    
    //set node loop rate. Do not necessay if waiting for VISP is a blocking instruction
    ros::Rate node_rate(LOOP_RATE);    
    
    //init publisher
    arts_publisher = nh.advertise<cargo_ants_msgs::ArtsArray>("/arts_ranges", 100);
    
    //flas indicating new IVSP data is received
    bool new_arts_data = false;
    
    //node loop 
    while ( ros::ok() )
    {
	//waiting IVSP message
	//to do
      
	//Fill ROS message(s). Convert IVSP message to ROS . Set flags for received data
	//to do
	arts_msg.header.stamp = ros::Time::now();
	//arts_msg.measuremnts[0].remote_id = ..
        //arts_msg.measuremnts[0].range = ..
	//arts_msg.measuremnts[0].snr = ..
	// ...
        new_arts_data = true;
      
	//publish ad reset flags
	if(new_arts_data) 
	{
	  arts_publisher.publish(arts_msg);
	  new_arts_data = false;
	}

        //relax to fit output rate. Do not necessay if waiting for VISP is a blocking instruction
        node_rate.sleep();            
    }
        
    //exit program
    return 0;
}
