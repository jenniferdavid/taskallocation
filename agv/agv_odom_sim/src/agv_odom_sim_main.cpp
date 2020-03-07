
//this package dependencies
#include "agv_odom_sim_node.h"

//node main
int main(int argc, char **argv)
{
      //init ros
      ros::init(argc, argv, "agv_odom_sim_node");
      
      //create ros wrapper object
      AgvOdomSimNode agv_odom;
      
      //set node loop rate
      ros::Rate looprate(agv_odom.getRate());
      
      //node loop 
      while ( ros::ok() )
      {
            //execute pending callbacks
            ros::spinOnce(); 

            //publish
            agv_odom.publish();
                        
            //relax to fit output rate
            looprate.sleep();            
      }
            
      //exit program
      return 0;
}