
//this package dependencies
#include "agv_drive_sim_node.h"

//node main
int main(int argc, char **argv)
{
      //init ros
      ros::init(argc, argv, "agv_drive_node");
      
      //create ros wrapper object
      AgvDriveNode agv;
      
      //set node loop rate
      ros::Rate looprate(agv.getRate());
      
      //node loop 
      while ( ros::ok() )
      {
            //execute pending callbacks
            ros::spinOnce(); 

            //do things
            agv.process();
            
            //publish
            agv.publish();
                        
            //relax to fit output rate
            looprate.sleep();            
      }
            
      //exit program
      return 0;
}