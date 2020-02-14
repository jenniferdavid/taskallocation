#include "task_subpub.h"

using namespace std;
using namespace Eigen;

//constructor function that publishes and subscribes
TaskSubPub::TaskSubPub()
    {
        probSub = n.subscribe("/problem", 100, &TaskSubPub::probCallback,this);
        deltamatSub = n.subscribe("/deltamat", 100, &TaskSubPub::deltaMatCallback,this);
        coordSub = n.subscribe("/coords", 100, &TaskSubPub::coordsCallback,this);
        ros::Duration(10).sleep();
        goalPub = n.advertise<navi_msgs::Goals>("/goals", 100);
        taskPub = n.advertise<navi_msgs::GoalsList>("/tasks", 100);
    }
        
//TaskSubPub::~TaskSubPub(){}

    //callback for subscribing /deltamat 
void TaskSubPub::deltaMatCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {   
        h.DeltaMatrix = MatrixXd::Ones(h.nDim,h.nDim);
        int k = 0;
        for (int i=0;i<h.nDim;i++){
            for (int j=0;j<h.nDim;j++){   
                h.DeltaMatrix(i,j) = msg->data[k];
                k++;}}
    }

    //callback for subscribing /problem 
void TaskSubPub::probCallback(const navi_msgs::Problem::ConstPtr& msg)
    {
        h.nVehicles = msg->nRobots;
        h.nTasks= msg->nTasks;
        h.nDim = msg->nTotalmodels;
        h.rDim = msg->nModels;
    }
        
    //callback for subscribing /coords 
void TaskSubPub::coordsCallback(const navi_msgs::ItemStruct::ConstPtr& msg)
    {
        for (int i=0; i< h.nDim; ++i) //msg->coords.size()
        {const navi_msgs::Item &data = msg->coords[i];
        totalCoord.push_back(std::make_pair(data.name,(std::make_tuple(data.x, data.y, data.yaw))));}
        h.compute(h.nVehicles,h.nTasks,h.nDim,h.rDim,h.DeltaMatrix);
    }

    //parses the solution
std::tuple<std::string, std::string, int, std::vector <std::vector <char>> > TaskSubPub::displaySolution(Eigen::MatrixXd &VMatrix, Eigen::MatrixXd &DeltaMatrix, Eigen::VectorXd &TVec) 
    {   
        int indx = 0;
        int indxB = 0;
        cTime = Eigen::VectorXd(h.nVehicles);
        sub = VectorXd::Ones(h.nVehicles);
      
        cout << "\nTHE SOLUTION is: \n" << endl;
        cout << VMatrix << endl;
           
        for (int i = 0; i < h.nVehicles; i++)
            {
                for (int j = 0; j < h.nDim; j++)
                    {if (VMatrix(i,j)==1)
                        {indx = j;}}
                if (i == 0)
                {
                    solStrA = std::string("S") + veh_alpha[i] + std::string(" -> ") + task_alpha[indx-h.nVehicles];
                    plotVehicle.push_back((veh_alpha[i]));
                    plotVehicle.push_back((task_alpha[indx-h.nVehicles]));
                    solStrB = "max(" + std::to_string(h.TVec(i)) + std::string(" + ")+ std::to_string(h.DeltaMatrix(i,indx)) + std::string(" + ") + std::to_string(h.TVec(indx));
                }
                else
                {
                    solStrA = solStrA + std::string(" & S") + veh_alpha[i] + std::string(" -> ") + task_alpha[indx-h.nVehicles];
                    if (!plotVehicle.empty())
                        {plotString.push_back(plotVehicle);}
                    plotVehicle.clear();
                    plotVehicle.push_back((veh_alpha[i]));
                    plotVehicle.push_back((task_alpha[indx-h.nVehicles]));
                    solStrB = solStrB + std::string(", ") + std::to_string(h.TVec(i)) + std::string(" + ")+ std::to_string(h.DeltaMatrix(i,indx)) + std::string(" + ") + std::to_string(h.TVec(indx));
                }
                cTime[i] = h.DeltaMatrix(i,indx) + h.TVec(indx) + h.TVec(i);
            
                while (indx <= ((h.nDim-h.nVehicles)-1))
                {
                    for (int j = 0; j < h.nDim; j++)
                        {if (VMatrix(indx,j)==1)
                            {indxB = j;}}
               
                    if (indxB > (h.nVehicles+h.nTasks-1))
                    {   
                        solStrA = solStrA + std::string(" -> E") + veh_alpha[indxB-h.nVehicles-h.nTasks];
                        plotVehicle.push_back((veh_alpha[indxB-h.nVehicles-h.nTasks]));
                        if (!plotVehicle.empty())
                            {plotString.push_back(plotVehicle);}
                        plotVehicle.clear();
                        solStrB = solStrB + std::string(" + ") + std::to_string(h.DeltaMatrix(indx,indxB)) + std::string(" + ") +std::to_string(h.TVec(i));
                        cTime[i] = cTime[i] + h.DeltaMatrix(indx,indxB) + h.TVec(i);
                        solStrB = solStrB + std::string(" = ") + std::to_string(cTime[i]);            
                    }   
                    else
                    {    
                        solStrA = solStrA + std::string(" -> ") + task_alpha[indxB-h.nVehicles];
                        plotVehicle.push_back((task_alpha[indxB-h.nVehicles]));
                        solStrB = solStrB + std::string(" + ") + std::to_string(h.DeltaMatrix(indx,indxB)) + std::string(" + ") + std::to_string(h.TVec(indxB));
                        cTime[i] = cTime[i] + h.DeltaMatrix(indx,indxB) + h.TVec(indxB);
                    }
                    indx = indxB;
                }
            }
        solStrB = solStrB + std::string(")");
        checkTimeVec = VectorXd(2*h.nVehicles);
        checkTimeVec << cTime, cTime + (h.nVehicles * sub);
        VectorXf::Index maxE;
        checkTime = checkTimeVec.maxCoeff(&maxE) - h.nVehicles;
        cout << "The tasks are ordered as:\n";
        cout << "\n" <<solStrA << endl;
        cout << "\n" <<solStrB << endl;
        cout << "\n" <<checkTime << endl;
        
        for ( std::vector<std::vector<char>>::size_type i = 0; i < plotString.size(); i++ )
            {for ( std::vector<char>::size_type j = 0; j < plotString[i].size(); j++ )
                {std::cout << plotString[i][j] << ' ';}
                std::cout << std::endl;} 
        return std::make_tuple(solStrA, solStrB, checkTime, plotString);    
    }

    //Publishes tasks 
void TaskSubPub::publishTasks(int nVehicles, std::vector<std::vector<char> >plotString)
{
    int q = 0;
    msg.input.nRobots = h.nVehicles;
    msg.input.nTasks = h.nTasks;
    msg.input.nModels = h.rDim;
    msg.input.nTotalmodels = h.nDim;
    msg.input.nDropoffs = h.nVehicles;

    while (q < h.nVehicles)
    {
        data.robotName = q;
        //publish robot name
        data.name = "robot"+std::to_string(q); 
        std::cout << data.name << " - robot name" << endl;
        
        //publish no of tasks
        data.tasks = plotString[q].size() - 1; 
        std::cout << data.tasks << " - no of tasks" << endl;

        for (int j=1; j< plotString[q].size();j++)       //for each robot -> the list of tasks
             {   
                 string s(1,plotString[q][j]);           //convert char to string
                 stringD1.ltask = s;  
                 /*publish string of task*/
                 data.lT.push_back(stringD1);  
                 int x = (int)plotString[q][j];          //int of all tasks
               
                if (j != plotString[q].size() -1)
                {    
                    int y = x - 96;                      //int of all tasks
                    intD2.liTask = y;
                    /*publish int of tasks*/
                    data.lIT.push_back(intD2);  
                    
                    std::string t = "task"+std::to_string(y);
                    stringD3.namTask = t;     
                    /*publish string+int tasks*/
                    data.nT.push_back(stringD3);
                    
                    for(int i = h.nVehicles; i<totalCoord.size(); i++)  //pose of all tasks 
                    {    
                        if (t.compare(totalCoord[i].first) != 0)
                            {
                                //nothing
                            }
                        else
                            {
                                poseMsg.position.x = std::get<1>(totalCoord[i].second);
                                poseMsg.position.y = -1 * (std::get<0>(totalCoord[i].second));
                                
                                double radians = (std::get<2>(totalCoord[i].second)) * (M_PI/180);
                                tf::Quaternion quaternion;
                                quaternion = tf::createQuaternionFromYaw(radians);
                                geometry_msgs::Quaternion qMsg;
                                tf::quaternionTFToMsg(quaternion, qMsg);
                                poseMsg.orientation = qMsg;
                            }
                      }
                }
                else
                {
                    int z = x - 64 + h.nTasks; //int of all tasks                    
                    std::string dt = "dropoff"+std::to_string(z);
                    stringD3.namTask = dt;
                    data.nT.push_back(stringD3);
                    
                    //pose of the dropoff
                    for (int i = h.nTasks; i<totalCoord.size(); i++)
                    {
                        if (dt.compare(totalCoord[i].first) == 0)
                            {
                                poseMsg.position.x = std::get<1>(totalCoord[i].second);
                                poseMsg.position.y = -1 * (std::get<0>(totalCoord[i].second));
                                
                                double radians = (std::get<2>(totalCoord[i].second)) * (M_PI/180);
                                tf::Quaternion quaternion;
                                quaternion = tf::createQuaternionFromYaw(radians);
                                geometry_msgs::Quaternion qMsg;
                                tf::quaternionTFToMsg(quaternion, qMsg);
                                poseMsg.orientation = qMsg;
                            }
                    }
                }
             poseArrayMsg.header.stamp = ros::Time::now(); // timestamp of creation of the msg
             poseArrayMsg.header.frame_id = "map" ;
             poseArrayMsg.poses.push_back(poseMsg);
            }
     data.pList= poseArrayMsg;
     goalPub.publish(data);
     msg.list.push_back(data);
     q++;       
     data.lT.clear();  
     data.lIT.clear();
     data.nT.clear();
     poseArrayMsg.poses.clear();
    }
    taskPub.publish(msg);
}

   

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
