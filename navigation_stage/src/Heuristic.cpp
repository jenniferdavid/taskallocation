#include <fstream>
#include <sstream>
#include <math.h>
#include <iomanip> // needed for setw(int)
#include <string>
#include "stdio.h"
#include <iostream>
#include <cstdlib>
#include <Eigen/Dense>
#include <Eigen/LU> 
#include <vector>
#include <cmath>
#include <algorithm>
#include <time.h>
#include <limits>
//#include "gnuplot_iostream.h"
#include <cstring>
#include <sys/stat.h>
#include <ctime>
#include <sys/types.h>
#include <locale.h>
#include <wchar.h>
#include <unistd.h>
#include <csignal>
#include <map>
#include "Hungarian.h"
#include "ros/ros.h"
#include "navi_msgs/Item.h"
#include "navi_msgs/ItemStruct.h"
#include "navi_msgs/Problem.h"
#include "navi_msgs/Goals.h"
#include "navi_msgs/GoalsList.h"
#include "navi_msgs/nameTasks.h"
#include "navi_msgs/listIntTasks.h"
#include "navi_msgs/listTasks.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Float64MultiArray.h"
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace Eigen;

std::string outputpath = "/home/jendav/Videos/potts_spin_nn/Matrices/inputs/OUTPUTS/";
std::string inputpath = "/home/jendav/Videos/potts_spin_nn/Matrices/inputs/";
std::string solStrA;
std::string solStrB;
std::string veh_alpha = "ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890";
std::string task_alpha = "abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@";

char tasks[] = "abcdefghijklmnopqrstuvwxyz{|}~^_`!#$&'()*+,-./:;<=>?@abcdefghijklmnopqrstuvwxyz";

std::vector<char> plotVehicle;
std::vector<std::vector<char> >plotString;

int nVehicles;      
int nTasks;       
int nDim; 
int rDim; 
    
double checkTime;
Eigen::VectorXd cTime;
Eigen::VectorXd checkTimeVec;
Eigen::VectorXd sub;
Eigen::MatrixXd DeltaMatrix;
Eigen::MatrixXd CostMatrix;
Eigen::MatrixXd TaskMatrix;
Eigen::MatrixXd EndMatrix;
Eigen::MatrixXd StartMatrix;
Eigen::MatrixXd EndHungMatrix;
Eigen::MatrixXd StartHungMatrix;
Eigen::MatrixXd TotalEndHungMatrix;
Eigen::MatrixXd TotalStartHungMatrix;
Eigen::VectorXd TVec; 
Eigen::VectorXd Edge; 
Eigen::VectorXd Edge2; 
Eigen::VectorXd Edge3; 

navi_msgs::Goals data;
navi_msgs::GoalsList msg;
navi_msgs::listTasks stringD1;
navi_msgs::listIntTasks intD2;
navi_msgs::nameTasks stringD3;
geometry_msgs::PoseArray poseArrayMsg;
geometry_msgs::Pose poseMsg;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SubPub
{
    public:
    std::vector<std::pair<std::string,std::tuple<double,double,double>> > totalCoord;

    //constructor function that publishes and subscribes
    SubPub()
    {
        probSub = n.subscribe("/problem", 100, &SubPub::probCallback,this);
        deltamatSub = n.subscribe("/deltamat", 100, &SubPub::deltaMatCallback,this);
        coordSub = n.subscribe("/coords", 100, &SubPub::coordsCallback,this);
        
        goalPub = n.advertise<navi_msgs::Goals>("/goals", 100);
        taskPub = n.advertise<navi_msgs::GoalsList>("/tasks", 100);
       // Heuristic();
       // void publish(int n, std::vector<std::vector<char> > pS);
    }
        
void deltaMatCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {   
        DeltaMatrix = MatrixXd::Ones(nDim,nDim);
        int k = 0;
        for (int i=0;i<nDim;i++)
        {
            for (int j=0;j<nDim;j++)
            {   
                DeltaMatrix(i,j) = msg->data[k];
                k++;
            }
        }
    }

void probCallback(const navi_msgs::Problem::ConstPtr& msg)
    {
        nVehicles = msg->nRobots;
        nTasks= msg->nTasks;
        nDim = msg->nTotalmodels;
        rDim = msg->nModels;
    }
        
    //callback for subscribing /coords 
void coordsCallback(const navi_msgs::ItemStruct::ConstPtr& msg)
    {
        for (int i=0; i< nDim; ++i) //msg->coords.size()
        {
            const navi_msgs::Item &data = msg->coords[i];
            totalCoord.push_back(std::make_pair(data.name,(std::make_tuple(data.x, data.y, data.yaw))));
        }
     Heuristic();
     //publish();
    }

    //parses the solution
std::tuple<std::string, std::string, int, std::vector <std::vector <char>> > displaySolution(Eigen::MatrixXd &VMatrix, Eigen::MatrixXd &DeltaMatrix, Eigen::VectorXd &TVec) 
    {   
        int indx = 0;
        int indxB = 0;
        cTime = Eigen::VectorXd(nVehicles);
        sub = VectorXd::Ones(nVehicles);
      
        cout << "\nTHE SOLUTION is: \n" << endl;
        cout << VMatrix << endl;
           
        for (int i = 0; i < nVehicles; i++)
            {
                for (int j = 0; j < nDim; j++)
                    {if (VMatrix(i,j)==1)
                        {indx = j;}}
                if (i == 0)
                {
                    solStrA = std::string("S") + veh_alpha[i] + std::string(" -> ") + task_alpha[indx-nVehicles];
                    plotVehicle.push_back((veh_alpha[i]));
                    plotVehicle.push_back((task_alpha[indx-nVehicles]));
                    solStrB = "max(" + std::to_string(TVec(i)) + std::string(" + ")+ std::to_string(DeltaMatrix(i,indx)) + std::string(" + ") + std::to_string(TVec(indx));
                }
                else
                {
                    solStrA = solStrA + std::string(" & S") + veh_alpha[i] + std::string(" -> ") + task_alpha[indx-nVehicles];
                    if (!plotVehicle.empty())
                        {plotString.push_back(plotVehicle);}
                    plotVehicle.clear();
                    plotVehicle.push_back((veh_alpha[i]));
                    plotVehicle.push_back((task_alpha[indx-nVehicles]));
                    solStrB = solStrB + std::string(", ") + std::to_string(TVec(i)) + std::string(" + ")+ std::to_string(DeltaMatrix(i,indx)) + std::string(" + ") + std::to_string(TVec(indx));
                }
                cTime[i] = DeltaMatrix(i,indx) + TVec(indx) + TVec(i);
            
                while (indx <= (nDim-nVehicles-1))
                {
                    for (int j = 0; j < nDim; j++)
                        {if (VMatrix(indx,j)==1)
                            {indxB = j;}}
               
                    if (indxB > (nVehicles+nTasks-1))
                    {   
                        solStrA = solStrA + std::string(" -> E") + veh_alpha[indxB-nVehicles-nTasks];
                        plotVehicle.push_back((veh_alpha[indxB-nVehicles-nTasks]));
                        if (!plotVehicle.empty())
                            {plotString.push_back(plotVehicle);}
                        plotVehicle.clear();
                        solStrB = solStrB + std::string(" + ") + std::to_string(DeltaMatrix(indx,indxB)) + std::string(" + ") +std::to_string(TVec(i));
                        cTime[i] = cTime[i] + DeltaMatrix(indx,indxB) + TVec(i);
                        solStrB = solStrB + std::string(" = ") + std::to_string(cTime[i]);            
                    }   
                    else
                    {    
                        solStrA = solStrA + std::string(" -> ") + task_alpha[indxB-nVehicles];
                        plotVehicle.push_back((task_alpha[indxB-nVehicles]));
                        solStrB = solStrB + std::string(" + ") + std::to_string(DeltaMatrix(indx,indxB)) + std::string(" + ") + std::to_string(TVec(indxB));
                        cTime[i] = cTime[i] + DeltaMatrix(indx,indxB) + TVec(indxB);
                    }
                    indx = indxB;
                }
            }
        solStrB = solStrB + std::string(")");
        checkTimeVec = VectorXd(2*nVehicles);
        checkTimeVec << cTime, cTime + (nVehicles * sub);
        VectorXf::Index maxE;
        checkTime = checkTimeVec.maxCoeff(&maxE) - nVehicles;
        cout << "The tasks are ordered as:\n";
        cout << "\n" <<solStrA << endl;
        cout << "\n" <<solStrB << endl;
        cout << "\n" <<checkTime << endl;
        
        for ( std::vector<std::vector<char>>::size_type i = 0; i < plotString.size(); i++ )
            {for ( std::vector<char>::size_type j = 0; j < plotString[i].size(); j++ )
                {std::cout << plotString[i][j] << ' ';}
                std::cout << std::endl;} 
       // publish(nVehicles, plotString);
        return std::make_tuple(solStrA, solStrB, checkTime, plotString);    
    }

void publish(int nVehicles, std::vector<std::vector<char> >plotString)
{
    int q = 0;
    msg.input.nRobots = nVehicles;
    msg.input.nTasks = nTasks;
    msg.input.nModels = rDim;
    msg.input.nTotalmodels = nDim;
    msg.input.nDropoffs = nVehicles;

    while (q < nVehicles)
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
                    
                    for(int i = nVehicles; i<totalCoord.size(); i++)  //pose of all tasks 
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
                    int z = x - 64 + nTasks; //int of all tasks                    
                    std::string dt = "dropoff"+std::to_string(z);
                    stringD3.namTask = dt;
                    data.nT.push_back(stringD3);
                    
                    //pose of the dropoff
                    for (int i = nTasks; i<totalCoord.size(); i++)
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

void Heuristic()
{
    TaskMatrix = MatrixXd::Ones(nTasks,nTasks);
    TVec = VectorXd(nDim);
    for (int i=0; i<nDim; i++)
        {TVec(i) = 1;}
    
    //subscribe to DeltaMatrix
    cout << "\nDeltaMatrix is:" << DeltaMatrix << endl;

    DeltaMatrix.diagonal().array() = 10000000000;
    DeltaMatrix.leftCols(nVehicles) *= 10000000000;
    DeltaMatrix.bottomRows(nVehicles) *= 10000000000;
    DeltaMatrix.topRightCorner(nVehicles,nVehicles) = DeltaMatrix.bottomLeftCorner(nVehicles,nVehicles).eval();       
    
    Eigen::MatrixXd TaskMatrix = DeltaMatrix.block(nVehicles,nVehicles,nTasks,nTasks);
    std::cout <<"TaskMatrix is\n" << TaskMatrix <<endl;
  
    int node = 0; // every task loop
    int start_node; //start node of every task loop
    std::vector<std::vector<int> > paths_list; //paths of each of the task
    std::vector <double> values; //minimum values of all the paths
    std::vector<std::vector<double> > paths_values_list; //paths of each of the task

    LABEL0:
    start_node = node;
    std::vector<int> visited; //Global unvisited vertices vector
    std::vector <int> paths;
    paths.push_back(node);
    std::vector <double>value_paths; //stores the values of each of the path of each task
    double sum_of_elems = 0;
    int visitNo = 0;
    LABEL: 
    visitNo++;
    visited.push_back(start_node);
    Edge = VectorXd((nTasks)); 
    for(int i = 0; i < ((nTasks)); i++)
        {
            if (start_node==i)
                Edge(i) = INT_MAX;
            else if (std::find(visited.begin(),visited.end(),i) != visited.end() )
                Edge(i) = INT_MAX;
            else
            {Edge(i) = TaskMatrix(start_node,i);} //min dist from start to each of neighbour
    //cout<<"\nEdge is"<<Edge(i)<<endl;
        } 
    
    MatrixXf::Index minI;
    double minValue = Edge.minCoeff(&minI);
    //cout<<"\nminVal is"<<minValue<<endl;
    //cout<<"\nminI is"<<minI<<endl;
    paths.push_back(minI);
    value_paths.push_back(minValue);
    //cout<<"\nvisited vertices are"<<visitNo<<endl;    
    start_node = minI;
    //value of the path    
    
    if (visitNo == (nTasks-1))
        {node++;
        paths_list.push_back(paths);
        paths_values_list.push_back(value_paths);
        std::for_each(value_paths.begin(), value_paths.end(), [&] (double n) 
        {sum_of_elems += n;});
        values.push_back(sum_of_elems);
        cout << "\n////////////////////////////////////////////////////////////////" << endl;
        if (node == (nTasks))
            {goto LABEL1;} //paths for all the tasks found
        else
            goto LABEL0; //run for the next task available
        }
    else
        {goto LABEL;} //run till all the tasks are done for that task loop
    LABEL1:

    for ( std::vector<std::vector<int> >::size_type i = 0; i < paths_list.size(); i++ )
    {for ( std::vector<int>::size_type j = 0; j < paths_list[i].size(); j++ )
    {std::cout << paths_list[i][j] << ' ';}
    std::cout << std::endl;}
    cout << "\n////////////////////////////////////////////////////////////////" << endl;
       
    for ( std::vector<std::vector<int> >::size_type i = 0; i < paths_values_list.size(); i++ )
    {for ( std::vector<int>::size_type j = 0; j < paths_values_list[i].size(); j++ )
    {std::cout << paths_values_list[i][j] << ' ';}
    std::cout << std::endl;}
    cout << "\n////////////////////////////////////////////////////////////////" << endl;

    std::cout << "The smallest path is " << *std::min_element(std::begin(values),std::end(values)) << '\n';
    auto smallest = std::min_element(values.begin(), values.end() );
    double avg1 = std::accumulate(values.begin(), values.end(), 0LL) / values.size();
    std::cout << "The avg is: " << avg1 << endl;
    std::vector<double>::iterator first = values.begin();
    int  path_index = std::distance(first,smallest);
    std::cout << "The distance is: " << path_index << endl;
  
    for ( std::vector<int>::size_type j = 0; j < nTasks; j++ )
    {std::cout << paths_list[path_index][j] << ' ';}
    std::cout << std::endl; // the path array
             
    for ( std::vector<int>::size_type j = 0; j < paths_values_list[path_index].size(); j++ )
    {   
        std::cout << paths_values_list[path_index][j] << ' ';
    } //the path array with values

    std::vector<std::pair <int,int>> edge;
    
    double chunk_length = *smallest/(nVehicles);
    cout << "\nchunk_length " << chunk_length<< endl;
    cout << "\n////////////////////////////////////////////////////////////////" << endl;
    double val = 0;
    double addChunk = 0;
    int count =0;
    vector<int>cut;
   // cut.push_back(0);
      
    for ( std::vector<int>::size_type j = 0; j < paths_values_list[path_index].size(); j++ )
    {  
       if (count != (nVehicles-1))
       {
       val = val + paths_values_list[path_index][j];
       addChunk = paths_values_list[path_index][j];

       if (addChunk > chunk_length)
       {           
           cut.push_back(j-1);
           count ++;
           val = 0;
       }
       
       if (val > chunk_length)
       {
        cut.push_back(j-1);
        cout << "adding j-1= "<<j-1<<endl;
        count++;
        val = 0;
        if ((j != 0) || (j != paths_values_list[path_index].size()-1))
        j=j-1;
        else
            break;
        }
       }
    } 
    int check = 0;
    cout <<"before"<<endl;
    for (int i=0; i<cut.size(); i++) 
    { 
        cout <<cut[i] << " " ;
    } 
    if ( std::any_of(cut.begin(), cut.end(), [](int i){return i<0;}))
    {check=1;}
    
    cut.push_back(nTasks-1);
    cout << "\nafter" <<endl;
    for (int i=0; i<cut.size(); i++) 
    { 
       if (check ==1)
        { cut[i] = cut[i]+1;}
         cout <<cut[i] << " ";
    } 
    cout << "\n////////////////////////////////////////////////////////////////" << endl;
    int cutNo = cut.size();
    cout <<"\nNo of cuts:"<<cutNo << endl;
        
    vector<double> temp_path;
    vector<double> temp_path_value;
    vector<double> reduced_chunk;
    std::vector <std::vector<double>> chunks; //should be equal to number of vehicles
    std::vector <std::vector<double>> chunks_path; //should be equal to number of vehicles
    
    for (int i = 0; i< paths_values_list[path_index].size(); i++)
    {
        reduced_chunk.push_back(paths_values_list[path_index][i]);
    }

    for (int i = 0; i< cut.size(); i++)
    {
    reduced_chunk.erase(std::remove(reduced_chunk.begin(), reduced_chunk.end(), paths_values_list[path_index][cut[i]]), reduced_chunk.end());
    }
    
    int i =0;
    for ( std::vector<int>::size_type j = 0; j < paths_list.size(); j++ )
        {  temp_path.push_back(paths_list[path_index][j]);
           if (j == cut[i])
           { chunks.push_back(temp_path); i++;
               temp_path.clear();}
        }
    
    int yy =0; int zz=0;
    for ( std::vector<int>::size_type j = 0; j < paths_values_list.size(); j++ )
        {  
           temp_path_value.push_back(paths_values_list[path_index][j]);
           if(std::find(temp_path_value.begin(),temp_path_value.end(), paths_values_list[path_index][cut[zz]]) != temp_path_value.end() )
           {temp_path_value.erase(std::remove(temp_path_value.begin(), temp_path_value.end(), paths_values_list[path_index][cut[zz]]), temp_path_value.end()); zz++;}
           if (j == cut[yy]-1)
           {     
               chunks_path.push_back(temp_path_value); yy++;
               temp_path_value.clear();
           }
        }
        
    cout <<"\nChunks are"<< endl;
    for ( std::vector<std::vector<int>>::size_type i = 0; i < chunks.size(); i++ )
    {for ( std::vector<int>::size_type j = 0; j < chunks[i].size(); j++ )
    {std::cout << chunks[i][j] << ' ';}
    std::cout << std::endl;}
    
    cout <<"\nChunks value are"<< endl;
    for ( std::vector<std::vector<int>>::size_type i = 0; i < chunks_path.size(); i++ )
    {for ( std::vector<int>::size_type j = 0; j < chunks_path[i].size(); j++ )
    {std::cout << chunks_path[i][j] << ' ';}
    std::cout << std::endl;}
    
    //start points
    vector<int> start_points;
    vector<int> end_points;
    vector<int> sh_points;
    vector<vector<int>> start_hung_points;

    cout <<"\nStart and End points are"<< endl;
    for ( std::vector<std::vector<int>>::size_type i = 0; i < chunks.size(); i++ )
    {start_points.push_back(chunks[i][0]);
        cout << start_points[i]<<endl;
    end_points.push_back(chunks[i][(chunks[i].size())-1]);
        cout << end_points[i]<<endl;
    }
    
    cout <<"\nStart Hung points are"<< endl;
    for ( std::vector<std::vector<int>>::size_type i = 0; i < chunks.size(); i++ )
    {    if ((chunks[i].size())-1 > 0)
        {for ( std::vector<std::vector<int>>::size_type j = 1; j < (chunks[i].size())-1; j++ )
            {sh_points.push_back(chunks[i][j]);}}
        else if ((chunks[i].size())-1 == 0)
           {//sh_points.push_back(chunks[i][0]);
               
        } 
        else
        {}
        start_hung_points.push_back(sh_points);
        sh_points.clear();
    }
     
    for ( std::vector<std::vector<int>>::size_type i = 0; i < start_hung_points.size(); i++ )
    {for ( std::vector<int>::size_type j = 0; j < start_hung_points[i].size(); j++ )
    {std::cout << start_hung_points[i][j] << ' ';}
    std::cout << std::endl;}
    
    //std::cout <<"DeltaMatrix is\n" << DeltaMatrix <<endl;
    Eigen::MatrixXd StartMatrix = DeltaMatrix.block(0,nVehicles,nVehicles,nTasks);
    std::cout <<"\nStartMatrix is\n" << StartMatrix <<endl;
    
    Eigen::MatrixXd EndMatrix = DeltaMatrix.block(nVehicles,nVehicles+nTasks,nTasks,nVehicles);
    std::cout <<"\nEndMatrix is\n" << EndMatrix <<endl;
    
    Eigen::MatrixXd StartHungMatrix = MatrixXd::Zero(nVehicles,nVehicles);
    Eigen::MatrixXd EndHungMatrix = MatrixXd::Zero(nVehicles,nVehicles);
    Eigen::MatrixXd TotalStartHungMatrix = MatrixXd::Zero(nVehicles,nVehicles);
    
    std::vector<std::vector<double>> v1; //double rows of start hung matrix
    std::vector<std::vector<double>> v2; //double rows of end hung matrix
    std::vector<double> v3; //rows of start hung matrix
    std::vector<double> v4; //rows of end hung matrix
    std::vector<std::vector<double>> v5; //double rows of total_start_hung matrix
    std::vector<double> v6; //rows of total_start hung matrix
    std::cout <<"\nTaskMatrix is\n" << TaskMatrix <<endl;
    
     for (int i = 0; i < cutNo; i++)
        {
            for (int j = 0; j < cutNo; j++)
            {
                StartHungMatrix(i,j) =  StartMatrix(j,start_points[i]);  
                v3.push_back(StartMatrix(j,start_points[i]));
            }
            v1.push_back(v3);
            v3.clear();
        }
    std::cout <<"\nStartHungMatrix is\n" << StartHungMatrix <<endl;
    TotalStartHungMatrix = StartHungMatrix;
    
    for (int i = 0; i < cutNo; i++)
        {
            for (int j = 0; j < cutNo; j++)
            {
                if((start_hung_points[i].size()) > 0)
                 { TotalStartHungMatrix(i,j) = TotalStartHungMatrix(i,j) + TaskMatrix(start_points[i],start_hung_points[i][0]);
                     if ((start_hung_points[i].size()) > 1)
                     {for(int k=0; k<(start_hung_points[i].size()-1); k++)
                     { //cout << "\ntask value is:" << TaskMatrix(start_hung_points[i][k],start_hung_points[i][k+1]) << endl;
                     TotalStartHungMatrix(i,j) = TotalStartHungMatrix(i,j) + TaskMatrix(start_hung_points[i][k],start_hung_points[i][k+1]);  
                     }}
                }   
                else {}
            }
        }
    std::cout <<"\nTotalStartHungMatrix is\n" << TotalStartHungMatrix <<endl;
    
    for (int i = 0; i < cutNo; i++)
        {
            for (int j = 0; j < cutNo; j++)
            {
                 EndHungMatrix(i,j) = EndMatrix(end_points[i],j); 
                 v4.push_back(EndMatrix(end_points[i],j));
            }
            v2.push_back(v4);
            v4.clear();
        }
    std::cout <<"\nEndHungMatrix is\n" << EndHungMatrix <<endl;
    
    //total start hung matrix in vector form
    for ( std::vector<std::vector<int>>::size_type i = 0; i < nVehicles; i++ )
    {for ( std::vector<int>::size_type j = 0; j < nVehicles; j++ )
    {
        v6.push_back(TotalStartHungMatrix(i,j));
    }
    v5.push_back(v6);
    v6.clear();
    }
    
    std::cout <<"\nTotalStartHungMatrix in vector form is" <<endl;
    for ( std::vector<std::vector<int>>::size_type i = 0; i < v5.size(); i++ )
    {for ( std::vector<int>::size_type j = 0; j < v5[i].size(); j++ )
    {std::cout << v5[i][j] << ' ';}
    std::cout << std::endl;}   
    
    std::cout <<"\nEndHungMatrix in vector form is" <<endl;
    //end hung matrix in vector form
    for ( std::vector<std::vector<int>>::size_type i = 0; i < v2.size(); i++ )
    {for ( std::vector<int>::size_type j = 0; j < v2[i].size(); j++ )
    {std::cout << v2[i][j] << ' ';}
    std::cout << std::endl;}    
     
    HungarianAlgorithm HungAlgo;
    vector<int> assignment;
    vector<int> assignment2;
    double start_cost = HungAlgo.Solve(v1, assignment);
    double end_cost = HungAlgo.Solve(v2, assignment2);
    cout << "\n////////////////////////////////////////////////////////////////" << endl;
    for (unsigned int x = 0; x < v1.size(); x++)
    {
     std::cout << x << "," << assignment[x] << "\n";
    }
    cout << "\n////////////////////////////////////////////////////////////////" << endl;
    for (unsigned int x = 0; x < v2.size(); x++)
    {
     std::cout << x << "," << assignment2[x] << "\n";
    }
    cout << "\n////////////////////////////////////////////////////////////////" << endl;

    chunks.clear();
    temp_path.clear();
    int z =0;
    for ( std::vector<int>::size_type j = 0; j < paths_list.size(); j++ )
        {  
           temp_path.push_back(paths_list[path_index][j]+nVehicles);
           if (j == cut[z])
           { 
               temp_path.insert(temp_path.begin(),assignment[z]);
               temp_path.push_back(assignment2[z]+nVehicles+nTasks);
               chunks.push_back(temp_path); z++;
               temp_path.clear();}
        }
     
    for ( std::vector<std::vector<int>>::size_type i = 0; i < chunks.size(); i++ )
    {for ( std::vector<int>::size_type j = 0; j < chunks[i].size(); j++ )
    {std::cout << chunks[i][j] << ' ';}
    std::cout << std::endl;}
    cout << "\n////////////////////////////////////////////////////////////////" << endl;
    cout << "\nTotal number of vehicles are: " << chunks.size() <<endl;

    int count2 = 0;
    for ( std::vector<std::vector<int>>::size_type i = 0; i < chunks.size(); i++ )
    {std::cout << "\nVehicle " <<i+1<<" does "<<chunks[i].size()-2 <<" tasks"<<endl;
    count2 = count2 + chunks[i].size()-2;}
    cout << "\nTotal number of tasks are: " << count2 <<endl;
    cout << "\n////////////////////////////////////////////////////////////////" << endl;

    Eigen::MatrixXd CostMatrix = MatrixXd::Zero(nDim,nDim);
    vector<double>total_cost;
    double cost;
    for ( std::vector<int>::size_type y = 0; y < nVehicles; y++ )
    {
        cost=0;
        for ( std::vector<int>::size_type j = 0; j < chunks[y].size()-1; j++ )
        {cost = cost+DeltaMatrix(chunks[y][j],chunks[y][j+1]);
         CostMatrix(chunks[y][j],chunks[y][j+1]) = 1;
        }
        cost = cost + chunks[y].size()-2;
        total_cost.push_back(cost);
        cout << "\nTotal cost is " <<total_cost[y] << endl;
    }   
    
    cout << "\nCostMatrix is:\n" << CostMatrix << endl;
    cout << "\n////////////////////////////////////////////////////////////////" << endl;

    for (unsigned int x = 0; x < total_cost.size(); x++)
    {
     std::cout << total_cost[x] << "\n";
    }
    cout << "\n////////////////////////////////////////////////////////////////" << endl;
           
    displaySolution(CostMatrix, DeltaMatrix, TVec);//Printing out the solution
    publish(nVehicles, plotString);

}
    private:
        ros::NodeHandle n;
        ros::Subscriber coordSub;
        ros::Subscriber deltamatSub;
        ros::Subscriber probSub;
        ros::Publisher goalPub;
        ros::Publisher taskPub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "HeuristicNode"); 
    SubPub sp;
    ros::spin();

    return 0;
}

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
