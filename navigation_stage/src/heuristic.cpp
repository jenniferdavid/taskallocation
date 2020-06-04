//////////////////////////////////////////////////////////////////////////////
// Heuristic.cpp: Implementation file for Class HungarianAlgorithm.
// 
// This is a C++ wrapper with slight modification of the approximation algorithm implementation by Turpin.
// 
// Both this code and the original code are published under the BSD license.
// by David J, 2019
// 

#include <stdlib.h>
#include <cfloat> // for DBL_MAX
#include <cmath>  // for fabs()
#include "hungarian.h"
#include "heuristic.h"

using namespace std;
using namespace Eigen;

Heuristic::Heuristic(){}

Eigen::MatrixXd Heuristic::compute(int nVehicles, int nTasks, int nDim, int rDim, Eigen::MatrixXd DeltaMatrix)
{
    cout << "\nnvehicle is:" << nVehicles<< endl;
    cout << "\ntask is:" << nTasks << endl;
    cout << "\nndim is:" << rDim << endl;
    cout << "\nrdim is:" << nDim << endl;

    
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
           
    //displaySolution(CostMatrix, DeltaMatrix, TVec);//Printing out the solution
    //publish(nVehicles, plotString);
    return CostMatrix;

}
  

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
