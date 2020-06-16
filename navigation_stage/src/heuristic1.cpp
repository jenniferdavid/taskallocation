//////////////////////////////////////////////////////////////////////////////
// Heuristic.cpp: Implementation file for Class HungarianAlgorithm.
// 
// This is a C++ wrapper with slight modification of the approximation algorithm implementation by Turpin.
// 
// Both this code and the orignal code are published under the BSD license.
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
    std::vector<std::vector<int>> paths_list; //paths of each of the task
    std::vector <double> values; //minimum values of all the paths
    std::vector<std::vector<double>> paths_values_list; //paths of each of the task

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
    cout<<"\nEdge is"<<Edge(i)<<endl;
        } 
    
    MatrixXf::Index minI;
    double minValue = Edge.minCoeff(&minI);
    cout<<"\nminVal is"<<minValue<<endl;
    cout<<"\nminI is"<<minI<<endl;
    paths.push_back(minI);
    value_paths.push_back(minValue);
    cout<<"\nvisited vertices are"<<visitNo<<endl;    
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

    for ( std::vector<std::vector<int>>::size_type i = 0; i < paths_list.size(); i++ )
    {for ( std::vector<int>::size_type j = 0; j < paths_list[i].size(); j++ )
    {std::cout << paths_list[i][j] << ' ';}
    std::cout << std::endl;}
    cout << "\n////////////////////////////////////////////////////////////////" << endl;
       
    for ( std::vector<std::vector<int>>::size_type i = 0; i < paths_values_list.size(); i++ )
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
    {std::cout << paths_values_list[path_index][j] << ' ';} //the path array with values
    
    double chunk_length = *smallest/(nVehicles);
    cout << "\nchunk_length " << chunk_length<< endl;
    cout << "\n////////////////////////////////////////////////////////////////" << endl;
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //create a tuple for holding path values and nodes
    std::vector <tuple<double, int, int> > task_map;

    for (int i = 0; i< paths_values_list[path_index].size(); i++)
    {task_map.push_back(std::make_tuple (paths_values_list[path_index][i], paths_list[path_index][i], paths_list[path_index][i+1] ));}
   
    //sorting them according to size in descending order
    sort(task_map.begin(), task_map.end()); 
    reverse(task_map.begin(), task_map.end()); 

	//printing them
    std::cout << "taskMap contains:\n";
    for(auto const &i : task_map)
        {cout<<get<0>(i)<<" ==> "<<get<1>(i)<<"  "<<get<2>(i)<<endl;}
    cout << "\n////////////////////////////////////////////////////////////////" << endl;
    
    //we cut the (n-1) biggest chunks which does not have any repeated nodes
    vector<int> cuts; //should be equal to (n-1)
    int cutNo = nVehicles-1;
    vector <vector <double>> chunks;
	vector<int> recheck;
    vector<int> recheck2;
    int scount = 0;
    int exp_cuts = (nVehicles-1);
    vector<double> temp_path;
    
    //checking how many repeats of the nodes
    for (auto it=task_map.begin(); it!=task_map.end(); ++it)
      {
		recheck.push_back(get<2>(*it));
		std::vector<int>::iterator i1 = std::find(std::begin(recheck), std::end(recheck), get<1>(*it));
		if (i1 != std::end(recheck)) 
			{++scount;}
	  }

	cout << "\nscount is:" << scount <<endl;

   if (nVehicles == nTasks)
    {
		for (int i =0; i< nVehicles; ++i)
		{cuts.push_back(i);
		cout <<cuts[i] << " ";}
		
    cout <<"\nNo of cuts:"<<cutNo << endl;
	}
	else
	{
        for (auto itr=task_map.begin(); itr!=task_map.end(); ++itr)
		{
			if (abs(exp_cuts - scount) < (nVehicles/2))    
			{
				//check if the biggest chunk node (get<1>(*itr)) value is in the paths_list
				std::vector<int>::iterator is = std::find(std::begin(paths_list[path_index]), std::end(paths_list[path_index]), get<1>(*itr)); 
				if (is != std::end(paths_list[path_index])) //if found
					{
						int ind = std::distance(paths_list[path_index].begin(), is); //find the index to add it to cut
						cout << "..... " << get<1>(*itr) << " and found at " << ind << endl;
						cuts.push_back(ind);
					}
			}
			else
			{
				std::vector<int>::iterator i3 = std::find(std::begin(recheck2), std::end(recheck2), get<2>(*itr));
				if (i3 != std::end(recheck2)) 
				{
					//do nothing  
					cout << "here" << endl;
				}
				else
				{
					recheck2.push_back(get<2>(*itr)); //check if the second value of the biggest node is there in the present search
					std::vector<int>::iterator i2 = std::find(std::begin(recheck2), std::end(recheck2), get<1>(*itr));
						if (i2 != std::end(recheck2)) 
							{
								cout << "here2" << endl;
							}
						else
						{
							std::vector<int>::iterator is = std::find(std::begin(paths_list[path_index]), std::end(paths_list[path_index]), get<1>(*itr));
							if (is != std::end(paths_list[path_index])) 
							{   int ind = std::distance(paths_list[path_index].begin(), is);
								cout << "searching " << get<1>(*itr) << " and found at " << ind << endl;
								cuts.push_back(ind);
								recheck2.push_back(get<1>(*itr));
							}
						}
				}
			}			
			}
	
    cout << "\nCuts original size" << endl;
    for (int i=0; i<cuts.size(); i++) 
    {cout <<cuts[i] << " ";} 
     
    cout << "\nCuts after pruning" << endl;
	//cuts.erase(cuts.begin(), cuts.size() > (nVehicles-1) ?  cuts.begin() + (nVehicles-1) : cuts.end() );
    cuts.erase(cuts.begin()+(nVehicles-1), cuts.end());
    for (int i=0; i<cuts.size(); i++) 
    {cout <<cuts[i] << " ";} 
                
    cout << "\nCuts after pruning and sorting" << endl;
    sort(cuts.begin(), cuts.end());
    for (int i=0; i<cuts.size(); i++) 
    {cout <<cuts[i] << " ";} 
	
	cutNo = cuts.size();
    cout <<"\nNo of cuts:"<<cutNo << endl;
}    

    int i =0;
    for ( std::vector<int>::size_type j = 0; j < paths_list.size(); j++ )
        {  
			if (i < cuts.size())
				{temp_path.push_back(paths_list[path_index][j]);
					if (j == cuts[i])
						{ chunks.push_back(temp_path); i++;
						temp_path.clear();}
				}
			if (i >= cuts.size()) //the last chunk
				{temp_path.push_back(paths_list[path_index][j+1]);
				if (j == paths_list.size()-2)
				{chunks.push_back(temp_path);}
				}
        }
        
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////

    cout <<"\nChunks are"<< endl;
    for ( std::vector<std::vector<int>>::size_type i = 0; i < chunks.size(); i++ )
    {for ( std::vector<int>::size_type j = 0; j < chunks[i].size(); j++ )
    {std::cout << chunks[i][j] << ' ';}
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
    {   
		if ((chunks[i].size())-2 > 0)
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
    
    std::cout <<"\nTaskMatrix is\n" << TaskMatrix <<endl;
    
     for (int i = 0; i < cutNo+1; i++)
        {
            for (int j = 0; j < cutNo+1; j++)
            {
                StartHungMatrix(i,j) =  StartMatrix(j,start_points[i]);  
                v3.push_back(StartMatrix(j,start_points[i]));
            }
            v1.push_back(v3);
            v3.clear();
        }
    std::cout <<"\nStartHungMatrix is\n" << StartHungMatrix <<endl;
    
    for (int i = 0; i < cutNo+1; i++)
        {
            for (int j = 0; j < cutNo+1; j++)
            {
                 EndHungMatrix(i,j) = EndMatrix(end_points[i],j); 
                 v4.push_back(EndMatrix(end_points[i],j));
            }
            v2.push_back(v4);
            v4.clear();
        }
    std::cout <<"\nEndHungMatrix is\n" << EndHungMatrix <<endl;
    
    
    std::cout <<"\nStartHungMatrix in vector form is" <<endl;
    //start hung matrix in vector form
    for ( std::vector<std::vector<int>>::size_type i = 0; i < v1.size(); i++ )
    {for ( std::vector<int>::size_type j = 0; j < v1[i].size(); j++ )
    {std::cout << v1[i][j] << ' ';}
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
    
    cout << "\n////////////ASSIGNMENT OF START POINTS////////////////" << endl;
    for (unsigned int x = 0; x < v1.size(); x++)
    {
     std::cout << x << " assigned to " << assignment[x] << "\n";
    }
    cout << "\n///////////ASSIGNMENT OF END POINTS/////////////////////" << endl;
    for (unsigned int x = 0; x < v2.size(); x++)
    {
     std::cout << x << " assigned to " << assignment2[x] << "\n";
    }
    cout << "\n////////////////////////////////////////////////////" << endl;
    
    int z =0;
    for ( std::vector<int>::size_type i = 0; i < chunks.size(); i++ )
	{
        chunks[i].insert(chunks[i].begin(),(assignment[z]-nVehicles));
		for ( std::vector<int>::size_type j = 0; j < chunks[i].size(); j++ )
			{  chunks[i][j] = chunks[i][j] + nVehicles;}
        chunks[i].push_back(assignment2[z]+nVehicles+nTasks);
        z++;
    }
           
    for ( std::vector<std::vector<int>>::size_type i = 0; i < chunks.size(); i++ )
    {for ( std::vector<int>::size_type j = 0; j < chunks[i].size(); j++ )
    {std::cout << chunks[i][j] << ' ';}
    std::cout << std::endl;}
    
    cout << "\n////////////////////////////////////////////////////////////////" << endl;
    cout << "\nTotal number of vehicles are: " << chunks.size() <<endl;
    int count2 = 0;
    for ( std::vector<std::vector<int>>::size_type i = 0; i < chunks.size(); i++ )
    {std::cout << "\nVehicle " << (chunks[i][0] + 1)<<" does "<<chunks[i].size()-2 <<" tasks"<<endl;
    count2 = count2 + chunks[i].size()-2;}
    cout << "\nTotal number of tasks are: " << count2 <<endl;
    cout << "\n////////////////////////////////////////////////////////////////" << endl;

    Eigen::MatrixXd CostMatrix = MatrixXd::Zero(nDim,nDim);  
    vector<double>total_cost;
    double cost;
    
    for ( std::vector<int>::size_type i = 0; i < nVehicles; i++ )
    {
        cost=0;
        for ( std::vector<int>::size_type j = 0; j < chunks[i].size()-1; j++ )
        {cost = cost+DeltaMatrix(chunks[i][j],chunks[i][j+1]);
         CostMatrix(chunks[i][j],chunks[i][j+1]) = 1;
        }
        cost = cost + chunks[i].size()-2;
        total_cost.push_back(cost);
        cout << "\nTotal cost is " <<total_cost[i] << endl;
    }   
    
    cout << "\nCostMatrix is:\n" << CostMatrix << endl;
    cout << "\n////////////////////////////////////////////////////////////////" << endl;

    for (unsigned int x = 0; x < total_cost.size(); x++)
    {
     std::cout << total_cost[x] << "\n";
    }
    cout << "\n////////////////////////////////////////////////////////////////" << endl;
           
    return CostMatrix;

}
  

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
