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

Eigen::MatrixXd Heuristic::compute(int nVehicles, int nTasks, int nDim, int rDim, Eigen::MatrixXd DeltaMatrix, int eta)
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
    
    std::vector <std::vector<double>> chunks; //should be equal to number of vehicles
	std::vector <std::vector<double>> chunks_path; //should be equal to number of vehicles
    vector<int>cuts;
    vector<int>cut;
	int cutNo;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    if (eta == 0) //first heuristic method
    {
		cout << "THE FIRST APPROXIMATION METHOD" << endl;
		double val = 0;
		double addChunk = 0;
		int count =0;
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
								{j=j-1;}
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
		cutNo = cut.size();
		cout <<"\nNo of cuts:"<<cutNo << endl;
        
		vector<double> temp_path;
		vector<double> temp_path_value;
		vector<double> reduced_chunk;
		
		for (int i = 0; i< paths_values_list[path_index].size(); i++)
		{reduced_chunk.push_back(paths_values_list[path_index][i]);}

		for (int i = 0; i< cut.size(); i++)
		{reduced_chunk.erase(std::remove(reduced_chunk.begin(), reduced_chunk.end(), paths_values_list[path_index][cut[i]]), reduced_chunk.end());}
    
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
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

	else if (eta == 1) //second heuristic method 
    {	
		cout << "THE SECOND APPROXIMATION METHOD" << endl;
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
		//vector<int> cuts; //should be equal to (n-1)
		cutNo = nVehicles-1;
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
	cutNo = cutNo+1;	
	}
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    
	else if (eta == 2) //the third method
	{
		cout << "THE THIRD APPROXIMATION METHOD" << endl;
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
		cutNo = nVehicles-1;
    
		int lc = 0; int sc=0;
		//find number of large and small chunks
		for (int i = 0; i< paths_values_list[path_index].size(); i++)
			{if (paths_values_list[path_index][i] > chunk_length)
			++lc;
			else ++sc;
			}
    
		if (lc < 1) {lc = lc+1;}
    
		vector <int> chk;
		vector <vector <int>> mainchk;
		vector <int> inlist; // to track the nodes that are added
		vector <int> outlist; // to track the nodes that are yet to be added
		int lc_count = 0;
		int sc_count = 0;
		int repeat = 0;
		int exp_lc = nVehicles - lc;
		int ratio = nTasks/nVehicles;
    
		for (int i = 0; i< paths_list[path_index].size(); i++)
		{outlist.push_back(paths_list[path_index][i]);}

		for ( std::vector<std::vector<int>>::size_type i = 0; i < outlist.size(); i++ )
		{ std::cout << outlist[i] << ' ';}
   
		//dividing the biggest chunks
		auto it=task_map.begin();
		while (lc_count < (exp_lc+repeat))
		{
		  // find the first largest chunk
		  std::vector<int>::iterator i1 = std::find(std::begin(inlist), std::end(inlist), get<1>(*it));
		  if (i1 != std::end(inlist)) 
		  {  //cout << "\n first already exist" << endl;
			 std::vector<int>::iterator i2 = std::find(std::begin(inlist), std::end(inlist), get<2>(*it));
			 if (i2 != std::end(inlist)) 
				{ //cout << "\n second already exist" << endl;
					++repeat;}
			 else
				{
					//cout << "first if else" << endl;
					chk.push_back(get<2>(*it));
					inlist.push_back(get<2>(*it));
					outlist.erase(std::remove(outlist.begin(), outlist.end(), get<2>(*it)), outlist.end());
					mainchk.push_back(chk); ++lc_count;
					chk.clear();
				}
		  }
		  else
		  {
			//cout << "first else" << endl;
			chk.push_back(get<1>(*it));
			inlist.push_back(get<1>(*it));
			outlist.erase(std::remove(outlist.begin(), outlist.end(), get<1>(*it)), outlist.end());
			mainchk.push_back(chk); ++lc_count;
			chk.clear();
			
			std::vector<int>::iterator i2 = std::find(std::begin(inlist), std::end(inlist), get<2>(*it));
		    if (i2 != std::end(inlist)) 
				{ //cout << "\n second already exist" << endl;
					}
			 else
				{
					//cout << "second else else" << endl;
					chk.push_back(get<2>(*it));
					inlist.push_back(get<2>(*it));
					outlist.erase(std::remove(outlist.begin(), outlist.end(), get<2>(*it)), outlist.end());
					mainchk.push_back(chk); ++lc_count;
					chk.clear();
				}
		  }
		  ++it;
		}
	  
		cout <<"\n Newchunks are"<< endl;
		for ( std::vector<std::vector<int>>::size_type i = 0; i < mainchk.size(); i++ )
		{for ( std::vector<int>::size_type j = 0; j < mainchk[i].size(); j++ )
			{std::cout << mainchk[i][j] << ' ';}
			std::cout << std::endl;}
    
		/* cout << "lc_count " << lc_count << endl;
		cout << "sc_count " << sc_count << endl;
		cout << "exp_lc " << exp_lc << endl;
		cout << "lc is " << lc << endl;
		cout << "sc is " << sc << endl;*/
    
		int repeat2 = 0;
		//now adding small chunks for the remaining
		int ncount = nVehicles - lc_count;
	
		for (auto it=task_map.rbegin(); it!=task_map.rbegin()+ncount; ++it)
		{
			std::vector<int>::iterator i1 = std::find(std::begin(inlist), std::end(inlist), get<1>(*it));
			if (i1 != std::end(inlist)) 
			{  //cout << "\n SC first already exist" << endl;
			 std::vector<int>::iterator i2 = std::find(std::begin(inlist), std::end(inlist), get<2>(*it));
			 if (i2 != std::end(inlist)) 
				{ //cout << "\n SC second already exist" << endl; 
					++repeat2;}
			 else
				{
					//cout << "SC first if else" << endl;
					chk.push_back(get<2>(*it));
					inlist.push_back(get<2>(*it));
					outlist.erase(std::remove(outlist.begin(), outlist.end(), get<2>(*it)), outlist.end());
					mainchk.push_back(chk); ++sc_count;
					chk.clear();
				}
			}
			else
			{
				std::vector<int>::iterator i2 = std::find(std::begin(inlist), std::end(inlist), get<2>(*it));
				if (i2 != std::end(inlist)) 
				{ //cout << "\n SC2 second already exist" << endl;
					}
				else
				{
					chk.push_back(get<1>(*it));
					inlist.push_back(get<1>(*it));
					outlist.erase(std::remove(outlist.begin(), outlist.end(), get<1>(*it)), outlist.end());

					chk.push_back(get<2>(*it));
					inlist.push_back(get<2>(*it));
					outlist.erase(std::remove(outlist.begin(), outlist.end(), get<2>(*it)), outlist.end());
					
					mainchk.push_back(chk); ++sc_count;
					chk.clear();
			
					//cout << "both added" << endl;
				}
		  }
		}

		//now adding the remaining missing nodes 
		// cout << "\new" <<mainchk.size() << endl;
		int diff = nVehicles - mainchk.size();
    
		if (mainchk.size() != nVehicles)
		{
			for (int i = 0; i < diff; i++)
			{
				std::cout << outlist[i] << ' ' << endl;
				chk.push_back(outlist[i]);
				inlist.push_back(outlist[i]);
				outlist.erase(std::remove(outlist.begin(), outlist.end(), outlist[i]), outlist.end());
				mainchk.push_back(chk);
				chk.clear();
			}
		}
		vector <double> dummy;
		cout <<"\n Newchunks are"<< endl;
		for ( std::vector<std::vector<int>>::size_type i = 0; i < mainchk.size(); i++ )
			{for ( std::vector<int>::size_type j = 0; j < mainchk[i].size(); j++ )
			{   dummy.push_back(mainchk[i][j]);
				std::cout << mainchk[i][j] << ' ';}
	    chunks.push_back(dummy);
	    dummy.clear();
		std::cout << std::endl;}
    
		//now adding the remaining missing nodes depending upon the chunk_length
    
		cout <<"\ninlist size is "<< inlist.size()<< " with:"<<endl;
		for ( int i = 0; i < inlist.size(); i++ )
			{std::cout << inlist[i] << ' ';}
    
		cout <<"\noutlist size is "<< outlist.size()<< " with:"<<endl;
		for ( int i = 0; i < outlist.size(); i++ )
			{std::cout << outlist[i] << ' ';}
    
		cout <<"\n finalchunks are"<< endl;
		for ( std::vector<std::vector<int>>::size_type i = 0; i < chunks.size(); i++ )
			{for ( std::vector<int>::size_type j = 0; j < chunks[i].size(); j++ )
				{std::cout << chunks[i][j] << ' ';}
				std::cout << std::endl;}
    
		std::vector <int> dub_pairs_first;
		std::vector <int> dub_pairs_second;

		while (!outlist.empty())
		{		
			for (int i =0 ; i< nVehicles;i++)
				{
					//cout << "i is " << i << endl;
					int first = chunks[i][0];
					int c = chunks[i].size()-1;
					int last = chunks[i][c];
					int front_element, back_element;
		
					//cout << "chunk[i][first]"<< first << endl;
					std::vector<int>::iterator it = std::find(paths_list[path_index].begin(), paths_list[path_index].end(), first);
					if (it != paths_list[path_index].end()) 
					{
						//cout << "E " << chunks[i][0] << endl;
						int max_ind = std::distance(paths_list[path_index].begin(), it);
						//	cout << "max_ind " << max_ind << endl;
						if (max_ind == 0)
							{front_element = INT_MAX;}
						else
							{front_element = paths_list[path_index][max_ind  - 1];}
						dub_pairs_first.push_back(front_element);
					}
				
					//cout << "chunk[i][last]"<< last << endl;
					std::vector<int>::iterator it3 = std::find(paths_list[path_index].begin(), paths_list[path_index].end(), last);
					if (it3 != paths_list[path_index].end()) 
					{
						int max_ind = std::distance(paths_list[path_index].begin(), it3);
						//	cout << "max_ind " << max_ind << endl;
						if (max_ind == paths_list[path_index].size()-1)
							{back_element = INT_MAX;}
						else
							{back_element = paths_list[path_index][max_ind  + 1];}
							dub_pairs_second.push_back(back_element);
						//	cout << "////////////////" << endl;
					}
						
					cout << "front " << front_element << endl;
					cout << "end " << back_element << endl;
				}
		cout <<"\nDubpairs first is"<< endl;
		for ( int i = 0; i < dub_pairs_first.size(); i++ )
			{std::cout << "first " <<dub_pairs_first[i] << endl;}
  
		cout <<"\nDubpairs second is"<< endl;
		for ( int i = 0; i < dub_pairs_second.size(); i++ )
		{std::cout << "second " <<dub_pairs_second[i] << endl;}
    
		for ( int i = 0; i < dub_pairs_first.size(); i++ )
		{ 
				if (std::find(inlist.begin(),inlist.end(), dub_pairs_first[i]) != inlist.end() )
					{ //do nothing
					}
				else
					{
						if (dub_pairs_first[i] < 1000)
							{
								chunks[i].insert(chunks[i].begin(),dub_pairs_first[i]);
								inlist.insert(inlist.begin(),dub_pairs_first[i]);
								outlist.erase(std::remove(outlist.begin(), outlist.end(), dub_pairs_first[i]), outlist.end());}
					}
		}
		for ( int i = 0; i < dub_pairs_second.size(); i++ )
		{ 	if (std::find(inlist.begin(),inlist.end(), dub_pairs_second[i]) != inlist.end() )
			{ //do nothing
			}
			else
			{
			  if (dub_pairs_second[i] < 1000)
			  {
				chunks[i].insert(chunks[i].begin()+chunks[i].size(),dub_pairs_second[i]);
				inlist.insert(inlist.begin(),dub_pairs_second[i]);
				outlist.erase(std::remove(outlist.begin(), outlist.end(), dub_pairs_second[i]), outlist.end());}
			  }
		}
		dub_pairs_first.clear();
		dub_pairs_second.clear();
	  		
		cout <<"\ninlist size is "<< inlist.size()<< " with:"<<endl;
		for ( int i = 0; i < inlist.size(); i++ )
			{std::cout << inlist[i] << ' ';}
    
		cout <<"\noutlist size is "<< outlist.size()<< " with:"<<endl;
		for ( int i = 0; i < outlist.size(); i++ )
		{std::cout << outlist[i] << ' ';}
		}
	
	cutNo = cutNo+1;	
	}
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
     
    else
    {cout << "\nMETHOD NOT SPECIFIED" << endl;}   
	////////////////////////////////////////////////////////////////////////////////////////////////////////

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
            
    //displaySolution(CostMatrix, DeltaMatrix, TVec);//Printing out the solution
    //publish(nVehicles, plotString);
    return CostMatrix;

}
  

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
