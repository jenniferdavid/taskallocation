/*
 * main.cpp
 *
 *  Created on: Apr 18, 2016
 *      Author: kardee13
 */

#include "Dstar.h"
#include<iostream>
#include<stack>
#include<queue>
#include<list>
#include<ext/hash_map>
#include<cmath>
#include<cstdio>
#include<cstdlib>

using namespace std;

int main() {

 Dstar *dstar = new Dstar();
 list<state> mypath;

 dstar->init(0,0,29,32);         // set start to (0,0) and goal to (10,5)
 //dstar->updateCell(3,0,-1);     // set cell (3,4) to be non traversable
 dstar->updateCell(9,1,-1);     // set cell (3,4) to be non traversable
 dstar->updateCell(9,2,-1);     // set cell (3,4) to be non traversable
 //dstar->updateCell(9,3,-1);     // set cell (3,4) to be non traversable
 //dstar->updateCell(3,3,-1);     // set cell (3,4) to be non traversable
 //dstar->updateCell(2,2,42.432); // set set (2,2) to have cost 42.432

 dstar->replan();               // plan a path
 mypath = dstar->getPath();     // retrieve path

 for (std::list<state>::iterator it=mypath.begin(); it != mypath.end(); ++it)
 	    std::cout << ' ' << it->x << ' ' << it->y <<'\n';

 printf("Path achieved\n");
/* dstar->updateStart(10,2);      // move start to (10,2)
 dstar->replan();               // plan a path

 mypath = dstar->getPath();     // retrieve path

 dstar->updateGoal(0,1);        // move goal to (0,1)
 dstar->replan();               // plan a path
 mypath = dstar->getPath();     // retrieve path
*/

  cout << "Exiting the main..." << endl;
 return 0;
}



