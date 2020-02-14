/*
 * Dstar.cpp
 *
 *  Created on: Apr 18, 2016
 *      Author: kardee13
 */
#include<iostream>
#include<stack>
#include<queue>
#include<list>
#include<ext/hash_map>
#include<cmath>
#include<cstdlib>

using namespace std;
#include "Dstar.h"

Dstar::Dstar()
{
  maxSteps = 80000;  // node expansions before we give up
  C1       = 1;      // cost of an unseen cell
  k_m = 0;
  cout << "Default constructor" << endl;
}


float Dstar::keyHashCode(state u) {

  return (float)(u.k.first + 1193*u.k.second);

}

bool Dstar::isValid(state u)
{

  ds_oh::iterator cur = openHash.find(u);
  if (cur == openHash.end())
	  return false;
  if (!close(keyHashCode(u), cur->second))
	  return false;
  return true;

}

list<state> Dstar::getPath()
{
  return path;
}

bool Dstar::occupied(state u)
{

  ds_ch::iterator cur = cellHash.find(u);

  if (cur == cellHash.end())
	  return false;
  return (cur->second.cost < 0);
}

void Dstar::init(int sX, int sY, int gX, int gY) {

  cellHash.clear();
  path.clear();
  openHash.clear();

  while(!openList.empty()) openList.pop();

  k_m = 0;

  s_start.x = sX;
  s_start.y = sY;
  s_goal.x  = gX;
  s_goal.y  = gY;

  cout << "Start_X = " << sX
       << "\nStart_Y = " << sY
       << "\nGoal_X = " << gX
       << "\nGoal_Y = " << gY << endl;

  cellInfo tmp;
  tmp.g = tmp.rhs =  0;
  tmp.cost = C1;

  cellHash[s_goal] = tmp;
  tmp.g = tmp.rhs = heuristic(s_start,s_goal);
  tmp.cost = C1;
  cellHash[s_start] = tmp;
  s_start = calculateKey(s_start);

  s_last = s_start;
cout << "Initialized!!" << endl;
}


void Dstar::makeNewCell(state u)
{

  if (cellHash.find(u) != cellHash.end()) return;

  cellInfo tmp;
  tmp.g       = tmp.rhs = heuristic(u,s_goal);
  tmp.cost    = C1;
  cellHash[u] = tmp;

}


double Dstar::getG(state u)
{
  //cout << "Inside getG" << endl;
  if(cellHash.find(u) == cellHash.end())
  {
    return heuristic(u,s_goal);
  }
  return cellHash[u].g;

}


double Dstar::getRHS(state u) {

  //cout << "Inside getRHS" << endl;
  if (u == s_goal) return 0;

  if (cellHash.find(u) == cellHash.end())
  {
    return heuristic(u,s_goal);
  }
  return cellHash[u].rhs;

}

void Dstar::setG(state u, double g) {

  makeNewCell(u);
  cellHash[u].g = g;
}


void Dstar::setRHS(state u, double rhs) {

  makeNewCell(u);
  cellHash[u].rhs = rhs;

}


double Dstar::eightCondist(state a, state b)
{
  double temp;
  double min = abs(a.x - b.x);
  double max = abs(a.y - b.y);
  cout << "Min = " << min << " Max = " << max << endl;
  if (min > max) {
    temp = min;
    min = max;
    max = temp;
  }
  double ret = ((M_SQRT2-1.0)*min + max);
  cout << ret << endl;
  return ret; //M_SQRT2 = 1.414 i.e Sqrt(2)
}


int Dstar::computeShortestPath() {
  cout << "Into the computeShortestPath function....." << endl;
  list<state> s;
  list<state>::iterator i;

  if (openList.empty()) return 1;

  int k=0;
  while ((!openList.empty()) &&
         ((openList.top() < (s_start = calculateKey(s_start))) ||
         (getRHS(s_start) != getG(s_start)))) {

    if (k++ > maxSteps) {
      //fprintf(stderr, "At maxsteps\n");
      return -1;
    }


    state u;

    bool test = (getRHS(s_start) != getG(s_start));

    // lazy remove
    while(1) {
      if (openList.empty()) return 1;
      u = openList.top();
      openList.pop();

      if (!isValid(u)) continue;
      if (!(u < s_start) && (!test)) return 2;
      break;
    }

    ds_oh::iterator cur = openHash.find(u);
    openHash.erase(cur);

    state k_old = u;

    if (k_old < calculateKey(u)) { // u is out of date
      insert(u);
    } else if (getG(u) > getRHS(u)) { // needs update (got better)
      setG(u,getRHS(u));
      getPred(u,s);
      for (i=s.begin();i != s.end(); i++) {
        updateVertex(*i);
      }
    } else {   // g <= rhs, state has got worse
      setG(u,INFINITY);
      getPred(u,s);
      for (i=s.begin();i != s.end(); i++) {
        updateVertex(*i);
      }
      updateVertex(u);
    }
  }
  cout << "Exiting the shortest path function..." << endl;
  return 0;
}


bool Dstar::close(double x, double y)
{

  if (isinf(x) && isinf(y)) return true;
  return (fabs(x-y) < 0.00001);

}


void Dstar::updateVertex(state u) {

  list<state> s;
  list<state>::iterator i;

  if (u != s_goal) {
    getSucc(u,s);
    double tmp = INFINITY;
    double tmp2;

    for (i=s.begin();i != s.end(); i++) {
      tmp2 = getG(*i) + cost(u,*i);
      if (tmp2 < tmp) tmp = tmp2;
    }
    if (!close(getRHS(u),tmp)) setRHS(u,tmp);
  }

  if (!close(getG(u),getRHS(u))) insert(u);

}


void Dstar::insert(state u) {

  ds_oh::iterator cur;
  float csum;

  u    = calculateKey(u);
  cur  = openHash.find(u);
  csum = keyHashCode(u);
  // return if cell is already in list. TODO: this should be
  // uncommented except it introduces a bug, I suspect that there is a
  // bug somewhere else and having duplicates in the openList queue
  // hides the problem...
  //if ((cur != openHash.end()) && (close(csum,cur->second))) return;

  openHash[u] = csum;
  openList.push(u);
}


void Dstar::remove(state u) {

  ds_oh::iterator cur = openHash.find(u);
  if (cur == openHash.end()) return;
  openHash.erase(cur);
}

double Dstar::trueDist(state a, state b) {

  float x = a.x-b.x;
  float y = a.y-b.y;
  return sqrt(x*x + y*y);

}

double Dstar::heuristic(state a, state b)
{
  double d = eightCondist(a,b)*C1;
  //cout << "heuristic(state a, state b) = " << d << endl;
  return d;
}

state Dstar::calculateKey(state u)
{
  double val = fmin(getRHS(u),getG(u));

  u.k.first  = val + heuristic(u,s_start) + k_m;
  u.k.second = val;

  return u;

}

double Dstar::cost(state a, state b) {

  int xd = abs(a.x-b.x);
  int yd = abs(a.y-b.y);
  double scale = 1;

  if (xd+yd>1) scale = M_SQRT2;

  if (cellHash.count(a) == 0) return scale*C1;
  return scale*cellHash[a].cost;

}

void Dstar::updateCell(int x, int y, double val) {

   state u;

  u.x = x;
  u.y = y;

  if ((u == s_start) || (u == s_goal)) return;

  makeNewCell(u);
  cellHash[u].cost = val;

  updateVertex(u);
}


void Dstar::getSucc(state u,list<state> &s) {

  s.clear();
  u.k.first  = -1;
  u.k.second = -1;

  if (occupied(u)) return;

  u.x += 1;
  s.push_front(u);
  u.y += 1;
  s.push_front(u);
  u.x -= 1;
  s.push_front(u);
  u.x -= 1;
  s.push_front(u);
  u.y -= 1;
  s.push_front(u);
  u.y -= 1;
  s.push_front(u);
  u.x += 1;
  s.push_front(u);
  u.x += 1;
  s.push_front(u);

}


void Dstar::getPred(state u,list<state> &s) {

  s.clear();
  u.k.first  = -1;
  u.k.second = -1;

  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  u.y += 1;
  if (!occupied(u)) s.push_front(u);
  u.x -= 1;
  if (!occupied(u)) s.push_front(u);
  u.x -= 1;
  if (!occupied(u)) s.push_front(u);
  u.y -= 1;
  if (!occupied(u)) s.push_front(u);
  u.y -= 1;
  if (!occupied(u)) s.push_front(u);
  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  u.x += 1;
  if (!occupied(u)) s.push_front(u);

}

void Dstar::updateStart(int x, int y) {

  s_start.x = x;
  s_start.y = y;

  k_m += heuristic(s_last,s_start);

  s_start = calculateKey(s_start);
  s_last  = s_start;

}


void Dstar::updateGoal(int x, int y) {

  list< pair<ipoint2, double> > toAdd;
  pair<ipoint2, double> tp;

  ds_ch::iterator i;
  list< pair<ipoint2, double> >::iterator kk;

  for(i=cellHash.begin(); i!=cellHash.end(); i++) {
    if (!close(i->second.cost, C1)) {
      tp.first.x = i->first.x;
      tp.first.y = i->first.y;
      tp.second = i->second.cost;
      toAdd.push_back(tp);
    }
  }

  cellHash.clear();
  openHash.clear();

  while(!openList.empty())
    openList.pop();

  k_m = 0;

  s_goal.x  = x;
  s_goal.y  = y;

  cellInfo tmp;
  tmp.g = tmp.rhs =  0;
  tmp.cost = C1;

  cellHash[s_goal] = tmp;

  tmp.g = tmp.rhs = heuristic(s_start,s_goal);
  tmp.cost = C1;
  cellHash[s_start] = tmp;
  s_start = calculateKey(s_start);

  s_last = s_start;

  for (kk=toAdd.begin(); kk != toAdd.end(); kk++) {
    updateCell(kk->first.x, kk->first.y, kk->second);
  }


}


bool Dstar::replan() {

  path.clear();

  int res = computeShortestPath();
  cout << "res = " << res << endl;
  //printf("res: %d ols: %d ohs: %d tk: [%f %f] sk: [%f %f] sgr: (%f,%f)\n",res,openList.size(),openHash.size(),openList.top().k.first,openList.top().k.second, s_start.k.first, s_start.k.second,getRHS(s_start),getG(s_start));
  if (res < 0) {
   // fprintf(stderr, "NO PATH TO GOAL\n");
    return false;
  }
  list<state> n;
  list<state>::iterator i;

  state cur = s_start;

    //fprintf(stderr, "NO PATH TO GOAL\n");
  if (isinf(getG(s_start))) {
    return false;
  }

  while(cur != s_goal) {

    path.push_back(cur);
    getSucc(cur, n);

    if (n.empty()) {
      //fprintf(stderr, "NO PATH TO GOAL\n");
      return false;
    }

    double cmin = INFINITY;
    double tmin;
    state smin;

    for (i=n.begin(); i!=n.end(); i++)
    {
      //if (occupied(*i)) continue;
      double val  = cost(cur,*i);
      double val2 = trueDist(*i,s_goal) + trueDist(s_start,*i); // (Euclidean) cost to goal + cost to pred
      val += getG(*i);

      if (close(val,cmin))
      {
        if (tmin > val2)
        {
          tmin = val2;
          cmin = val;
          smin = *i;
        }
      }
      else if (val < cmin)
      {
        tmin = val2;
        cmin = val;
        smin = *i;
      }
    }
    n.clear();
    cur = smin;
  }
  path.push_back(s_goal);
  return true;
}
