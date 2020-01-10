/*
 * Dstar.h
 *
 *  Created on: Apr 18, 2016
 *      Author: kardee13
 */

#ifndef DSTAR_H_
#define DSTAR_H_

#include<iostream>
#include<stack>
#include<queue>
#include<list>
#include<ext/hash_map>
#include<cmath>
#include<cstdlib>

using namespace std;
using namespace __gnu_cxx;

class state
{
   public:
      int x;
      int y;
      pair<double,double> k;   //std::pair

      bool operator == (const state &s2) const
      {
         return ((x == s2.x) && (y==s2.y));
      }

      bool operator != (const state &s2) const
      {
         return ((x != s2.x) && (y!=s2.y));
      }

      bool operator > (const state &s2) const
      {
             if (k.first < s2.k.first) return true;
             else if (k.first > s2.k.first) return false;
             return k.second < s2.k.second + 0.00001;
      }

      bool operator < (const state &s2) const
      {
          if (k.first + 0.000001 < s2.k.first) return true;
          else if (k.first - 0.000001 > s2.k.first) return false;
         return k.second < s2.k.second;
      }
};

struct ipoint2
{
  int x,y;
};

struct cellInfo
{

  double g;
  double rhs;
  double cost;

};

class state_hash
{
 public:
  size_t operator()(const state &s) const
  {
    return s.x + 34245*s.y;
  }
};

typedef priority_queue<state, vector<state>, greater<state> > ds_pq;    /*prioritize the states from lower to higher*/
typedef hash_map<state,cellInfo, state_hash, equal_to<state> > ds_ch;
typedef hash_map<state, float, state_hash, equal_to<state> > ds_oh;

class Dstar {
public:
    Dstar();
    void   init(int sX, int sY, int gX, int gY);
    void   updateCell(int x, int y, double val);
    void   updateStart(int x, int y);
    void   updateGoal(int x, int y);
    bool   replan();
    list<state> getPath();
protected:
private:

    list<state> path; //list of state type

    double C1;     // cost of the cell
    double k_m;
    state s_start, s_goal, s_last;
    int maxSteps;  // maximum steps till we give up

    ds_pq openList;
    ds_ch cellHash;
    ds_oh openHash;

    bool   close(double x, double y);
    void   makeNewCell(state u);
    double getG(state u);
    double getRHS(state u);
    void   setG(state u, double g);
    void   setRHS(state u, double rhs);
    double eightCondist(state a, state b);
    int    computeShortestPath();
    void   updateVertex(state u);
    void   insert(state u);
    void   remove(state u);
    double trueDist(state a, state b);
    double heuristic(state a, state b);
    state  calculateKey(state u);
    void   getSucc(state u, list<state> &s);
    void   getPred(state u, list<state> &s);
    double cost(state a, state b);
    bool   occupied(state u);
    bool   isValid(state u);
    float  keyHashCode(state u);
};

#endif /* DSTAR_H_ */
