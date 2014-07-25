#ifndef _SOLVER_HPP_
#define _SOLVER_HPP_

#include <string>
#include <vector>
#include "solver.hpp"
#include "solution.hpp"
#include "frame.hpp"

using namespace std;

class Solver
{
  public:
  	Solver();
  	virtual ~Solver();
  	Solution solve(vector<Frame*>);
  	Solution solve(vector<Frame*>, vector<float> params);
  	string getName(); //get the solver name. Every class inheriting solver has its own Name
  	string getId(); //get the solver Id. Every class inheriting solver has is own ID
  private:
  	int id;
  	string name;
};

#endif  // _SOLVER_HPP_

