#ifndef _SOLVER_HPP_
#define _SOLVER_HPP_

#include <string>
#include <vector>
#include "solvlet.hpp"
#include "frame.hpp"

using namespace std;

class Solver
{
  public:
  	Solver(void);
  	virtual ~Solver(void);
  	virtual vector<Solvlet> solve(const vector<Frame*>& v);
  	virtual vector<Solvlet> solve(const vector<Frame*>& v, map<string, float> params);
  	string getName(void); //get the solver name. Every class inheriting solver has its own Name
  	int getId(void); //get the solver Id. Every class inheriting solver has its own ID
  protected:
  	int id;
  	string name;
};

#endif  // _SOLVER_HPP_

