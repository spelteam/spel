#include "solver.hpp"

namespace SPEL
{

  Solver::Solver(void)
  {
    id = -1;
    name = "BaseClass";
  }

  Solver::~Solver(void)
  {

  }

  std::string Solver::getName() //get the solver name. Every class inheriting solver has its own Name
  {
    return name;
  }

  int Solver::getId()
  {
    return id;
  }

  std::vector<Solvlet> Solver::solve(const Sequence& v)
  {
    std::map<std::string, float> params;
    return solve(v, params);
  }

  std::vector<Solvlet> Solver::solve(const Sequence& v, std::map<std::string, float> params)
  {
    std::vector<Solvlet> empty;
    return empty;
  }

}
