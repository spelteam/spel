#ifndef _SOLVER_HPP_
#define _SOLVER_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <string>
#include <vector>

#include "solvlet.hpp"
#include "sequence.hpp"

namespace SPEL
{
  class Solver
  {
  public:
    Solver(void) noexcept;
    virtual ~Solver(void) noexcept;
    virtual std::vector<Solvlet> solve(Sequence& v) = 0;
    virtual std::vector<Solvlet> solve(Sequence& v, std::map<std::string, float> params) = 0;
    ///get the solver name. Every class inheriting solver has its own Name
    std::string getName(void) const noexcept;
    ///get the solver Id. Every class inheriting solver has its own ID
    int getId(void) const noexcept;
  protected:
    int id;
    std::string name;
  };

}

#endif  // _SOLVER_HPP_