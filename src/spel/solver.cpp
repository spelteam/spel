#include "solver.hpp"

namespace SPEL
{
  Solver::Solver(void) noexcept
  {
    id = -1;
    name = "BaseClass";
  }

  Solver::~Solver(void) noexcept
  {
  }

  std::string Solver::getName(void) const noexcept
  {
    return name;
  }

  int Solver::getId(void) const noexcept
  {
    return id;
  }

}
