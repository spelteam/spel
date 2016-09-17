#include "solver.hpp"

namespace SPEL
{
  Solver::Solver(void) noexcept
  {
    m_id = -1;
    m_name = "BaseClass";
  }

  Solver::~Solver(void) noexcept
  {
  }

  std::string Solver::getName(void) const noexcept
  {
    return m_name;
  }

  int Solver::getId(void) const noexcept
  {
    return m_id;
  }

}
