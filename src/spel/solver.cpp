#include "solver.hpp"
#include "spelParameters.hpp"

namespace SPEL
{
  Solver::Solver(void) 
  {
    m_id = -1;
    m_name = "BaseClass";
  }

  Solver::~Solver(void) 
  {
  }

  std::string Solver::getName(void) const 
  {
    return m_name;
  }

  int Solver::getId(void) const 
  {
    return m_id;
  }

  uint32_t Solver::findFrameIndexById(int id, std::vector<Frame*> frames) const
  {
    for (auto i = 0U; i < frames.size(); ++i)
    {
      if (frames.at(i)->getID() == id)
        return i;
    }
    std::stringstream ss;
    ss << "There is no frame with id: " << id;
    throw std::logic_error(ss.str());
  }

  void Solver::emplaceDefaultParameters(std::map<std::string, float>& params)
    const 
  {
    spelHelper::mergeParameters(
      params, COMMON_SOLVER_PARAMETERS::getParameters());
  }

}
