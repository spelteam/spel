// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com
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

  uint32_t Solver::findFrameIndexById(const int id, const std::vector<Frame*> &frames) const
  {
    auto idx = 0U;
    for (const auto frame : frames)
      if (frame->getID() != id)
        ++idx;
      else
        return idx;

    std::stringstream ss;
    ss << "There is no frame with id: " << id;
    throw std::logic_error(ss.str());
  }

  void Solver::emplaceDefaultParameters(std::map<std::string, float>& params)
    const 
  {
    spelHelper::mergeParameters(params,
      COMMON_SPEL_PARAMETERS::getParameters());
    spelHelper::mergeParameters(params,
      COMMON_DETECTOR_PARAMETERS::getParameters());
    spelHelper::mergeParameters(params,
      DETECTOR_DETECT_PARAMETERS::getParameters());
    spelHelper::mergeParameters(
      params, COMMON_SOLVER_PARAMETERS::getParameters());
  }

}
