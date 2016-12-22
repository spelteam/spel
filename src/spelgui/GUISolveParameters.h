#include "predef.hpp"

#include "spelParameters.hpp"
#include <map>
#include <vector>
#include <string>

namespace posegui 
{
  void emplaceAdditionalParameters(std::map<std::string, float> &parameters);
  void  emplaceDefaultParameters(std::map<std::string, float> &parameters);
  void emplaceDefaultGroupedParameters(std::map<std::string, std::map<std::string, float>> &GroupedParameters);
  //std::map<std::string, std::map<std::string, float>> DefaultParameters(std::vector<std::string> groupsNames);
}

