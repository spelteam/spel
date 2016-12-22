#include "predef.hpp"

#include "spelHelper.hpp"
#include "spelParameters.hpp"
#include <GUISolveParameters.h>

using namespace SPEL;

namespace posegui 
{
  void emplaceAdditionalParameters(std::map<std::string, float> &parameters)
  {
    //Ungrouped parameters
    auto parameter = std::pair<std::string, float>("temporalWindowSize", 0.0f);
    parameters.emplace(parameter);

    parameter = std::pair<std::string, float>("maxPartCandidates", 2000.0f);
    parameters.emplace(parameter);

    parameter = std::pair<std::string, float>("baseRotationRange", 40.0f);
    parameters.emplace(parameter);

    /*parameter = std::pair<std::string, float>("baseRotationStep", parameter.second / 4.0f);
    parameters.emplace(parameter);

    parameter = std::pair<std::string, float>("baseSearchRadius", 30.0f);  // (framre_height/30.0f)
    parameters.emplace(parameter);

    parameter = std::pair<std::string, float>("baseSearchStep", 3.0f);  // (framre_height/30.0f)/10.0f */
    parameters.emplace(parameter);

    /*parameter = std::pair<std::string, float>("partShiftCoeff", 1.5f);
    parameters.emplace(parameter);*/

    parameter = std::pair<std::string, float>("partRotationCoeff", 1.5f);
    parameters.emplace(parameter);

    parameter = std::pair<std::string, float>("scoreIndex", 0.0f);
    parameters.emplace(parameter);

    parameter = std::pair<std::string, float>("imageCoeff", 1.0f);
    parameters.emplace(parameter);

    parameter = std::pair<std::string, float>("jointCoeff", 0.5f);
    parameters.emplace(parameter);

    parameter = std::pair<std::string, float>("jointLeeway", 0.05f);
    parameters.emplace(parameter);

    /*parameter = std::pair<std::string, float>("tempCoeff", 0.1f);
    parameters.emplace(parameter);*/

    /*parameter = std::pair<std::string, float>("tlpsLockframeThreshold", 0.52f);
    parameters.emplace(parameter);*/

    /*parameter = std::pair<std::string, float>("partDepthRotationCoeff", 1.0f);
    parameters.emplace(parameter);*/

    /*parameter = std::pair<std::string, float>("withTLPS", 0.0f);
    parameters.emplace(parameter);*/

    /*parameter = std::pair<std::string, float>("nskpLockframeThreshold", 0.0f);
    parameters.emplace(parameter);*/
  }
  
  void  emplaceDefaultParameters(std::map<std::string, float> &parameters)
  {
    //"Global"
    spelHelper::mergeParameters(parameters,
      COMMON_SPEL_PARAMETERS::getParameters());
    spelHelper::mergeParameters(parameters,
      COMMON_DETECTOR_PARAMETERS::getParameters());
    spelHelper::mergeParameters(parameters,
      DETECTOR_DETECT_PARAMETERS::getParameters());
    spelHelper::mergeParameters(parameters,
        COMMON_SOLVER_PARAMETERS::getParameters());
    emplaceAdditionalParameters(parameters);

    //"NSKP"
    spelHelper::mergeParameters(parameters,
        COMMON_NSKP_SOLVER_PARAMETERS::getParameters());

    //"TLPS"
    spelHelper::mergeParameters(parameters,
        COMMON_TLPS_SOLVER_PARAMETERS::getParameters());
    
    //"CH"
    spelHelper::mergeParameters(parameters,
        COMMON_CH_DETECTOR_PARAMETERS::getParameters());

    //"HOG"
    spelHelper::mergeParameters(parameters,
        COMMON_HOG_DETECTOR_PARAMETERS::getParameters());

    //"SURF"
    spelHelper::mergeParameters(parameters,
        COMMON_SURF_DETECTOR_PARAMETERS::getParameters());
  }

  void  emplaceDefaultGroupedParameters(std::map<std::string, std::map<std::string, float>> &GroupedParameters)
  {
    GroupedParameters.emplace("Global", std::map<std::string, float>()); //std::pair<std::string, std::map<std::string, float>>
    spelHelper::mergeParameters(GroupedParameters["Global"],
      COMMON_SPEL_PARAMETERS::getParameters());
    spelHelper::mergeParameters(GroupedParameters["Global"],
      COMMON_DETECTOR_PARAMETERS::getParameters());
    spelHelper::mergeParameters(GroupedParameters["Global"],
      DETECTOR_DETECT_PARAMETERS::getParameters());
    spelHelper::mergeParameters(GroupedParameters["Global"],
        COMMON_SOLVER_PARAMETERS::getParameters());
    emplaceAdditionalParameters(GroupedParameters["Global"]);

    GroupedParameters.emplace("NSKP", std::map<std::string, float>());
    spelHelper::mergeParameters(GroupedParameters["NSKP"],
        COMMON_NSKP_SOLVER_PARAMETERS::getParameters());

    GroupedParameters.emplace("TLPS", std::map<std::string, float>());
    spelHelper::mergeParameters(GroupedParameters["TLPS"],
        COMMON_TLPS_SOLVER_PARAMETERS::getParameters());
    
    GroupedParameters.emplace("CH", std::map<std::string, float>());
    spelHelper::mergeParameters(GroupedParameters["CH"],
        COMMON_CH_DETECTOR_PARAMETERS::getParameters());

    GroupedParameters.emplace("HOG", std::map<std::string, float>());
    spelHelper::mergeParameters(GroupedParameters["HOG"],
        COMMON_HOG_DETECTOR_PARAMETERS::getParameters());

    GroupedParameters.emplace("SURF", std::map<std::string, float>());
    spelHelper::mergeParameters(GroupedParameters["SURF"],
        COMMON_SURF_DETECTOR_PARAMETERS::getParameters());
  }

  /*
  std::map<std::string, std::map<std::string, float>> DefaultParameters(std::vector<std::string> groupsNames)
  {
    std::map<std::string, std::map<std::string, float>> GroupedParameters;
    std::string groupName;
    //std::map<std::string, float> temp;
    for (unsigned int i = 0; i < groupsNames.size(); i++)
    {
      groupName = groupsNames[i];
      GroupedParameters.emplace(std::pair<std::string, std::map<std::string, float>>(groupName, std::map<std::string, float>()));
    }
    std::pair<std::string, float> parameter;

    // "Global" parameters
    groupName = "Global"; // groupsNames[0]; //"Global"
    if (GroupedParameters.find(groupName) != GroupedParameters.end())
    {
    parameter = std::pair<std::string, float>("debugLevel", 1);
    GroupedParameters[groupName].emplace(parameter); //"debugLevel"

    parameter = COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT();
    GroupedParameters[groupName].emplace(parameter); //"maxFrameHeight"

    parameter = COMMON_DETECTOR_PARAMETERS::USE_CH_DETECTOR(); //"useCSdet"
    GroupedParameters[groupName].emplace(parameter);

    parameter = COMMON_DETECTOR_PARAMETERS::USE_HOG_DETECTOR(); //"useHoGdet"
    GroupedParameters[groupName].emplace(parameter);

    parameter = COMMON_DETECTOR_PARAMETERS::USE_SURF_DETECTOR(); //"useSURFdet"
    GroupedParameters[groupName].emplace(parameter);

    parameter = DETECTOR_DETECT_PARAMETERS::SEARCH_DISTANCE_COEFFICIENT(); //"searchDistCoeff"
    GroupedParameters[groupName].emplace(parameter);

    parameter = DETECTOR_DETECT_PARAMETERS::SEARCH_DISTANCE_MULT_COEFFICIENT(); //"searchDistCoeffMult"
    GroupedParameters[groupName].emplace(parameter);

    parameter = DETECTOR_DETECT_PARAMETERS::SEARCH_STEP_COEFFICIENT(); //"searchStepCoeff"
    GroupedParameters[groupName].emplace(parameter);

    parameter = DETECTOR_DETECT_PARAMETERS::MIN_THETA(); //"minTheta"
    GroupedParameters[groupName].emplace(parameter);

    parameter = DETECTOR_DETECT_PARAMETERS::MAX_THETA(); //"maxTheta"
    GroupedParameters[groupName].emplace(parameter);

    parameter = DETECTOR_DETECT_PARAMETERS::STEP_THETA(); //"stepTheta"
    GroupedParameters[groupName].emplace(parameter);

    parameter = DETECTOR_DETECT_PARAMETERS::UNIQUE_LOCATION_CANDIDATES_COEFFICIENT(); //"uniqueLocationCandidates"
    GroupedParameters[groupName].emplace(parameter);

    parameter = DETECTOR_DETECT_PARAMETERS::UNIQUE_ANGLE_CANDIDATES_COEFFICIENT(); //"uniqueAngleCandidates"
    GroupedParameters[groupName].emplace(parameter);

    parameter = DETECTOR_DETECT_PARAMETERS::SCALE_COEFFICIENT(); //"scaleParam"
    GroupedParameters[groupName].emplace(parameter);

    parameter = DETECTOR_DETECT_PARAMETERS::ROTATION_THRESHOLD(); //"rotationThreshold"
    GroupedParameters[groupName].emplace(parameter);

    parameter = DETECTOR_DETECT_PARAMETERS::IS_WEAK_THRESHOLD(); //"isWeakThreshold"
    GroupedParameters[groupName].emplace(parameter);
    }

    //Ungrouped parameters
    groupName = "Global"; //groupsNames[0]; //"Global"
    if (GroupedParameters.find(groupName) != GroupedParameters.end())
    {
      emplaceAdditionalParameters(GroupedParameters[groupName]);
    }

    //HOG dtetector parameters
    groupName = "HOG"; // groupsNames[4]; //"HOG"
    if (GroupedParameters.find(groupName) != GroupedParameters.end())
    {
      parameter = COMMON_HOG_DETECTOR_PARAMETERS::USE_GRAY_IMAGES(); //"grayImages"
      GroupedParameters[groupName].emplace(parameter);
    }

    //SURF dtetector parameters
    groupName = "SURF";// groupsNames[5]; //"SURF"
    if (GroupedParameters.find(groupName) != GroupedParameters.end())
    {
      parameter = COMMON_SURF_DETECTOR_PARAMETERS::MIN_HESSIAN(); //"minHessian"
      GroupedParameters[groupName].emplace(parameter);

      parameter = COMMON_SURF_DETECTOR_PARAMETERS::KNN_MATCH_COEFFICIENT(); //"knnMathCoeff"
      GroupedParameters[groupName].emplace(parameter);
    }

    //NSKPSolver parameters
    groupName = "NSKP"; // groupsNames[1]; //"NSKP"
    if (GroupedParameters.find(groupName) != GroupedParameters.end())
    {
      parameter = COMMON_SURF_DETECTOR_PARAMETERS::KNN_MATCH_COEFFICIENT(); //"nskpLockframeThreshold"
      GroupedParameters[groupName].emplace(parameter);
    }
    return GroupedParameters;
  }*/
}