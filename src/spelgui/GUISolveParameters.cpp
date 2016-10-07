#include "predef.hpp"

#include "spelParameters.hpp"
#include <GUISolveParameters.h>

using namespace SPEL;

namespace posegui 
{
  std::map<std::string, std::map<std::string, float>> setDefaultParameters(std::vector<std::string> groupsNames)
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
    groupName = groupsNames[0]; //"Global"
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

    //UNGROUPED
    groupName = groupsNames[0]; //"Global"
    if (GroupedParameters.find(groupName) != GroupedParameters.end())
    {
    parameter = std::pair<std::string, float>("temporalWindowSize", 0.0f);
    GroupedParameters[groupName].emplace(parameter);

    parameter = std::pair<std::string, float>("maxPartCandidates", 2000.0f);
    GroupedParameters[groupName].emplace(parameter);

    parameter = std::pair<std::string, float>("baseRotationRange", 40.0f);
    GroupedParameters[groupName].emplace(parameter);

    parameter = std::pair<std::string, float>("baseRotationStep", parameter.second / 4.0f);
    GroupedParameters[groupName].emplace(parameter);

    parameter = std::pair<std::string, float>("baseSearchRadius", 30.0f);  // (framre_height/30.0f)
    GroupedParameters[groupName].emplace(parameter);

    parameter = std::pair<std::string, float>("baseSearchStep", 3.0f);  // (framre_height/30.0f)/10.0f
    GroupedParameters[groupName].emplace(parameter);

    parameter = std::pair<std::string, float>("partShiftCoeff", 1.5f);
    GroupedParameters[groupName].emplace(parameter);

    parameter = std::pair<std::string, float>("partRotationCoeff", 1.5f);
    GroupedParameters[groupName].emplace(parameter);

    parameter = std::pair<std::string, float>("scoreIndex", 0.0f);
    GroupedParameters[groupName].emplace(parameter);

    parameter = std::pair<std::string, float>("imageCoeff", 1.0f);
    GroupedParameters[groupName].emplace(parameter);

    parameter = std::pair<std::string, float>("jointCoeff", 0.5f);
    GroupedParameters[groupName].emplace(parameter);

    parameter = std::pair<std::string, float>("jointLeeway", 0.05f);
    GroupedParameters[groupName].emplace(parameter);

    parameter = std::pair<std::string, float>("tempCoeff", 0.1f);
    GroupedParameters[groupName].emplace(parameter);

    parameter = std::pair<std::string, float>("tlpsLockframeThreshold", 0.52f);
    GroupedParameters[groupName].emplace(parameter);

    parameter = std::pair<std::string, float>("tlpsLockframeThreshold", 0.52f);
    GroupedParameters[groupName].emplace(parameter);

    parameter = std::pair<std::string, float>("partDepthRotationCoeff", 1.0f);
    GroupedParameters[groupName].emplace(parameter);

    parameter = std::pair<std::string, float>("withTLPS", 0.0f);
    GroupedParameters[groupName].emplace(parameter);

    parameter = std::pair<std::string, float>("nskpLockframeThreshold", 0.0f);
    GroupedParameters[groupName].emplace(parameter);

    }

    //HOG dtetector parameters
    groupName = groupsNames[6]; //"HOG"
    if (GroupedParameters.find(groupName) != GroupedParameters.end())
    {
    parameter = COMMON_HOG_DETECTOR_PARAMETERS::USE_GRAY_IMAGES(); //"grayImages"
    GroupedParameters[groupName].emplace(parameter);
    }

    //SURF dtetector parameters
    groupName = "SURF";// groupsNames[7]; //"SURF"
    if (GroupedParameters.find(groupName) != GroupedParameters.end())
    {
    parameter = COMMON_SURF_DETECTOR_PARAMETERS::MIN_HESSIAN(); //"minHessian"
    GroupedParameters[groupName].emplace(parameter);

    parameter = COMMON_SURF_DETECTOR_PARAMETERS::KNN_MATCH_COEFFICIENT(); //"knnMathCoeff"
    GroupedParameters[groupName].emplace(parameter);
    }

    //NSKPSolver parameters
    groupName = groupsNames[3]; //"NSKP"
    if (GroupedParameters.find(groupName) != GroupedParameters.end())
    {
    parameter = COMMON_NSKP_SOLVER_PARAMETERS::KNN_MATCH_COEFFICIENT(); //"nskpLockframeThreshold"
    GroupedParameters[groupName].emplace(parameter);
    }
    return GroupedParameters;
  }
}