#ifndef _SPELPARAMETERS_HPP_
#define _SPELPARAMETERS_HPP_
#include <utility>
#include <string>

namespace SPEL
{
  typedef std::pair<std::string, float> SPEL_PARAMETER;

  inline SPEL_PARAMETER SPEL_SET_PARAMETER(const std::string &name, const float &value)
  {
    return make_pair(name, value);
  }

  class COMMON_SPEL_PARAMETERS
  {
  public:
    static SPEL_PARAMETER MAX_FRAME_HEIGHT(void)
    {
      return SPEL_SET_PARAMETER("maxFrameHeight", 720.0f);
    }
//    static SPEL_PARAMETER DEBUG_LEVEL(void)
//    {
//        return SPEL_SET_PARAMETER("debugLevel", 1);
//    }
  };

  class COMMON_DETECTOR_PARAMETERS
  {
  public:
    static SPEL_PARAMETER USE_CH_DETECTOR(void)
    {
      return SPEL_SET_PARAMETER("useCSdet", 1.0f);
    }

    static SPEL_PARAMETER USE_HOG_DETECTOR(void)
    {
      return SPEL_SET_PARAMETER("useHoGdet", 1.0f);
    }

    static SPEL_PARAMETER USE_SURF_DETECTOR(void)
    {
      return SPEL_SET_PARAMETER("useSURFdet", 1.0f);
    }
  };

  class DETECTOR_DETECT_PARAMETERS
  {
  public:
    static SPEL_PARAMETER SEARCH_DISTANCE_COEFFICIENT(void)
    {
      return SPEL_SET_PARAMETER("searchDistCoeff", 0.5f);
    }

    static SPEL_PARAMETER SEARCH_DISTANCE_MULT_COEFFICIENT(void)
    {
      return SPEL_SET_PARAMETER("searchDistCoeffMult", 1.25f);
    }

    static SPEL_PARAMETER SEARCH_STEP_COEFFICIENT(void)
    {
      return SPEL_SET_PARAMETER("searchStepCoeff", 0.2f);
    }

    // border for search
    static SPEL_PARAMETER MIN_THETA(void)
    {
      return SPEL_SET_PARAMETER("minTheta", 90.0f);
    }

    // border for search
    static SPEL_PARAMETER MAX_THETA(void)
    {
      return SPEL_SET_PARAMETER("maxTheta", 100.0f);
    }

    // angular step of search
    static SPEL_PARAMETER STEP_THETA(void)
    {
      return SPEL_SET_PARAMETER("stepTheta", 10.0f);
    }

    // limiting the choice of the solutions number for each bodypart
    static SPEL_PARAMETER UNIQUE_LOCATION_CANDIDATES_COEFFICIENT(void)
    {
      return SPEL_SET_PARAMETER("uniqueLocationCandidates", 0.1f);
    }

    // limiting the choice of the solutions number for each bodypart
    static SPEL_PARAMETER UNIQUE_ANGLE_CANDIDATES_COEFFICIENT(void)
    {
      return SPEL_SET_PARAMETER("uniqueAngleCandidates", 0.1f);
    }

    // scaling coefficient
    static SPEL_PARAMETER SCALE_COEFFICIENT(void)
    {
      return SPEL_SET_PARAMETER("scaleParam", 1.0f);
    }

    static SPEL_PARAMETER ROTATION_THRESHOLD(void)
    {
      return SPEL_SET_PARAMETER("rotationThreshold", 0.025f);
    }

    static SPEL_PARAMETER IS_WEAK_THRESHOLD(void)
    {
      return SPEL_SET_PARAMETER("isWeakThreshold", 0.1f);
    }
  };

  class COMMON_CH_DETECTOR_PARAMETERS
  {
  public:
  };

  class COMMON_HOG_DETECTOR_PARAMETERS
  {
  public:
    static SPEL_PARAMETER USE_GRAY_IMAGES(void)
    {
      return SPEL_SET_PARAMETER("grayImages", 0.0f);
    }
  };

  class COMMON_SURF_DETECTOR_PARAMETERS
  {
  public:
    static SPEL_PARAMETER MIN_HESSIAN(void)
    {
      return SPEL_SET_PARAMETER("minHessian", 500.0f);
    }

    static SPEL_PARAMETER KNN_MATCH_COEFFICIENT(void)
    {
      return SPEL_SET_PARAMETER("knnMathCoeff", 0.8f);
    }
  };

  class COMMON_SOLVER_PARAMETERS
  {
  public:
      //      params.emplace("useCSdet", 1.0); //determine if ColHist detector is used and with what coefficient
      //      params.emplace("useHoGdet", 0.0); //determine if HoG descriptor is used and with what coefficient
      //      params.emplace("useSURFdet", 0.0); //determine whether SURF detector is used and with what coefficient
      //      params.emplace("maxPartCandidates", 1.0); //set the max number of part candidates to allow into the solver
      //      params.emplace("baseRotationRange", 40); //search angle range of +/- 60 degrees
      //      float baseRotationRange = params.at("baseRotationRange");
      //      params.emplace("baseRotationStep", baseRotationRange / 4.0); //search with angle step of 10 degrees
      //      params.emplace("baseSearchRadius", image.rows / 30.0); //search a radius of 100 pixels
      //      int baseSearchRadius = params.at("baseSearchRadius");
      //      params.emplace("baseSearchStep", baseSearchRadius / 10.0); //search in a grid every 10 pixels
      //      params.emplace("imageCoeff", 1.0); //set solver detector infromation sensitivity
      //      params.emplace("jointCoeff", 0.5); //set solver body part connectivity sensitivity
      //      params.emplace("jointLeeway", 0.05); //set solver lenience for body part disconnectedness, as a percentage of part length

  };

  class COMMON_NSKP_SOLVER_PARAMETERS
  {
  public:

//      //the params vector should contain all necessary parameters, if a parameter is not present, default values should be used
//      params.emplace("debugLevel", 1); //set up the lockframe accept threshold by mask coverage
//      params.emplace("propagateFromLockframes", 1); //don't propagate from lockframes, only from keyframes

//      //detector enablers
//      params.emplace("useCSdet", 1.0); //determine if ColHist detector is used and with what coefficient
//      params.emplace("useHoGdet", 0.0); //determine if HoG descriptor is used and with what coefficient
//      params.emplace("useSURFdet", 0.0); //determine whether SURF detector is used and with what coefficient
//      params.emplace("maxPartCandidates", 40); //set the max number of part candidates to allow into the solver

//      //detector search parameters

//      params.emplace("partDepthRotationCoeff", 1.2); // 20% increase at each depth level
//      params.emplace("anchorBindDistance", 0); //restrict search regions if within bind distance of existing keyframe or lockframe (like a temporal link
//      params.emplace("anchorBindCoeff", 0.0); //multiplier for narrowing the search range if close to an anchor (lockframe/keyframe)
//      params.emplace("bindToLockframes", 0); //should binds be also used on lockframes?

//      //detector search parameters
//      params.emplace("baseRotationRange", 40); //search angle range of +/- 40 degrees
//      float baseRotationRange = params.at("baseRotationRange");
//      params.emplace("baseRotationStep", baseRotationRange / 4.0); //search with angle step of 10 degrees, this a per-part range and overrides globals
//      params.emplace("stepTheta", baseRotationRange / 4.0); //search in a grid every 10 pixels

//      params.emplace("baseSearchRadius", image.rows / 30.0); //search a radius of 100 pixels
//      int baseSearchRadius = params.at("baseSearchRadius");
//      params.emplace("baseSearchStep", baseSearchRadius / 10.0); //do 9-10 steps in each direction
//      //solver sensitivity parameters
//      params.emplace("imageCoeff", 1.0); //set solver detector infromation sensitivity
//      params.emplace("jointCoeff", 0.5); //set solver body part connectivity sensitivitymaxPartCandidates
//      params.emplace("jointLeeway", 0.05); //set solver lenience for body part disconnectedness, as a percentage of part length
//      params.emplace("priorCoeff", 0.0); //set solver distance to prior sensitivity

//      //solver eval parameters
//      params.emplace("nskpLockframeThreshold", 0.52); //set up the lockframe accept threshold by mask coverage

      static SPEL_PARAMETER KNN_MATCH_COEFFICIENT(void)
      {
            return SPEL_SET_PARAMETER("nskpLockframeThreshold", 0.52);
      }
  };

  class COMMON_TLPS_SOLVER_PARAMETERS
  {
  public:
//      //params.emplace("debugLevel", 1); //set up the lockframe accept threshold by mask coverage
//      params.emplace("temporalWindowSize", 0); //0 for unlimited window size
//      params.emplace("partShiftCoeff", 1.5); //search radius multiplier of distance between part in current and prev frames
//      params.emplace("partRotationCoeff", 1.5); //rotation radius multiplier of distance between part in current and prev frames
//      params.emplace("tempCoeff", 0.1); //set the temporal link coefficient
//      params.emplace("tlpsLockframeThreshold", 0.0); //set up the lockframe accept threshold by mask coverage
  };
}
#endif // _SPELPARAMETERS_HPP_
