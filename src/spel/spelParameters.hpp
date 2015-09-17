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
#ifdef DEBUG
    static SPEL_PARAMETER DEBUG_LEVEL(void)
    {
      return SPEL_SET_PARAMETER("debugLevel", 5.0f);
    };
#else
    static SPEL_PARAMETER DEBUG_LEVEL(void)
    {
      return SPEL_SET_PARAMETER("debugLevel", 1.0f);
    };
#endif  // DEBUG
    static SPEL_PARAMETER MAX_FRAME_HEIGHT(void)
    {
      return SPEL_SET_PARAMETER("maxFrameHeight", 720.0f);
    };
  };

  class COMMON_DETECTOR_PARAMETERS
  {
  public:
    static SPEL_PARAMETER USE_CH_DETECTOR(void)
    {
      return SPEL_SET_PARAMETER("useCSdet", 1.0f);
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
  };

  class COMMON_SURF_DETECTOR_PARAMETERS
  {
  public:
  };

  class COMMON_SOLVER_PARAMETERS
  {
  public:
  };

  class COMMON_NSKP_SOLVER_PARAMETERS
  {
  public:
  };

  class COMMON_TLPS_SOLVER_PARAMETERS
  {
  public:
  };
}
#endif // _SPELPARAMETERS_HPP_
