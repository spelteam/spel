#ifndef _SPELPARAMETERS_HPP_
#define _SPELPARAMETERS_HPP_
#include <utility>
#include <string>
#include <map>

namespace SPEL
{
  struct SPEL_PARAMETER : public std::pair<std::string, float>
  {
    typedef std::pair<std::string, float> T;
    SPEL_PARAMETER() : T() {}
    SPEL_PARAMETER(const SPEL_PARAMETER &param) : T(param) {}
    SPEL_PARAMETER(const std::string &name, const float value) : 
      T(name, value) {}

    std::string &name() { return first; }
    const std::string &name() const { return first; }

    float &value() { return second; }
    const float &value() const { return second; }
  };

  inline auto SPEL_SET_PARAMETER(const std::string &name,
    const float &value)
  {
    return SPEL_PARAMETER(name, value);
  }

  class COMMON_SPEL_PARAMETERS
  {
  public:
    static auto MAX_FRAME_HEIGHT(void)
    {
      return SPEL_SET_PARAMETER("maxFrameHeight", 720.0f);
    }

    static auto getParameters(void)
    {
      std::map<std::string, float> map;

      map.emplace(COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT());

      return map;
    }
  };

  class COMMON_DETECTOR_PARAMETERS
  {
  public:
    static auto USE_CH_DETECTOR(void)
    {
      return SPEL_SET_PARAMETER("useCSdet", 1.0f);
    }

    static auto USE_HOG_DETECTOR(void)
    {
      return SPEL_SET_PARAMETER("useHoGdet", 1.0f);
    }

    static auto USE_SURF_DETECTOR(void)
    {
      return SPEL_SET_PARAMETER("useSURFdet", 1.0f);
    }

    static auto getParameters(void)
    {
      std::map<std::string, float> map;

      map.emplace(COMMON_DETECTOR_PARAMETERS::USE_CH_DETECTOR());
      map.emplace(COMMON_DETECTOR_PARAMETERS::USE_HOG_DETECTOR());
      map.emplace(COMMON_DETECTOR_PARAMETERS::USE_SURF_DETECTOR());

      return map;
    }
  };

  class DETECTOR_DETECT_PARAMETERS
  {
  public:
    static auto SEARCH_DISTANCE_COEFFICIENT(void)
    {
      return SPEL_SET_PARAMETER("searchDistCoeff", 0.5f);
    }

    static auto SEARCH_DISTANCE_MULT_COEFFICIENT(void)
    {
      return SPEL_SET_PARAMETER("searchDistCoeffMult", 1.25f);
    }

    static auto SEARCH_STEP_COEFFICIENT(void)
    {
      return SPEL_SET_PARAMETER("searchStepCoeff", 0.2f);
    }

    // border for search
    static auto MIN_THETA(void)
    {
      return SPEL_SET_PARAMETER("minTheta", 90.0f);
    }

    // border for search
    static auto MAX_THETA(void)
    {
      return SPEL_SET_PARAMETER("maxTheta", 100.0f);
    }

    // angular step of search
    static auto STEP_THETA(void)
    {
      return SPEL_SET_PARAMETER("stepTheta", 10.0f);
    }

    // limiting the choice of the solutions number for each bodypart
    static auto UNIQUE_LOCATION_CANDIDATES_COEFFICIENT(void)
    {
      return SPEL_SET_PARAMETER("uniqueLocationCandidates", 0.1f);
    }

    // limiting the choice of the solutions number for each bodypart
    static auto UNIQUE_ANGLE_CANDIDATES_COEFFICIENT(void)
    {
      return SPEL_SET_PARAMETER("uniqueAngleCandidates", 0.1f);
    }

    // scaling coefficient
    static auto SCALE_COEFFICIENT(void)
    {
      return SPEL_SET_PARAMETER("scaleParam", 1.0f);
    }

    static auto ROTATION_THRESHOLD(void)
    {
      return SPEL_SET_PARAMETER("rotationThreshold", 0.025f);
    }

    static auto IS_WEAK_THRESHOLD(void)
    {
      return SPEL_SET_PARAMETER("isWeakThreshold", 0.1f);
    }

    static auto getParameters(void)
    {
      std::map<std::string, float> map;

      map.emplace(DETECTOR_DETECT_PARAMETERS::SEARCH_DISTANCE_COEFFICIENT());
      map.emplace(DETECTOR_DETECT_PARAMETERS::SEARCH_DISTANCE_MULT_COEFFICIENT());
      map.emplace(DETECTOR_DETECT_PARAMETERS::SEARCH_STEP_COEFFICIENT());
      map.emplace(DETECTOR_DETECT_PARAMETERS::MIN_THETA());
      map.emplace(DETECTOR_DETECT_PARAMETERS::MAX_THETA());
      map.emplace(DETECTOR_DETECT_PARAMETERS::STEP_THETA());
      map.emplace(DETECTOR_DETECT_PARAMETERS::UNIQUE_LOCATION_CANDIDATES_COEFFICIENT());
      map.emplace(DETECTOR_DETECT_PARAMETERS::UNIQUE_ANGLE_CANDIDATES_COEFFICIENT());
      map.emplace(DETECTOR_DETECT_PARAMETERS::SCALE_COEFFICIENT());
      map.emplace(DETECTOR_DETECT_PARAMETERS::ROTATION_THRESHOLD());
      map.emplace(DETECTOR_DETECT_PARAMETERS::IS_WEAK_THRESHOLD());

      return map;
    }
  };

  class COMMON_CH_DETECTOR_PARAMETERS
  {
  public:
    static auto getParameters(void)
    {
      std::map<std::string, float> map;

      return map;
    }
  };

  class COMMON_HOG_DETECTOR_PARAMETERS
  {
  public:
    static auto USE_GRAY_IMAGES(void)
    {
      return SPEL_SET_PARAMETER("grayImages", 0.0f);
    }

    static auto getParameters(void)
    {
      std::map<std::string, float> map;

      map.emplace(COMMON_HOG_DETECTOR_PARAMETERS::USE_GRAY_IMAGES());

      return map;
    }
  };

  class COMMON_SURF_DETECTOR_PARAMETERS
  {
  public:
    static auto MIN_HESSIAN(void)
    {
      return SPEL_SET_PARAMETER("minHessian", 500.0f);
    }

    static auto KNN_MATCH_COEFF(void)
    {
      return SPEL_SET_PARAMETER("knnMathCoeff", 0.8f);
    }

    static auto getParameters(void)
    {
      std::map<std::string, float> map;

      map.emplace(COMMON_SURF_DETECTOR_PARAMETERS::MIN_HESSIAN());
      map.emplace(COMMON_SURF_DETECTOR_PARAMETERS::KNN_MATCH_COEFF());

      return map;
    }
  };

  class COMMON_SOLVER_PARAMETERS
  {
  public:
    static auto MAX_PART_CANDIDATES(void)
    {
      return SPEL_SET_PARAMETER("maxPartCandidates", 40.0f);
    }

    static auto PART_DEPTH_ROTATION_COEFF(void)
    {
      return SPEL_SET_PARAMETER("partDepthRotationCoeff", 1.2f);
    }

    static auto BASE_ROTATION_RANGE(void)
    {
      return SPEL_SET_PARAMETER("baseRotationRange", 40.0f);
    }

    static auto BASE_ROTATION_STEP(void)
    {
      return SPEL_SET_PARAMETER("baseRotationStep", 
        COMMON_SOLVER_PARAMETERS::BASE_ROTATION_RANGE().second / 4.0f);
    }

    static auto BASE_SEARCH_RADIUS(void)
    {
      return SPEL_SET_PARAMETER("baseSearchRadius", 
        COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().second / 30.0f);
    }

    static auto BASE_SEARCH_STEP(void)
    {
      return SPEL_SET_PARAMETER("baseSearchStep", 
        COMMON_SOLVER_PARAMETERS::BASE_SEARCH_RADIUS().second / 10.0f);
    }

    static auto IMAGE_COEFF(void)
    {
      return SPEL_SET_PARAMETER("imageCoeff", 1.0f);
    }

    static auto JOINT_COEFF(void)
    {
      return SPEL_SET_PARAMETER("jointCoeff", 0.5f);
    }

    static auto JOINT_LEEWAY(void)
    {
      return SPEL_SET_PARAMETER("jointLeeway", 0.05f);
    }

    static auto PRIOR_COEFF(void)
    {
      return SPEL_SET_PARAMETER("priorCoeff", 0.0f);
    }

    static auto BAD_LABEL_THRESH(void)
    {
      return SPEL_SET_PARAMETER("badLabelThresh", 0.52f);
    }

    static auto getParameters(void)
    {
      std::map<std::string, float> map;

      map.emplace(COMMON_SOLVER_PARAMETERS::MAX_PART_CANDIDATES());
      map.emplace(COMMON_SOLVER_PARAMETERS::PART_DEPTH_ROTATION_COEFF());
      map.emplace(COMMON_SOLVER_PARAMETERS::BASE_ROTATION_RANGE());
      map.emplace(COMMON_SOLVER_PARAMETERS::BASE_ROTATION_STEP());
      map.emplace(COMMON_SOLVER_PARAMETERS::BASE_SEARCH_RADIUS());
      map.emplace(COMMON_SOLVER_PARAMETERS::BASE_SEARCH_STEP());
      map.emplace(COMMON_SOLVER_PARAMETERS::IMAGE_COEFF());
      map.emplace(COMMON_SOLVER_PARAMETERS::JOINT_COEFF());
      map.emplace(COMMON_SOLVER_PARAMETERS::JOINT_LEEWAY());
      map.emplace(COMMON_SOLVER_PARAMETERS::PRIOR_COEFF());
      map.emplace(COMMON_SOLVER_PARAMETERS::BAD_LABEL_THRESH());

      return map;
    }
  };

  class COMMON_NSKP_SOLVER_PARAMETERS
  {
  public:
    static auto ITERATIONS(void)
    {
      return SPEL_SET_PARAMETER("nskpIters", static_cast<float>(std::numeric_limits<uint32_t>::max()));
    }

    static auto USE_TLPS(void)
    {
      return SPEL_SET_PARAMETER("withTLPS", 0.0f);
    }

    static auto PROPAGATE_FROM_LOCKFRAMES(void)
    {
      return SPEL_SET_PARAMETER("propagateFromLockframes", 1.0f);
    }

    static auto ANCHOR_BIND_DISTANCE(void)
    {
      return SPEL_SET_PARAMETER("anchorBindDistance", 0.0f);
    }

    static auto ANCHOR_BIND_COEFF(void)
    {
      return SPEL_SET_PARAMETER("anchorBindCoeff", 0.0f);
    }

    static auto LOCKFRAME_THRESHOLD(void)
    {
      return SPEL_SET_PARAMETER("nskpLockframeThreshold", 0.52f);
    }

    static auto BIND_TO_LOCKFRAMES(void)
    {
      return SPEL_SET_PARAMETER("bindToLockframes", 0.0f);
    }

    static auto MIN_KEY_FRAME_DISTANCE(void)
    {
      return SPEL_SET_PARAMETER("minKeyframeDist", 1.0f);
    }

    static auto getParameters(void)
    {
      std::map<std::string, float> map;

      map.emplace(COMMON_NSKP_SOLVER_PARAMETERS::ITERATIONS());
      map.emplace(COMMON_NSKP_SOLVER_PARAMETERS::USE_TLPS());
      map.emplace(COMMON_NSKP_SOLVER_PARAMETERS::PROPAGATE_FROM_LOCKFRAMES());
      map.emplace(COMMON_NSKP_SOLVER_PARAMETERS::ANCHOR_BIND_DISTANCE());
      map.emplace(COMMON_NSKP_SOLVER_PARAMETERS::ANCHOR_BIND_COEFF());
      map.emplace(COMMON_NSKP_SOLVER_PARAMETERS::LOCKFRAME_THRESHOLD());
      map.emplace(COMMON_NSKP_SOLVER_PARAMETERS::BIND_TO_LOCKFRAMES());
      map.emplace(COMMON_NSKP_SOLVER_PARAMETERS::MIN_KEY_FRAME_DISTANCE());

      return map;
    }
  };

  class COMMON_TLPS_SOLVER_PARAMETERS
  {
  public:
    static auto SCORE_INDEX(void)
    {
      return SPEL_SET_PARAMETER("scoreIndex", 0.0f);
    }

    static auto TEMPORAL_WINDOW_SIZE(void)
    {
      return SPEL_SET_PARAMETER("temporalWindowSize", 0.0f);
    }

    static auto PART_SHIFT_COEFFICIENT(void)
    {
      return SPEL_SET_PARAMETER("partShiftCoeff", 1.5f);
    }

    static auto PART_ROTATION_COEFFICIENT(void)
    {
      return SPEL_SET_PARAMETER("partRotationCoeff", 1.5f);
    }

    static auto TEMPORAL_LINK_COEFFICIENT(void)
    {
      return SPEL_SET_PARAMETER("tempCoeff", 0.1f);
    }

    static auto LOCKFRAME_THRESHOLD(void)
    {
      return SPEL_SET_PARAMETER("tlpsLockframeThreshold", 0.52f);
    }

    static auto getParameters(void)
    {
      std::map<std::string, float> map;
      
      map.emplace(COMMON_TLPS_SOLVER_PARAMETERS::SCORE_INDEX());
      map.emplace(COMMON_TLPS_SOLVER_PARAMETERS::TEMPORAL_WINDOW_SIZE());
      map.emplace(COMMON_TLPS_SOLVER_PARAMETERS::PART_SHIFT_COEFFICIENT());
      map.emplace(COMMON_TLPS_SOLVER_PARAMETERS::PART_ROTATION_COEFFICIENT());
      map.emplace(COMMON_TLPS_SOLVER_PARAMETERS::TEMPORAL_LINK_COEFFICIENT());
      map.emplace(COMMON_TLPS_SOLVER_PARAMETERS::LOCKFRAME_THRESHOLD());

      return map;
    }
  };
}
#endif // _SPELPARAMETERS_HPP_
