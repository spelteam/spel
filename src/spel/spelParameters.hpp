#ifndef _SPELPARAMETERS_HPP_
#define _SPELPARAMETERS_HPP_
#include <utility>
#include <string>

namespace SPEL
{
  typedef std::pair<std::string, float> SPEL_PARAMETER;

  inline SPEL_PARAMETER SPEL_SET_PARAMETER(const std::string &&name, const float &&value)
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
