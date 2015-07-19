#include "interpolation.hpp"

namespace SPEL
{
  Interpolation::Interpolation(void) : frameType(INTERPOLATIONFRAME)
  {
  }

  FRAMETYPE Interpolation::getFrametype(void)
  {
    return frameType;
  }

}
