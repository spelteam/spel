#include "keyframe.hpp"

namespace SPEL
{

  Keyframe::Keyframe(void) : frameType(KEYFRAME)
  {
  }

  FRAMETYPE Keyframe::getFrametype(void)
  {
    return frameType;
  }

}
