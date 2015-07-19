#include "lockframe.hpp"

namespace SPEL
{
  Lockframe::Lockframe(void) : frameType(LOCKFRAME)
  {
  }

  FRAMETYPE Lockframe::getFrametype(void)
  {
    return frameType;
  }

}
