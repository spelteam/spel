#include "lockframe.hpp"

Lockframe::Lockframe(void) : frameType(LOCKFRAME)
{
}

FRAMETYPE Lockframe::getFrametype(void)
{
  return frameType;
}

