#include "keyframe.hpp"

Keyframe::Keyframe(void) : frameType(KEYFRAME)
{
}

FRAMETYPE Keyframe::getFrametype(void)
{
  return frameType;
}

