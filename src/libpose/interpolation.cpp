#include "interpolation.hpp"

Interpolation::Interpolation(void) : frameType(INTERPOLATIONFRAME)
{
}

FRAMETYPE Interpolation::getFrametype(void)
{
  return frameType;
}

