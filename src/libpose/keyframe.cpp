#include "keyframe.hpp"

Keyframe::Keyframe(void) : frameType(KEYFRAME)
{
}

vector <Point2f> Keyframe::getPartPolygon(int partID)
{
  vector <Point2f> result;

  return result;
}

FRAMETYPE Keyframe::getFrametype(void)
{
  return frameType;
}

