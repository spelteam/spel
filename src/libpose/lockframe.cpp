#include "lockframe.hpp"

Lockframe::Lockframe(void) : frameType(LOCKFRAME)
{
}

vector <Point2f> Lockframe::getPartPolygon(int partID)
{
  vector <Point2f> result;

  return result;
}

FRAMETYPE Lockframe::getFrametype(void)
{
  return frameType;
}

