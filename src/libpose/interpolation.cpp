#include "interpolation.hpp"

Interpolation::Interpolation(void) : frameType(INTERPOLATIONFRAME)
{
}

vector <Point2f> Interpolation::getPartPolygon(int partID)
{
  vector <Point2f> result;

  return result;
}

FRAMETYPE Interpolation::getFrametype(void)
{
  return frameType;
}

