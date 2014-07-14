#ifndef _INTERPOLATION_HPP_
#define _INTERPOLATION_HPP_

#include "frame.hpp"

class Interpolation : public Frame
{
  public:
    vector <Point2f> getPartPolygon(int partID);
  private:
};

#endif  // _INTERPOLATION_HPP_

