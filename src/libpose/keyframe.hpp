#ifndef _KEYFRAME_HPP_
#define _KEYFRAME_HPP_

#include "frame.hpp"

class Keyframe : public Frame
{
  public:
    vector <Point2f> getPartPolygon(int partID);
  private:
};

#endif  // _KEYFRAME_HPP_

