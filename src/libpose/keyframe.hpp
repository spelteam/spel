#ifndef _KEYFRAME_HPP_
#define _KEYFRAME_HPP_

#include "frame.hpp"

class Keyframe : public Frame
{
  public:
    Keyframe(void);
    vector <Point2f> getPartPolygon(int partID);
    FRAMETYPE getFrametype(void);
  private:
    const FRAMETYPE frameType;
};

#endif  // _KEYFRAME_HPP_

