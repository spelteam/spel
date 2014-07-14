#ifndef _LOCKFRAME_HPP_
#define _LOCKFRAME_HPP_

#include "frame.hpp"

class Lockframe : public Frame
{
  public:
    vector <Point2f> getPartPolygon(int partID);
  private:
};

#endif  // _LOCKFRAME_HPP_

