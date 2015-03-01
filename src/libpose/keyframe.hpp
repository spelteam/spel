#ifndef _KEYFRAME_HPP_
#define _KEYFRAME_HPP_

#include "frame.hpp"

//This class represents user defined
//frame( implies that user analyzes the frame
// and make a mark points ).
class Keyframe : public Frame
{
  public:
    Keyframe(void);
    FRAMETYPE getFrametype(void);
  private:
    const FRAMETYPE frameType;
};

#endif  // _KEYFRAME_HPP_

