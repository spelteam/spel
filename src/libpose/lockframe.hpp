#ifndef _LOCKFRAME_HPP_
#define _LOCKFRAME_HPP_

#include "frame.hpp"

class Lockframe : public Frame
{
  public:
    Lockframe(void);
    FRAMETYPE getFrametype(void);
  private:
    const FRAMETYPE frameType;
};

#endif  // _LOCKFRAME_HPP_

