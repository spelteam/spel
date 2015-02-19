#ifndef _LOCKFRAME_HPP_
#define _LOCKFRAME_HPP_

#include "frame.hpp"

//This class represents frames that
//founded by solver
class Lockframe : public Frame
{
  public:
    Lockframe(void);
    FRAMETYPE getFrametype(void);
  private:
    const FRAMETYPE frameType;
};

#endif  // _LOCKFRAME_HPP_

