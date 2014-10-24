#ifndef _INTERPOLATION_HPP_
#define _INTERPOLATION_HPP_

#include "frame.hpp"

class Interpolation : public Frame
{
  public:
    Interpolation(void);
    FRAMETYPE getFrametype(void);
  private:
    const FRAMETYPE frameType;
};

#endif  // _INTERPOLATION_HPP_

