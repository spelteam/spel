#ifndef _LOCKFRAME_HPP_
#define _LOCKFRAME_HPP_

// SPEL definitions
#include "predef.hpp"

#include "frame.hpp"

namespace SPEL
{
  ///This class represents frames that
  ///founded by solver
  class Lockframe : public Frame
  {
  public:
    Lockframe(void) noexcept;
    virtual ~Lockframe(void) noexcept;
  };
}

#endif  // _LOCKFRAME_HPP_
