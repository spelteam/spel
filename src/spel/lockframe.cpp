#include "lockframe.hpp"

namespace SPEL
{
  Lockframe::Lockframe(void)  : Frame(LOCKFRAME)
  {
  }

  Lockframe::Lockframe(const Lockframe & frame)  : Frame(frame)
  {
  }

  Lockframe & Lockframe::operator=(const Lockframe & frame) 
  {
    Frame::operator=(frame);
    return *this;
  }

  Lockframe::~Lockframe(void) 
  {
  }
}
