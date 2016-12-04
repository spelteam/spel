// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com
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
