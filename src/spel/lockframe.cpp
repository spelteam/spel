#include "lockframe.hpp"

namespace SPEL
{
  Lockframe::Lockframe(void) noexcept : Frame(LOCKFRAME)
  {
  }

  Lockframe::Lockframe(const Lockframe & frame) noexcept : Frame(frame)
  {
  }

  Lockframe & Lockframe::operator=(const Lockframe & frame) noexcept
  {
    Frame::operator=(frame);
    return *this;
  }

  Lockframe::~Lockframe(void) noexcept
  {
  }
}
