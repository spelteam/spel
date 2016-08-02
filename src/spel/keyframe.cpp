#include "keyframe.hpp"

namespace SPEL
{
  Keyframe::Keyframe(void) noexcept : Frame(KEYFRAME)
  {
  }
  
  Keyframe::Keyframe(const Keyframe & frame) noexcept : Frame(frame)
  {
  }
    
  Keyframe & Keyframe::operator=(const Keyframe & frame) noexcept
  {
    Frame::operator=(frame);
    return *this;
  }

  Keyframe::~Keyframe(void) noexcept
  {
  }
}
