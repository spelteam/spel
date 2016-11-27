#include "keyframe.hpp"

namespace SPEL
{
  Keyframe::Keyframe(void)  : Frame(KEYFRAME)
  {
  }
  
  Keyframe::Keyframe(const Keyframe & frame)  : Frame(frame)
  {
  }
    
  Keyframe & Keyframe::operator=(const Keyframe & frame) 
  {
    Frame::operator=(frame);
    return *this;
  }

  Keyframe::~Keyframe(void) 
  {
  }
}
