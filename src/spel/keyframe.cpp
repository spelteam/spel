// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com
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
