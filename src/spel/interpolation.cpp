// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com
#include "interpolation.hpp"

namespace SPEL
{
  Interpolation::Interpolation(void)  : Frame(INTERPOLATIONFRAME)
  {
  }

  Interpolation::Interpolation(const Interpolation & frame)  : Frame(frame)
  {
  }

  Interpolation & Interpolation::operator=(const Interpolation & frame) 
  {
    Frame::operator=(frame);
    return *this;
  }

  Interpolation::~Interpolation(void) 
  {
  }

}
