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
