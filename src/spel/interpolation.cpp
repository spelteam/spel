#include "interpolation.hpp"

namespace SPEL
{
  Interpolation::Interpolation(void) noexcept : Frame(INTERPOLATIONFRAME)
  {
  }

  Interpolation::Interpolation(const Interpolation & frame) noexcept : Frame(frame)
  {
  }

  Interpolation & Interpolation::operator=(const Interpolation & frame) noexcept
  {
    Frame::operator=(frame);
    return *this;
  }

  Interpolation::~Interpolation(void) noexcept
  {
  }

}
