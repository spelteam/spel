#ifndef _INTERPOLATION_HPP_
#define _INTERPOLATION_HPP_

// SPEL definitions
#include "predef.hpp"

#include "frame.hpp"
namespace SPEL
{
  /// <summary>
  /// This class represents frames that interpolated between keyframes.
  /// </summary>
  /// <seealso cref="Frame" />
  class Interpolation : public Frame
  {
  public:
    /// <summary>
    /// Initializes a new instance of the <see cref="Interpolation"/> class.
    /// </summary>
    Interpolation(void) noexcept;    
    /// <summary>
    /// Copy constructor.
    /// Initializes a new instance of the <see cref="Interpolation"/> class.
    /// </summary>
    /// <param name="frame">The frame.</param>
    Interpolation(const Interpolation &frame) noexcept;    
    /// <summary>Assignment operator.</summary>
    /// <param name="frame">The frame.</param>
    /// <returns>The frame.</returns>
    Interpolation& operator=(const Interpolation &frame) noexcept;
    /// <summary>
    /// Finalizes an instance of the <see cref="Interpolation"/> class.
    /// </summary>
    ~Interpolation(void) noexcept;
  };
}
#endif  // _INTERPOLATION_HPP_
