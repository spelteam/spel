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
    Interpolation(void) ;    
    /// <summary>
    /// Copy constructor.
    /// Initializes a new instance of the <see cref="Interpolation"/> class.
    /// </summary>
    /// <param name="frame">The frame.</param>
    Interpolation(const Interpolation &frame) ;    
    /// <summary>Assignment operator.</summary>
    /// <param name="frame">The frame.</param>
    /// <returns>The frame.</returns>
    Interpolation& operator=(const Interpolation &frame) ;
    /// <summary>
    /// Finalizes an instance of the <see cref="Interpolation"/> class.
    /// </summary>
    ~Interpolation(void) ;
  };
}
#endif  // _INTERPOLATION_HPP_
