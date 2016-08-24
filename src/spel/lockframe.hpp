#ifndef _LOCKFRAME_HPP_
#define _LOCKFRAME_HPP_

// SPEL definitions
#include "predef.hpp"

#include "frame.hpp"

namespace SPEL
{
  /// <summary>
  /// This class represents frames that founded by solver.
  /// </summary>
  /// <seealso cref="Frame" />
  class Lockframe : public Frame
  {
  public:
    /// <summary>
    /// Initializes a new instance of the <see cref="Lockframe"/> class.
    /// </summary>
    Lockframe(void) noexcept;    
    /// <summary>
    /// Copy constructor.
    /// Initializes a new instance of the <see cref="Lockframe"/> class.
    /// </summary>
    /// <param name="frame">The frame.</param>
    Lockframe(const Lockframe &frame) noexcept;    
    /// <summary>Assignment operator.</summary>
    /// <param name="frame">The frame.</param>
    /// <returns>The frame.</returns>
    Lockframe& operator=(const Lockframe &frame) noexcept;
    /// <summary>
    /// Finalizes an instance of the <see cref="Lockframe"/> class.
    /// </summary>
    virtual ~Lockframe(void) noexcept;
  };
}

#endif  // _LOCKFRAME_HPP_
