#ifndef _KEYFRAME_HPP_
#define _KEYFRAME_HPP_

// SPEL definitions
#include "predef.hpp"

#include "frame.hpp"

namespace SPEL
{
  /// <summary>
  /// This class represents user defined frame 
  /// (implies that user analyzes the frame and make a mark points).
  /// </summary>
  /// <seealso cref="Frame" />
  class Keyframe : public Frame
  {
  public:
    /// <summary>
    /// Initializes a new instance of the <see cref="Keyframe"/> class.
    /// </summary>
    Keyframe(void) noexcept;    
    /// <summary>
    /// Copy constructor.
    /// Initializes a new instance of the <see cref="Keyframe"/> class.
    /// </summary>
    /// <param name="frame">The frame.</param>
    Keyframe(const Keyframe &frame) noexcept; 
    /// <summary>Assignment operator.</summary>
    /// <param name="frame">The frame.</param>
    /// <returns>The frame.</returns>
    Keyframe& operator=(const Keyframe &frame) noexcept;
    /// <summary>
    /// Finalizes an instance of the <see cref="Keyframe"/> class.
    /// </summary>
    virtual ~Keyframe(void) noexcept;
  };

}

#endif  // _KEYFRAME_HPP_
