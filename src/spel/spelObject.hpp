#ifndef _SPELOBJECT_HPP_
#define _SPELOBJECT_HPP_

#include "predef.hpp"

#include <string>

namespace SPEL
{
  /// <summary>
  /// Represents the class for debug logging.
  /// </summary>
  class SpelObject
  {
  public:
    /// <summary>Returns the instance of this class.</summary>
    /// <returns>The instance of this class.</returns>
    static SpelObject& Instance(void) ;
    /// <summary>
    /// Initializes a new instance of the <see cref="SpelObject"/> class.
    /// Deleted function.
    /// </summary>
    /// <param name="">The .</param>
    SpelObject(const SpelObject&) = delete;
    /// <summary> 
    /// Assignment operator.
    /// Deleted function.
    /// </summary>
    void operator=(const SpelObject&) = delete;
    /// <summary>
    /// Finalizes an instance of the <see cref="SpelObject"/> class.
    /// </summary>
    ~SpelObject(void) ;
    /// <summary>Print the debug message.</summary>
    /// <param name="message">The message.</param>
    /// <param name="importance">The importance.</param>
    /// <param name="file">The file.</param>
    /// <param name="line">The line.</param>
    static void DebugMessage(const std::string &message,
      const uint8_t importance, const std::string &file, const int line)
      ;
    /// <summary>Sets the debug level.</summary>
    /// <param name="_debugLevel">The debug level.</param>
    static void setDebugLevel(const uint8_t _debugLevel) ;
    /// <summary>Gets the debug level.</summary>
    /// <returns>The debug level.</returns>
    static uint8_t getDebugLevel(void) ;
  private:
    /// <summary>
    /// Prevents a default instance of the <see cref="SpelObject"/> class 
    /// from being created.
    /// </summary>
    SpelObject(void) ;
    /// <summary>The debug level.</summary>
    uint8_t debugLevel;
    /// <summary>Print the debug message.</summary>
    /// <param name="message">The message.</param>
    /// <param name="importance">The importance.</param>
    /// <param name="file">The file.</param>
    /// <param name="line">The line.</param>
    void _DebugMessage(const std::string &message, const uint8_t importance,
      const std::string &file, const int line) const ;
    /// <summary>Sets the debug level.</summary>
    /// <param name="_debugLevel">The debug level.</param>
    void _setDebugLevel(const uint8_t _debugLevel) ;
    /// <summary>Gets the debug level.</summary>
    /// <returns>The debug level.</returns>
    uint8_t _getDebugLevel(void) const ;
  };

  #define DebugMessage(message, importance) \
    SpelObject::DebugMessage(message, importance, __FILE__, __LINE__)
}
#endif // _SPELOBJECT_HPP_
