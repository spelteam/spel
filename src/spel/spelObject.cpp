#include "spelObject.hpp"

#undef DebugMessage
namespace SPEL
{
  SpelObject & SpelObject::Instance(void) noexcept
  {
    static SpelObject instance;
    return instance;
  }

  SpelObject::SpelObject(void) noexcept
  {
#ifdef DEBUG
    debugLevel = 5;
#else
    debugLevel = 1;
#endif // DEBUG
  }

  void SpelObject::_DebugMessage(const std::string & message,
    const uint8_t importance, const std::string & file, const int line) const
    noexcept
  {
    if (debugLevel >= importance)
      std::cerr << file.c_str() << ":" << line << ":" << message.c_str() <<
      std::endl;
  }

  void SpelObject::_setDebugLevel(const uint8_t _debugLevel) noexcept
  {
    debugLevel = _debugLevel;
  }

  uint8_t SpelObject::_getDebugLevel(void) const noexcept
  {
    return debugLevel;
  }

  SpelObject::~SpelObject(void) noexcept
  {
  }

  void SpelObject::DebugMessage(const std::string &message,
    const uint8_t importance, const std::string &file, const int line) noexcept
  {
    SpelObject::Instance()._DebugMessage(message, importance, file, line);
  }

  void SpelObject::setDebugLevel(const uint8_t _debugLevel) noexcept
  {
    SpelObject::Instance()._setDebugLevel(_debugLevel);
  }

  uint8_t SpelObject::getDebugLevel(void) noexcept
  {
    return SpelObject::Instance()._getDebugLevel();
  }
}
