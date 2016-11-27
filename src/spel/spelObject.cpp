#include "spelObject.hpp"

#include <iostream>

#undef DebugMessage
namespace SPEL
{
  SpelObject & SpelObject::Instance(void) 
  {
    static SpelObject instance;
    return instance;
  }

  SpelObject::SpelObject(void) 
  {
#ifdef DEBUG
    debugLevel = 5;
#else
    debugLevel = 1;
#endif // DEBUG
  }

  void SpelObject::_DebugMessage(const std::string & message,
    const uint8_t importance, const std::string & file, const int line) const
    
  {
    if (debugLevel >= importance)
      std::cerr << file.c_str() << ":" << line << ":" << message.c_str() <<
      std::endl;
  }

  void SpelObject::_setDebugLevel(const uint8_t _debugLevel) 
  {
    debugLevel = _debugLevel;
  }

  uint8_t SpelObject::_getDebugLevel(void) const 
  {
    return debugLevel;
  }

  SpelObject::~SpelObject(void) 
  {
  }

  void SpelObject::DebugMessage(const std::string &message,
    const uint8_t importance, const std::string &file, const int line) 
  {
    SpelObject::Instance()._DebugMessage(message, importance, file, line);
  }

  void SpelObject::setDebugLevel(const uint8_t _debugLevel) 
  {
    SpelObject::Instance()._setDebugLevel(_debugLevel);
  }

  uint8_t SpelObject::getDebugLevel(void) 
  {
    return SpelObject::Instance()._getDebugLevel();
  }
}
