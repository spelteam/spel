#include "spelObject.hpp"

#undef DebugMessage

#ifdef DEBUG
uint8_t SpelObject::debugLevel = 5;
#else
uint8_t SpelObject::debugLevel = 1;
#endif // DEBUG

SpelObject::SpelObject(void) noexcept
{
}

SpelObject::~SpelObject(void) noexcept
{
}

void SpelObject::DebugMessage(const std::string &message, const uint8_t &importance, const std::string &file, const int &line) noexcept
{
  if (SpelObject::debugLevel >= importance)
    std::cerr << file.c_str() << ":" << line << ":" << message.c_str() << std::endl;
}

void SpelObject::setDebugLevel(const uint8_t & _debugLevel) noexcept
{
  SpelObject::debugLevel = _debugLevel;
}

uint8_t SpelObject::getDebugLevel(void) noexcept
{
  return SpelObject::debugLevel;
}
