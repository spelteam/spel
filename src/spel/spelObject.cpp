#include "spelObject.hpp"

SpelObject::SpelObject(void) noexcept
{
#ifdef DEBUG
  debugLevel = 5;
#else
  debugLevel = 1;
#endif // DEBUG
}

SpelObject::~SpelObject(void) noexcept
{
}

void SpelObject::setDebugLevel(const uint8_t & _debugLevel) noexcept
{
  debugLevel = _debugLevel;
}

uint8_t SpelObject::getDebugLevel(void) const noexcept
{
  return debugLevel;
}
