#ifndef _SPELOBJECT_HPP_
#define _SPELOBJECT_HPP_

#include "predef.hpp"
#include <cstdint>

class SpelObject
{
public:
  SpelObject(void) noexcept;
  ~SpelObject(void) noexcept;
  virtual void setDebugLevel(const uint8_t &_debugLevel) noexcept;
  virtual uint8_t getDebugLevel(void) const noexcept;
protected:
  static uint8_t debugLevel;
};

#endif // _SPELOBJECT_HPP_
