#ifndef _SPELOBJECT_HPP_
#define _SPELOBJECT_HPP_

#include "predef.hpp"
#include <cstdint>
#include <iostream>
#include <string>

class SpelObject
{
public:
  static SpelObject& Instance(void) noexcept;

  SpelObject(const SpelObject&) = delete;
  void operator=(const SpelObject&) = delete;

  ~SpelObject(void) noexcept;

  static void DebugMessage(const std::string &message, const uint8_t importance, const std::string &file, const int line) noexcept;
  static void setDebugLevel(const uint8_t _debugLevel) noexcept;
  static uint8_t getDebugLevel(void) noexcept;
private:
  SpelObject(void) noexcept;
  uint8_t debugLevel;

  void _DebugMessage(const std::string &message, const uint8_t importance, const std::string &file, const int line) const noexcept;
  void _setDebugLevel(const uint8_t _debugLevel) noexcept;
  uint8_t _getDebugLevel(void) const noexcept;
};

#define DebugMessage(message, importance) SpelObject::DebugMessage(message, importance, __FILE__, __LINE__)

#endif // _SPELOBJECT_HPP_
