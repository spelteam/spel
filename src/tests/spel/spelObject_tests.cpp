// SPEL definitions
#include "predef.hpp"

#include <gtest/gtest.h>
#include <spelObject.hpp>

using namespace std;

namespace SPEL
{
  TEST(spelObjectTest, CommonTest)
  {
    EXPECT_EQ(5, SpelObject::getDebugLevel());
    SpelObject::setDebugLevel(2);
    EXPECT_EQ(2, SpelObject::getDebugLevel());
  }
}