// SPEL definitions
#include "predef.hpp"

#include <gtest/gtest.h>
#include <spelObject.hpp>

using namespace std;

namespace SPEL
{
  TEST(spelObjectTest, CommonTest)
  {
    class A : public SpelObject {};
    class B : public SpelObject {};
    class C : public SpelObject {};

    A a;
    EXPECT_EQ(5, a.getDebugLevel());
    a.setDebugLevel(2);
    EXPECT_EQ(2, a.getDebugLevel());
    B b;
    EXPECT_EQ(2, b.getDebugLevel());
    b.setDebugLevel(3);
    C c;
    EXPECT_EQ(3, a.getDebugLevel());
    EXPECT_EQ(3, b.getDebugLevel());
    EXPECT_EQ(3, c.getDebugLevel());
  }
}