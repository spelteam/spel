// SPEL definitions
#include "predef.hpp"

#include <gtest/gtest.h>
#include <score.hpp>

using namespace std;

namespace SPEL
{
  TEST(scoreTest, GetAndSetTest)
  {
    float score = 5.5f;
    string detName = "detName";
    float coeff = 0.25f;

    Score s1;
    s1.setScore(score);
    s1.setDetName(detName);
    s1.setCoeff(coeff);

    EXPECT_EQ(score, s1.getScore());
    EXPECT_EQ(detName, s1.getDetName());
    EXPECT_FLOAT_EQ(coeff, s1.getCoeff());

    bool b1 = false, b2 = true;
    s1.setIsWeak(b1);
    EXPECT_EQ(b1, s1.getIsWeak());
    s1.setIsWeak(b2);
    EXPECT_EQ(b2, s1.getIsWeak());
  }
}
