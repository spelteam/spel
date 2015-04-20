#include <gtest/gtest.h>
#include <score.hpp>

TEST(scoreTest, OperatorsTest)
{
  Score s1(1.0, "Score1");
  Score s2(2.0, "Score2");
  Score s3(1.0, "Score3");

  EXPECT_TRUE(s1 < s2);
  EXPECT_TRUE(s2 > s1);
  EXPECT_TRUE(s3 == s1);
  EXPECT_TRUE(s1 != s2);

  Score s4 = s3;

  EXPECT_EQ(s4.getScore(), s3.getScore());
  EXPECT_EQ(s4.getDetName(), s3.getDetName());
  EXPECT_FLOAT_EQ(s4.getCoeff(), s3.getCoeff());
}

