#include <gtest/gtest.h>
#include <score.hpp>

TEST(scoreTest, ConstructorTest)
{
  Score s1;
  
  EXPECT_EQ(0, s1.getScore());
  EXPECT_TRUE(s1.getDetName().empty());

  float score = 12.5;
  string detName = "Test Name";
  Score s2(score, detName);

  EXPECT_EQ(score, s2.getScore());
  EXPECT_EQ(detName, s2.getDetName());
}

