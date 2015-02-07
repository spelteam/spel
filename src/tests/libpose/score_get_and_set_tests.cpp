#include <gtest/gtest.h>
#include <score.hpp>

TEST(scoreTest, GetAndSetTest)
{
  float score = 5.5;
  string detName = "detName";

  Score s1;
  s1.setScore(score);
  s1.setDetName(detName);

  EXPECT_EQ(score, s1.getScore());
  EXPECT_EQ(detName, s1.getDetName());
}

