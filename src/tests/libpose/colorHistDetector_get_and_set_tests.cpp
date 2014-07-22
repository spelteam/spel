#include <gtest/gtest.h>
#include <colorHistDetector.hpp>

TEST(colorHistDetectorTest, GetAndSetTest)
{
  ColorHistDetector chd;
  int id = 3;
  chd.setID(id);
  
  EXPECT_EQ(id, chd.getID());
// get a constant
  EXPECT_EQ(8, chd.getNBins());
}

