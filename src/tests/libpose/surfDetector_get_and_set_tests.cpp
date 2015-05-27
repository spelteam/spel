#include <gtest/gtest.h>
#include <surfDetector.hpp>

TEST(surfDetectorTest, GetAndSetTest)
{
  SurfDetector sd;
  int id = 3;
  sd.setID(id);
  
  EXPECT_EQ(id, sd.getID());
}

