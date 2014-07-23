#include <gtest/gtest.h>
#include <colorHistDetector.hpp>

TEST(colorHistDetectorTest, GetAndSetTest)
{
  ColorHistDetector chd;
  int id = 3;
  chd.setID(id);
  
  EXPECT_EQ(id, chd.getID());
// get a constant
  uint8_t nBins = 8;
  EXPECT_EQ(nBins, chd.getNBins());

  nBins = 7;
  ColorHistDetector chd1(nBins);
  EXPECT_EQ(nBins, chd1.getNBins());

// we can't test setter so test only getter (setter will be automatically tested by other tests. I hope so...)
// should return zero because of clean class without partHistogramm.
  EXPECT_EQ(0, chd.getSizeFG());
}

