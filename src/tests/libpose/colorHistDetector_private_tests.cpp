#include <gtest/gtest.h>
#include <colorHistDetector.hpp>

TEST(colorHistDetectorTest, PrivateFields)
{
  //Testing PartModel constructor
  uint8_t _nBins = 10;
  const int maxIndex = _nBins - 1;
  ColorHistDetector::PartModel x(_nBins);
  EXPECT_EQ(0.0f, x.partHistogramm[maxIndex][maxIndex][maxIndex]);
  EXPECT_EQ(x.nBins, x.partHistogramm[maxIndex][maxIndex].size());
  EXPECT_EQ(0.0f, x.bgHistogramm[maxIndex][maxIndex][maxIndex]);
  EXPECT_EQ(x.nBins, x.bgHistogramm[maxIndex][maxIndex].size());
  
  //Testing PartModel operator "="
  x.sizeFG = 1;
  x.sizeBG = 2;
  x.fgNumSamples = 3;
  x.bgNumSamples = 4;
  x.fgSampleSizes.push_back(100);
  x.bgSampleSizes.push_back(200);
  x.fgBlankSizes.push_back(300);
  ColorHistDetector::PartModel y = x;
  EXPECT_EQ(x.partHistogramm, y.partHistogramm);
  EXPECT_EQ(x.bgHistogramm, y.bgHistogramm);
  EXPECT_EQ(x.sizeFG, y.sizeFG);
  EXPECT_EQ(x.fgNumSamples, y.fgNumSamples);
  EXPECT_EQ(x.bgNumSamples, y.bgNumSamples);
  EXPECT_EQ(x.fgSampleSizes, y.fgSampleSizes);
  EXPECT_EQ(x.bgSampleSizes, y.bgSampleSizes);
  EXPECT_EQ(x.fgBlankSizes, y.fgBlankSizes);

  //Testing ColorHistDetector constructor with parameter "_nBins"
  ColorHistDetector chd1(_nBins);
  EXPECT_EQ(_nBins, chd1.nBins);
  
  //Testing  function computePixelBelongingLikelihood
  const uint8_t _nB = 8, outside = 255;
  const uint8_t _factor = static_cast<uint8_t> (ceil(pow(2, 8) / _nB));
  uint8_t i = 7;
  uint8_t t = i / _factor;
  ColorHistDetector chd2(_nB);
  ColorHistDetector::PartModel z(_nB);
  z.partHistogramm[t][t][t] = 3.14f;
  EXPECT_EQ(z.partHistogramm[t][t][t], chd2.computePixelBelongingLikelihood(z, i, i, i));
  EXPECT_EQ(0.f, chd2.computePixelBelongingLikelihood(z, outside, outside, outside));
  
  //Testing function setPartHistogramm
  const uint8_t nBins = 8;
  const int ColorsCount = 255, K = ColorsCount / RAND_MAX;
  const uint8_t factor = static_cast<uint8_t> (ceil(pow(2, 8) / _nB));
  ColorHistDetector::PartModel partModel1(nBins), partModel2 = partModel1;
  vector <Point3i> Colors;
  for (int c = 0; c < ColorsCount; c++)
  Colors.push_back(Point3i(rand()*K, rand()*K, rand()*K));
  partModel1.sizeFG = ColorsCount;
  partModel1.fgNumSamples = 1;
  partModel1.fgSampleSizes.push_back(Colors.size());
  for (uint32_t i = 0; i < ColorsCount; i++)
    partModel1.partHistogramm[Colors[i].x / factor][Colors[i].y / factor][Colors[i].z / factor]++;
  for (uint8_t r = 0; r < partModel1.nBins; r++)
    for (uint8_t g = 0; g < partModel1.nBins; g++)
      for (uint8_t b = 0; b < partModel1.nBins; b++)
        partModel1.partHistogramm[r][g][b] /= partModel1.sizeFG;
  
  ColorHistDetector chd3(nBins);
  chd3.setPartHistogramm(partModel2, Colors);
  EXPECT_EQ(partModel1.partHistogramm, partModel2.partHistogramm);
  EXPECT_EQ(partModel1.bgHistogramm, partModel2.bgHistogramm);
  EXPECT_EQ(partModel1.sizeFG, partModel2.sizeFG);
  EXPECT_EQ(partModel1.fgNumSamples, partModel2.fgNumSamples);
  EXPECT_EQ(partModel1.bgNumSamples, partModel2.bgNumSamples);
  EXPECT_EQ(partModel1.fgSampleSizes, partModel2.fgSampleSizes);
  EXPECT_EQ(partModel1.bgSampleSizes, partModel2.bgSampleSizes);
  EXPECT_EQ(partModel1.fgBlankSizes, partModel2.fgBlankSizes);
  
  //Testing function addPartHistogramm
  uint32_t nBlankPixels = 100;
  for (uint8_t r = 0; r < partModel1.nBins; r++)
    for (uint8_t g = 0; g < partModel1.nBins; g++)
      for (uint8_t b = 0; b < partModel1.nBins; b++)
        partModel1.partHistogramm[r][g][b] *= partModel1.sizeFG;
  partModel1.sizeFG += Colors.size();
  partModel1.fgNumSamples++;
  partModel1.fgSampleSizes.push_back(Colors.size());
  for (uint32_t i = 0; i < ColorsCount; i++)
    partModel1.partHistogramm[Colors[i].x / factor][Colors[i].y / factor][Colors[i].z / factor]++;
  for (uint8_t r = 0; r < partModel1.nBins; r++)
    for (uint8_t g = 0; g < partModel1.nBins; g++)
      for (uint8_t b = 0; b < partModel1.nBins; b++)
        partModel1.partHistogramm[r][g][b] /= partModel1.sizeFG;
  partModel1.fgBlankSizes.push_back(nBlankPixels);
  
  chd3.addPartHistogramm(partModel2, Colors, nBlankPixels);
  EXPECT_EQ(partModel1.partHistogramm, partModel2.partHistogramm);
  EXPECT_EQ(partModel1.bgHistogramm, partModel2.bgHistogramm);
  EXPECT_EQ(partModel1.sizeFG, partModel2.sizeFG);
  EXPECT_EQ(partModel1.fgNumSamples, partModel2.fgNumSamples);
  EXPECT_EQ(partModel1.bgNumSamples, partModel2.bgNumSamples);
  EXPECT_EQ(partModel1.fgSampleSizes, partModel2.fgSampleSizes);
  EXPECT_EQ(partModel1.bgSampleSizes, partModel2.bgSampleSizes);
  EXPECT_EQ(partModel1.fgBlankSizes, partModel2.fgBlankSizes);
  
  //Testing function getAvgSampleSizeFg
  float Sum = 0;
  for (uint32_t i = 0; i < partModel1.fgSampleSizes.size(); i++)
  Sum += partModel1.fgSampleSizes[i];
  Sum /= partModel1.fgNumSamples;
  EXPECT_EQ(Sum, chd3.getAvgSampleSizeFg(partModel2));
  
  //Testing function getAvgSampleSizeFgBetween
  uint32_t s1 = 0, s2 = 0;
  float f = (partModel1.fgSampleSizes[s1] + partModel1.fgSampleSizes[s2]) / 2.0f;
  EXPECT_EQ(f, chd3.getAvgSampleSizeFgBetween(partModel2, s1, s2));
  EXPECT_EQ(0.f, chd3.getAvgSampleSizeFgBetween(partModel2, partModel1.fgSampleSizes.size(), s2));
  
  //Testing function matchPartHistogramsED
  float distance = 0;
  for (uint8_t r = 0; r < partModel1.nBins; r++)
  for (uint8_t g = 0; g < partModel1.nBins; g++)
  for (uint8_t b = 0; b < partModel1.nBins; b++)
    distance += pow(partModel1.partHistogramm[r][g][b] - partModel1.partHistogramm[r][g][b], 2.0f);
  f = chd3.matchPartHistogramsED(partModel1, partModel1);
  EXPECT_EQ(sqrt(distance), f);
  
  //Testing function addBackgroundHistogramm
  vector <Point3i> cEmpty;
  for (uint8_t r = 0; r < partModel1.nBins; r++)
    for (uint8_t g = 0; g < partModel1.nBins; g++)
      for (uint8_t b = 0; b < partModel1.nBins; b++)
        partModel1.bgHistogramm[r][g][b] *= partModel1.sizeBG;
  partModel1.sizeBG += Colors.size();
  partModel1.bgNumSamples++;
  partModel1.bgSampleSizes.push_back(Colors.size());
  for (uint32_t i = 0; i < Colors.size(); i++)
    partModel1.bgHistogramm[Colors[i].x / factor][Colors[i].y / factor][Colors[i].z / factor]++;
  for (uint8_t r = 0; r < partModel1.nBins; r++)
    for (uint8_t g = 0; g < partModel1.nBins; g++)
      for (uint8_t b = 0; b < partModel1.nBins; b++)
        partModel1.bgHistogramm[r][g][b] /= (float)partModel1.sizeBG;
  
  chd3.addBackgroundHistogramm(partModel2, cEmpty);
  EXPECT_NE(partModel1.bgHistogramm, partModel2.bgHistogramm);
  chd3.addBackgroundHistogramm(partModel2, Colors);
  EXPECT_EQ(partModel1.partHistogramm, partModel2.partHistogramm);
  EXPECT_EQ(partModel1.bgHistogramm, partModel2.bgHistogramm);
  EXPECT_EQ(partModel1.sizeFG, partModel2.sizeFG);
  EXPECT_EQ(partModel1.fgNumSamples, partModel2.fgNumSamples);
  EXPECT_EQ(partModel1.bgNumSamples, partModel2.bgNumSamples);
  EXPECT_EQ(partModel1.fgSampleSizes, partModel2.fgSampleSizes);
  EXPECT_EQ(partModel1.bgSampleSizes, partModel2.bgSampleSizes);
  EXPECT_EQ(partModel1.fgBlankSizes, partModel2.fgBlankSizes);

}
