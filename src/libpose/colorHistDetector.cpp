#include "colorHistDetector.hpp"

//TODO (Vitaliy Koshura): Need unit test
ColorHistDetector::ColorHistDetector(uint8_t _nBins) : nBins(_nBins)
{
  partHistogramm.resize(nBins);
  for (uint8_t r = 0; r < nBins; r++)
  {
    partHistogramm[r].resize(nBins);
    for (uint8_t g = 0; g < nBins; g++)
    {
      partHistogramm[r][g].resize(nBins);
      for (int b = 0; b < nBins; b++)
      {
        partHistogramm[r][g][b] = 0.0;
      }
    }
  }
  sizeFG = 0;
  uniqueExists = false;
}

int ColorHistDetector::getID(void)
{
  return id;
}

void ColorHistDetector::setID(int _id)
{
  id = _id;
}

//TODO (Vitaliy Koshura): Write real implementation here
void ColorHistDetector::train(vector <Frame> frames, int id)
{
}

//TODO (Vitaliy Koshura): Write real implementation here
vector <vector <LimbLabel> > ColorHistDetector::detect(Frame *frame, vector <float> params)
{
  vector <vector <LimbLabel> > t;
  return t;
}

uint8_t ColorHistDetector::getNBins(void)
{
  return nBins;
}

//TODO (Vitaliy Koshura): Need unit test
float ColorHistDetector::computePixelBelongingLikelihood(uint8_t r, uint8_t g, uint8_t b)
{
  uint8_t factor = ceil(pow(2, 8)/getNBins());
  float isFG = partHistogramm[r/factor][g/factor][b/factor];
  return isFG;
}

//TODO (Vitaliy Koshura>: need unit test
void ColorHistDetector::setPartHistogramm(const vector <Point3i> &partColors)
{
  // do not add sample if the number of pixels is zero
  if (partColors.size() == 0)
    return;
  uniqueExists = false;
  uint8_t factor = ceil(pow(2, 8)/getNBins());  // divide the color space into bins
  sizeFG = partColors.size();
  fgNumSamples = 1;
  fgSampleSizes.clear();
  fgSampleSizes.push_back(partColors.size());

// clear histogram first
  for(uint8_t r = 0; r < getNBins(); r++)
  {
    for(uint8_t g = 0; g < getNBins(); g++)
    {
      for(uint8_t b = 0; b < getNBins(); b++)
      {
        partHistogramm[r][g][b] = 0.0;
      }
    }
  }
  for(int i = 0; i < partColors.size(); i++)
  {
    uint8_t r = partColors[i].x / factor;
    uint8_t g = partColors[i].y / factor;
    uint8_t b = partColors[i].z / factor;
    partHistogramm[r][g][b]++;
  }
  for(uint8_t r = 0; r < getNBins(); r++)
  {
    for(uint8_t g = 0; g < getNBins(); g++)
    {
      for(uint8_t b = 0; b < getNBins(); b++)
      {
// normalise the histograms
        partHistogramm[r][g][b] /= sizeFG;
      }
    }
  }
}

//TODO (Vitaliy Koshura>: need unit test
void ColorHistDetector::addPartHistogramm(const vector <Point3i> &partColors, uint32_t nBlankPixels)
{
  if(partColors.size()==0) //do not add sample if the number of pixels is zero
    return;
  uniqueExists = false;
//un-normalise
  for(uint8_t r = 0; r < getNBins(); r++)
  {
    for(uint8_t g = 0; g < getNBins(); g++)
    {
      for(uint8_t b = 0; b < getNBins(); b++)
      {
        partHistogramm[r][g][b] *= sizeFG;
      }
    }
  }

  int factor = ceil(pow(2, 8)/getNBins());//divide the color space into bins
  sizeFG += partColors.size();
  fgNumSamples++;
  fgSampleSizes.push_back(partColors.size());

  for(int i = 0; i < partColors.size(); i++)
  {
    uint8_t r = partColors[i].x / factor;
    uint8_t g = partColors[i].y / factor;
    uint8_t b = partColors[i].z / factor;
    partHistogramm[r][g][b]++;
  }

//renormalise
  for(uint8_t r = 0; r < getNBins(); r++)
  {
    for(uint8_t g = 0; g < getNBins(); g++)
    {
      for(uint8_t b = 0; b < getNBins(); b++)
      {
//normalise the histograms
        partHistogramm[r][g][b] /= sizeFG;
      }
    }
  }

  fgBlankSizes.push_back(nBlankPixels); //add the number of blank pixels for this model
}

float ColorHistDetector::getAvgSampleSizeFg(void)
{
  float sum = 0;
  for(uint32_t i = 0; i < fgSampleSizes.size(); i++)
  {
    sum += fgSampleSizes[i];
  }
  sum /= fgNumSamples;
  return sum;
}

float ColorHistDetector::getAvgSampleSizeFgBetween(uint32_t s1, uint32_t s2)
{
  if(s1 >= fgSampleSizes.size() || s2 >= fgSampleSizes.size())
    return 0;
  return (fgSampleSizes[s1] + fgSampleSizes[s2]) / 2.0;
}
