#include "colorHistDetector.hpp"

//TODO (Vitaliy Koshura): Need unit test
ColorHistDetector::ColorHistDetector(void) : nBins(8)  // for 32 bit colourspace
{
  partHistogramm.resize(nBins);
  for (int r = 0; r < nBins; r++)
  {
    partHistogramm[r].resize(nBins);
    for (int g = 0; g < nBins; g++)
    {
      partHistogramm[r][g].resize(nBins);
      for (int b = 0; b < nBins; b++)
      {
        partHistogramm[r][g][b] = 0.0;
      }
    }
  }
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

