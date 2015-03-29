#include "surfDetector.hpp"

SurfDetector::SurfDetector(void)
{
  id = 0x5344;
}

int SurfDetector::getID(void)
{
  return id;
}

void SurfDetector::setID(int _id)
{
  id = _id;
}

//TODO (Vitaliy Koshura): Write real implementation here
void SurfDetector::train(vector <Frame*> frames, map <string, float> params)
{
}

//TODO (Vitaliy Koshura): Write real implementation here
vector <vector <LimbLabel> > SurfDetector::detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels)
{
  const float useSURFdet = 1.0f;
  const string sUseSURFdet = "useSURFdet";

  params.emplace(sUseSURFdet, useSURFdet);

  return limbLabels;
}

