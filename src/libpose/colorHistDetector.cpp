#include "colorHistDetector.hpp"

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

