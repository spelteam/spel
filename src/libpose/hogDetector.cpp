#include "hogDetector.hpp"

int HogDetector::getID(void)
{
  return id;
}

void HogDetector::setID(int _id)
{
  id = _id;
}

//TODO (Vitaliy Koshura): Write real implementation here
void HogDetector::train(vector <Frame> frames, int id)
{
}

//TODO (Vitaliy Koshura): Write real implementation here
vector <vector <LimbLabel> > HogDetector::detect(Frame *frame, vector <float> params)
{
  vector <vector <LimbLabel> > t;
  return t;
}

