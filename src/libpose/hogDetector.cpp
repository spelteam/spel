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
void HogDetector::train(vector <Frame*> frames, map <string, float> params)
{
}

//TODO (Vitaliy Koshura): Write real implementation here
vector <vector <LimbLabel> > HogDetector::detect(Frame *frame, map <string, float> params)
{
  vector <vector <LimbLabel> > t;
  return t;
}

