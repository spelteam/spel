#include "solvlet.hpp"

Solvlet::Solvlet(void)
{
	frameId=-1;
}
Solvlet::Solvlet(int id, vector<LimbLabel> _labels)
{
	frameId=id;
	labels = _labels;
}
    
Solvlet &Solvlet::operator=(const Solvlet &s)
{
  if (this == &s)
  {
    return *this;
  }
  this->setLabels(s.getLabels());
  this->setFrameID(s.getFrameID());
  return *this;
}

// bool operator==(const Solvlet &s) const
// {
// 	if(s.getFrameId==frameId && s.)
// }

// bool operator!=(const Solvlet &s) const
// {

// }

int Solvlet::getFrameID(void) const
{
	return frameId;
}

void Solvlet::setFrameID(int _id)
{
	frameId=_id;
}

vector<LimbLabel> Solvlet::getLabels(void) const
{
	return labels;
}
void Solvlet::setLabels(vector<LimbLabel> _labels)
{
	labels = _labels;
}

Skeleton Solvlet::toSkeleton(void)
{
  return Skeleton();
}
