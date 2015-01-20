#include "bodyPart.hpp"
// See bodyPart.hpp and Skeleton.hpp for more info

// default constructor
BodyPart::BodyPart(void)
{
  setPartID(0);
  setPartName("");
  setParentJoint(0);
  setChildJoint(0);
  setIsOccluded(false);
  setSpaceLength(0);
  setLWRatio(0);
}

// constructor with params
BodyPart::BodyPart(int id, string name, int pJoint, int cJoint, bool isOcc, float spaceLen)
{
  setPartID(id);
  setPartName(name);
  setParentJoint(pJoint);
  setChildJoint(cJoint);
  setIsOccluded(isOcc);
  setSpaceLength(spaceLen);
}

int BodyPart::getPartID(void) const
{
  return partID;
}

void BodyPart::setPartID(int _partID)
{
  partID = _partID;
}

string BodyPart::getPartName(void) const
{
  return partName;
}

void BodyPart::setPartName(string _partName)
{
  partName = _partName;
}

int BodyPart::getParentJoint(void) const
{
  return parentJoint;
}

void BodyPart::setParentJoint(int _parentJoint)
{
  parentJoint = _parentJoint;
}

int BodyPart::getChildJoint(void) const
{
  return childJoint;
}

void BodyPart::setChildJoint(int _childJoint)
{
  childJoint = _childJoint;
}

bool BodyPart::getIsOccluded(void) const
{
  return isOccluded;
}

void BodyPart::setIsOccluded(bool _isOccluded)
{
  isOccluded = _isOccluded;
}

float BodyPart::getSpaceLength(void) const
{
  return spaceLength;
}

void BodyPart::setSpaceLength(float _spaceLength)
{
  spaceLength = _spaceLength;
}

bool BodyPart::operator==(const BodyPart &bp) const
{
  return this->getPartID() == bp.getPartID();
}

bool BodyPart::operator!=(const BodyPart &bp) const
{
  return !(*this == bp);
}

POSERECT <Point2f> BodyPart::getPartPolygon(void)
{
  return partPolygon;
}

void BodyPart::setPartPolygon(POSERECT <Point2f> _partPolygon)
{
  partPolygon = _partPolygon;
}

float BodyPart::getLWRatio(void)
{
  return lwRatio;
}

void BodyPart::setLWRatio(float _lwRatio)
{
  lwRatio = _lwRatio;
}

