#include "bodyPart.hpp"
BodyPart::BodyPart(void)
{
}

BodyPart::BodyPart(int id, string name, BodyJoint *pJoint, BodyJoint *cJoint)
{
  setPartID(id);
  setPartName(name);
  setParentJoint(pJoint);
  setChildJoint(cJoint);
}

int BodyPart::getPartID(void)
{
  return partID;
}

void BodyPart::setPartID(int _partID)
{
  partID = _partID;
}

string BodyPart::getPartName(void)
{
  return partName;
}

void BodyPart::setPartName(string _partName)
{
  partName = _partName;
}

BodyJoint *BodyPart::getParentJoint(void)
{
  return parentJoint;
}

void BodyPart::setParentJoint(BodyJoint *_parentJoint)
{
  parentJoint = _parentJoint;
}

BodyJoint *BodyPart::getChildJoint(void)
{
  return childJoint;
}

void BodyPart::setChildJoint(BodyJoint *_childJoint)
{
  childJoint = _childJoint;
}

bool BodyPart::getIsOccluded(void)
{
  return isOccluded;
}

void BodyPart::setIsOccluded(bool _isOccluded)
{
  isOccluded = _isOccluded;
}

float BodyPart::getSpaceLength(void)
{
  return spaceLength;
}

void BodyPart::setSpaceLength(float _spaceLength)
{
  spaceLength = _spaceLength;
}

