#include "bodyPart.hpp"
BodyPart::BodyPart(void)
{
  setPartID(0);
  setPartName("");
  setParentJoint(0);
  setChildJoint(0);
  setIsOccluded(false);
  setSpaceLength(0);
}

BodyPart::BodyPart(int id, string name, BodyJoint *pJoint, BodyJoint *cJoint, bool isOcc, float spaceLen)
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

BodyJoint *BodyPart::getParentJoint(void) const
{
  return parentJoint;
}

void BodyPart::setParentJoint(BodyJoint *_parentJoint)
{
  parentJoint = _parentJoint;
}

BodyJoint *BodyPart::getChildJoint(void) const
{
  return childJoint;
}

void BodyPart::setChildJoint(BodyJoint *_childJoint)
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

