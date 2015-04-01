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
  setRelativeLength(0);
}

//copy constructor
BodyPart::BodyPart(const BodyPart &bodyPart)
    : partID(bodyPart.partID),
      partName(bodyPart.partName),
      parentJoint(bodyPart.parentJoint),
      childJoint(bodyPart.childJoint),
      isOccluded(bodyPart.isOccluded),
      spaceLength(bodyPart.spaceLength),
      partPolygon(bodyPart.partPolygon),
      lwRatio(bodyPart.lwRatio),
      relativeLength(bodyPart.relativeLength)
{
}

//move constructor
BodyPart::BodyPart(BodyPart &&bodyPart)
    : partID(std::move(bodyPart.partID)),
      partName(std::move(bodyPart.partName)),
      parentJoint(std::move(bodyPart.parentJoint)),
      childJoint(std::move(bodyPart.childJoint)),
      isOccluded(std::move(bodyPart.isOccluded)),
      spaceLength(std::move(bodyPart.spaceLength)),
      partPolygon(std::move(bodyPart.partPolygon)),
      lwRatio(std::move(bodyPart.lwRatio)),
      relativeLength(std::move(bodyPart.relativeLength))
{
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

float BodyPart::getSearchRadius(void) const
{
  return searchRadius;
}

void BodyPart::setSearchRadius(float _searchRadius)
{
  searchRadius = _searchRadius;
}


float BodyPart::getSpaceLength(void) const
{
  return spaceLength;
}

void BodyPart::setSpaceLength(float _spaceLength)
{
  spaceLength = _spaceLength;
}

BodyPart& BodyPart::operator=( const BodyPart& bodyPart ){
    if( &bodyPart == this ) return *this;

    partID = bodyPart.partID;
    partName = bodyPart.partName;
    parentJoint = bodyPart.parentJoint;
    childJoint = bodyPart.childJoint;
    isOccluded = bodyPart.isOccluded;
    spaceLength = bodyPart.spaceLength;
    partPolygon = bodyPart.partPolygon;
    lwRatio = bodyPart.lwRatio;
    relativeLength = bodyPart.relativeLength;
    return *this;
}

BodyPart& BodyPart::operator=( BodyPart&& bodyPart ){
    partID = std::move(bodyPart.partID);
    std::swap( partName, bodyPart.partName );
    parentJoint = std::move(bodyPart.parentJoint);
    childJoint = std::move(bodyPart.childJoint);
    isOccluded = std::move(bodyPart.isOccluded);
    spaceLength = std::move(bodyPart.spaceLength);
    std::swap( partPolygon, bodyPart.partPolygon );
    lwRatio = std::move(bodyPart.lwRatio);
    relativeLength = std::move(bodyPart.relativeLength);
    return *this;
}

bool BodyPart::operator==(const BodyPart &bp) const
{
  return this->getPartID() == bp.getPartID();
}

bool BodyPart::operator!=(const BodyPart &bp) const
{
  return !(*this == bp);
}

std::ostream& operator<<(std::ostream& os, const BodyPart &bp)
{
    std::string s = std::to_string(bp.getPartID());
    return os<<s;
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

float BodyPart::getRelativeLength(void)
{
  return relativeLength;
}

void BodyPart::setRelativeLength(float _relativeLength)
{
  relativeLength = _relativeLength;
}
