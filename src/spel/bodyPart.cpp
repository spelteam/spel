#include "bodyPart.hpp"
// See bodyPart.hpp and Skeleton.hpp for more info
namespace SPEL
{
  // default constructor
  BodyPart::BodyPart(void) noexcept
  {
    partID = 0;
    partName = "";
    parentJoint = 0;
    childJoint = 0;  
    isOccluded = false;
    expectedDistance = 0;
    partPolygon = POSERECT<cv::Point2f>();
    lwRatio = 0;
    relativeLength = 0;
    searchRadius = 0;
    rotationSearchRange = 0;
  }

  //copy constructor
  BodyPart::BodyPart(const BodyPart &bodyPart) noexcept
    : partID(bodyPart.partID),
    partName(bodyPart.partName),
    parentJoint(bodyPart.parentJoint),
    childJoint(bodyPart.childJoint),
    isOccluded(bodyPart.isOccluded),
    expectedDistance(bodyPart.expectedDistance),
    partPolygon(bodyPart.partPolygon),
    lwRatio(bodyPart.lwRatio),
    relativeLength(bodyPart.relativeLength),
    searchRadius(bodyPart.searchRadius),
    rotationSearchRange(bodyPart.rotationSearchRange)
  {
  }

  //move constructor
  BodyPart::BodyPart(BodyPart &&bodyPart) noexcept
    : partID(std::move(bodyPart.partID)),
    partName(std::move(bodyPart.partName)),
    parentJoint(std::move(bodyPart.parentJoint)),
    childJoint(std::move(bodyPart.childJoint)),
    isOccluded(std::move(bodyPart.isOccluded)),
    expectedDistance(std::move(bodyPart.expectedDistance)),
    partPolygon(std::move(bodyPart.partPolygon)),
    lwRatio(std::move(bodyPart.lwRatio)),
    relativeLength(std::move(bodyPart.relativeLength)),
    searchRadius(std::move(bodyPart.searchRadius)),
    rotationSearchRange(std::move(bodyPart.rotationSearchRange))

  {
  }


  // constructor with params
  BodyPart::BodyPart(int id, std::string name, int pJoint, int cJoint, bool isOcc, float spaceLen) noexcept
  {
    partID = id;
    partName = name;
    parentJoint = pJoint;
    childJoint =cJoint;
    isOccluded = isOcc;
    expectedDistance = spaceLen;
    partPolygon = POSERECT<cv::Point2f>();
    lwRatio = 0;
    relativeLength = 0;
    searchRadius = 0;
    rotationSearchRange = 0;
  }

  BodyPart::~BodyPart(void) noexcept
  {
    return;
  }

  int BodyPart::getPartID(void) const noexcept
  {
    return partID;
  }

  void BodyPart::setPartID(int _partID) noexcept
  {
    partID = _partID;
  }

  std::string BodyPart::getPartName(void) const noexcept
  {
    return partName;
  }

  void BodyPart::setPartName(std::string _partName) noexcept
  {
    partName = _partName;
  }

  int BodyPart::getParentJoint(void) const noexcept
  {
    return parentJoint;
  }

  void BodyPart::setParentJoint(int _parentJoint) noexcept
  {
    parentJoint = _parentJoint;
  }

  int BodyPart::getChildJoint(void) const noexcept
  {
    return childJoint;
  }

  void BodyPart::setChildJoint(int _childJoint) noexcept
  {
    childJoint = _childJoint;
  }

  bool BodyPart::getIsOccluded(void) const noexcept
  {
    return isOccluded;
  }

  void BodyPart::setIsOccluded(bool _isOccluded) noexcept
  {
    isOccluded = _isOccluded;
  }

  float BodyPart::getSearchRadius(void) const noexcept
  {
    return searchRadius;
  }

  void BodyPart::setSearchRadius(float _searchRadius) noexcept
  {
    searchRadius = _searchRadius;
  }

  float BodyPart::getExpectedDistance(void) const noexcept
  {
    return expectedDistance;
  }

  void BodyPart::setExpectedDistance(float _expectedDistance) noexcept
  {
    expectedDistance = _expectedDistance;
  }

  BodyPart& BodyPart::operator=(const BodyPart& bodyPart) noexcept
  {
    if (&bodyPart == this) 
      return *this;

    partID = bodyPart.partID;
    partName = bodyPart.partName;
    parentJoint = bodyPart.parentJoint;
    childJoint = bodyPart.childJoint;
    isOccluded = bodyPart.isOccluded;
    expectedDistance = bodyPart.expectedDistance;
    partPolygon = bodyPart.partPolygon;
    lwRatio = bodyPart.lwRatio;
    relativeLength = bodyPart.relativeLength;
    searchRadius = bodyPart.searchRadius;
    rotationSearchRange = bodyPart.rotationSearchRange;
    return *this;
  }

  BodyPart& BodyPart::operator=(BodyPart&& bodyPart) noexcept
  {
    partID = std::move(bodyPart.partID);
    std::swap(partName, bodyPart.partName);
    parentJoint = std::move(bodyPart.parentJoint);
    childJoint = std::move(bodyPart.childJoint);
    isOccluded = std::move(bodyPart.isOccluded);
    expectedDistance = std::move(bodyPart.expectedDistance);
    std::swap(partPolygon, bodyPart.partPolygon);
    lwRatio = std::move(bodyPart.lwRatio);
    relativeLength = std::move(bodyPart.relativeLength);
    searchRadius = std::move(bodyPart.searchRadius);
    rotationSearchRange = std::move(bodyPart.rotationSearchRange);
    return *this;
  }

  bool BodyPart::operator==(const BodyPart &bp) const noexcept
  {
    return this->getPartID() == bp.getPartID();
  }

  bool BodyPart::operator!=(const BodyPart &bp) const noexcept
  {
    return !(*this == bp);
  }

  std::ostream& operator<<(std::ostream& os, const BodyPart &bp) noexcept
  {
    return os << std::to_string(bp.getPartID());
  }

  POSERECT <cv::Point2f> BodyPart::getPartPolygon(void) const noexcept
  {
    return partPolygon;
  }

  void BodyPart::setPartPolygon(POSERECT <cv::Point2f> _partPolygon) noexcept
  {
    partPolygon = _partPolygon;
  }

  float BodyPart::getLWRatio(void) const noexcept
  {
    return lwRatio;
  }

  void BodyPart::setLWRatio(float _lwRatio) noexcept
  {
    lwRatio = _lwRatio;
  }

  float BodyPart::getRelativeLength(void) const noexcept
  {
    return relativeLength;
  }

  void BodyPart::setRelativeLength(float _relativeLength) noexcept
  {
    relativeLength = _relativeLength;
  }

  float BodyPart::getRotationSearchRange(void) const noexcept
  {
    return rotationSearchRange;
  }

  void BodyPart::setRotationSearchRange(float _rotationAngle) noexcept
  {
    rotationSearchRange = _rotationAngle;
  }

}
