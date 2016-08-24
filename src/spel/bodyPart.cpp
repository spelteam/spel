#include "bodyPart.hpp"
#include "spelObject.hpp"
// See bodyPart.hpp and Skeleton.hpp for more info
namespace SPEL
{
  BodyPart::BodyPart(void) noexcept : BodyPart(0, "", 0, 0, false, 0.0f)
  {    
  }

  BodyPart::BodyPart(const BodyPart &bodyPart) noexcept
    : m_partID(bodyPart.m_partID),
    m_partName(bodyPart.m_partName),
    m_parentJoint(bodyPart.m_parentJoint),
    m_childJoint(bodyPart.m_childJoint),
    m_isOccluded(bodyPart.m_isOccluded),
    m_expectedDistance(bodyPart.m_expectedDistance),
    m_partPolygon(bodyPart.m_partPolygon),
    m_lwRatio(bodyPart.m_lwRatio),
    m_relativeLength(bodyPart.m_relativeLength),
    m_searchRadius(bodyPart.m_searchRadius),
    m_rotationSearchRange(bodyPart.m_rotationSearchRange)
  {
  }

  BodyPart::BodyPart(BodyPart &&bodyPart) noexcept
    : m_partID(std::move(bodyPart.m_partID)),
    m_partName(std::move(bodyPart.m_partName)),
    m_parentJoint(std::move(bodyPart.m_parentJoint)),
    m_childJoint(std::move(bodyPart.m_childJoint)),
    m_isOccluded(std::move(bodyPart.m_isOccluded)),
    m_expectedDistance(std::move(bodyPart.m_expectedDistance)),
    m_partPolygon(std::move(bodyPart.m_partPolygon)),
    m_lwRatio(std::move(bodyPart.m_lwRatio)),
    m_relativeLength(std::move(bodyPart.m_relativeLength)),
    m_searchRadius(std::move(bodyPart.m_searchRadius)),
    m_rotationSearchRange(std::move(bodyPart.m_rotationSearchRange))

  {
  }

  BodyPart::BodyPart(const int id, const std::string & name, 
    const int parentJoint, const int childJoint) noexcept : 
  BodyPart(id, name, parentJoint, childJoint, false, 0.0f)
  {    
  }

  BodyPart::BodyPart(const int id, const std::string & name, 
    const int parentJoint, const int childJoint, 
    const bool isOccluded) noexcept : 
  BodyPart(id, name, parentJoint, childJoint, isOccluded, 0.0f)
  {    
  }

  BodyPart::BodyPart(const int id, const std::string &name, 
    const int parentJoint, const int childJoint, const bool isOccluded, 
    const float spaceLength) noexcept
  {
    m_partID = id;
    m_partName = name;
    m_parentJoint = parentJoint;
    m_childJoint = childJoint;
    m_isOccluded = isOccluded;
    m_expectedDistance = spaceLength;
    m_partPolygon = POSERECT<cv::Point2f>();
    m_lwRatio = 0;
    m_relativeLength = 0;
    m_searchRadius = 0;
    m_rotationSearchRange = 0;
  }

  BodyPart::~BodyPart(void) noexcept
  {
  }

  int BodyPart::getPartID(void) const noexcept
  {
    return m_partID;
  }

  void BodyPart::setPartID(const int partID) noexcept
  {
    m_partID = partID;
  }

  std::string BodyPart::getPartName(void) const noexcept
  {
    return m_partName;
  }

  void BodyPart::setPartName(const std::string &partName) noexcept
  {
    m_partName = partName;
  }

  int BodyPart::getParentJoint(void) const noexcept
  {
    return m_parentJoint;
  }

  void BodyPart::setParentJoint(const int parentJoint) noexcept
  {
    m_parentJoint = parentJoint;
  }

  int BodyPart::getChildJoint(void) const noexcept
  {
    return m_childJoint;
  }

  void BodyPart::setChildJoint(const int childJoint) noexcept
  {
    m_childJoint = childJoint;
  }

  bool BodyPart::getIsOccluded(void) const noexcept
  {
    return m_isOccluded;
  }

  void BodyPart::setIsOccluded(const bool isOccluded) noexcept
  {
    m_isOccluded = isOccluded;
  }

  float BodyPart::getSearchRadius(void) const noexcept
  {
    return m_searchRadius;
  }

  void BodyPart::setSearchRadius(const float searchRadius) noexcept
  {
    m_searchRadius = searchRadius;
  }

  float BodyPart::getExpectedDistance(void) const noexcept
  {
    return m_expectedDistance;
  }

  void BodyPart::setExpectedDistance(const float expectedDistance) noexcept
  {
    m_expectedDistance = expectedDistance;
  }

  BodyPart& BodyPart::operator=(const BodyPart& bodyPart) noexcept
  {
    if (&bodyPart == this) 
      return *this;

    m_partID = bodyPart.m_partID;
    m_partName = bodyPart.m_partName;
    m_parentJoint = bodyPart.m_parentJoint;
    m_childJoint = bodyPart.m_childJoint;
    m_isOccluded = bodyPart.m_isOccluded;
    m_expectedDistance = bodyPart.m_expectedDistance;
    m_partPolygon = bodyPart.m_partPolygon;
    m_lwRatio = bodyPart.m_lwRatio;
    m_relativeLength = bodyPart.m_relativeLength;
    m_searchRadius = bodyPart.m_searchRadius;
    m_rotationSearchRange = bodyPart.m_rotationSearchRange;
    return *this;
  }

  BodyPart& BodyPart::operator=(BodyPart&& bodyPart) noexcept
  {
    m_partID = std::move(bodyPart.m_partID);
    std::swap(m_partName, bodyPart.m_partName);
    m_parentJoint = std::move(bodyPart.m_parentJoint);
    m_childJoint = std::move(bodyPart.m_childJoint);
    m_isOccluded = std::move(bodyPart.m_isOccluded);
    m_expectedDistance = std::move(bodyPart.m_expectedDistance);
    std::swap(m_partPolygon, bodyPart.m_partPolygon);
    m_lwRatio = std::move(bodyPart.m_lwRatio);
    m_relativeLength = std::move(bodyPart.m_relativeLength);
    m_searchRadius = std::move(bodyPart.m_searchRadius);
    m_rotationSearchRange = std::move(bodyPart.m_rotationSearchRange);
    return *this;
  }

  bool BodyPart::operator==(const BodyPart &bodyPart) const noexcept
  {
    return this->getPartID() == bodyPart.getPartID();
  }

  bool BodyPart::operator!=(const BodyPart &bodyPart) const noexcept
  {
    return !(*this == bodyPart);
  }

  std::ostream& operator<<(std::ostream& stream, 
    const BodyPart &bodyPart) noexcept
  {
    return stream << std::to_string(bodyPart.getPartID());
  }

  POSERECT <cv::Point2f> BodyPart::getPartPolygon(void) const noexcept
  {
    return m_partPolygon;
  }

  void BodyPart::setPartPolygon(const POSERECT <cv::Point2f> &partPolygon) 
    noexcept
  {
    m_partPolygon = partPolygon;
  }

  float BodyPart::getLWRatio(void) const noexcept
  {
    return m_lwRatio;
  }

  void BodyPart::setLWRatio(const float lwRatio) noexcept
  {
    m_lwRatio = lwRatio;
  }

  float BodyPart::getRelativeLength(void) const noexcept
  {
    return m_relativeLength;
  }

  void BodyPart::setRelativeLength(const float relativeLength) noexcept
  {
    m_relativeLength = relativeLength;
  }

  float BodyPart::getRotationSearchRange(void) const noexcept
  {
    return m_rotationSearchRange;
  }

  void BodyPart::setRotationSearchRange(const float rotationAngle) noexcept
  {
    m_rotationSearchRange = rotationAngle;
  }

  float BodyPart::getBoneLength(const cv::Point2f &begin, 
    const cv::Point2f &end) noexcept
  {
    return (begin == end) ? 1.0f : 
      static_cast<float>(sqrt(spelHelper::distSquared(begin, end)));
  }

  float BodyPart::getBoneLength(const BodyJoint & parent, 
    const BodyJoint & child) noexcept
  {
    return BodyPart::getBoneLength(parent.getImageLocation(), child.getImageLocation());
  }

  float BodyPart::getBoneWidth(const float length, 
    const BodyPart &bodyPart) noexcept
  {
    auto ratio = bodyPart.getLWRatio();
    if (ratio == 0.0f)
      return 0.0f;
    return length / ratio;
  }

  float BodyPart::getBoneWidth(const float length) const noexcept
  {
    auto ratio = getLWRatio();
    if (ratio == 0.0f)
      return 0.0f;
    return length / ratio;
  }

  float BodyPart::getBoneWidth(const cv::Point2f & begin, 
    const cv::Point2f & end, const BodyPart & bodyPart) noexcept
  {
    auto ratio = bodyPart.getLWRatio();
    if (ratio == 0.0f)
      return 0.0f;
    return bodyPart.getBoneLength(begin, end) / ratio;
  }

  float BodyPart::getBoneWidth(const BodyJoint & parent, 
    const BodyJoint & child, const BodyPart & bodyPart) noexcept
  {
    return bodyPart.getBoneWidth(parent, child);
  }

  float BodyPart::getBoneWidth(const cv::Point2f & begin,
    const cv::Point2f & end) const noexcept
  {
    auto ratio = getLWRatio();
    if (ratio == 0.0f)
      return 0.0f;
    return getBoneLength(begin, end) / ratio;
  }

  float BodyPart::getBoneWidth(const BodyJoint & parent, 
    const BodyJoint & child) const noexcept
  {
    return getBoneWidth(parent.getImageLocation(), child.getImageLocation());
  }

  POSERECT<cv::Point2f> BodyPart::getBodyPartRect(const BodyPart & bodyPart,
    const cv::Point2f & parent, const cv::Point2f & child) noexcept
  {
    return bodyPart.getBodyPartRect(bodyPart, parent, child, cv::Size(0, 0));
  }

  POSERECT<cv::Point2f> BodyPart::getBodyPartRect(const cv::Point2f & parent,
    const cv::Point2f & child) const noexcept
  {
    return BodyPart::getBodyPartRect(*this, parent, child);
  }

  POSERECT<cv::Point2f> BodyPart::getBodyPartRect(const BodyPart & bodyPart,
    const BodyJoint & parent, const BodyJoint & child) noexcept
  {
    return bodyPart.getBodyPartRect(parent, child);
  }

  POSERECT<cv::Point2f> BodyPart::getBodyPartRect(const BodyJoint & parent,
    const BodyJoint & child) const noexcept
  {
    return getBodyPartRect(parent.getImageLocation(), 
      child.getImageLocation());
  }

  POSERECT <cv::Point2f> BodyPart::getBodyPartRect(const BodyPart &bodyPart,
    const cv::Point2f &j0, const cv::Point2f &j1, const cv::Size &blockSize)
    noexcept
  {
    auto boxCenter = j0 * 0.5 + j1 * 0.5;
    auto boneLength = getBoneLength(j0, j1);
    if (blockSize.width > 0)
    {
      boneLength = round(boneLength);
      if (boneLength < blockSize.width)
        boneLength = static_cast <float> (blockSize.width);
      else if (static_cast<int>(boneLength) % blockSize.width != 0)
        boneLength = boneLength + blockSize.width - 
        (static_cast<int>(boneLength) % blockSize.width);
    }
    auto boxWidth = getBoneWidth(boneLength, bodyPart);
    if (blockSize.height > 0)
    {
      boxWidth = round(boxWidth);
      if (boxWidth < blockSize.height)
        boxWidth = static_cast <float> (blockSize.height);
      else if (static_cast<int>(boxWidth) % blockSize.height != 0)
        boxWidth = boxWidth + blockSize.height - 
        (static_cast<int>(boxWidth) % blockSize.height);
    }
    auto angle = spelHelper::angle2D(1.0f, 0.0f, j1.x - j0.x, j1.y - j0.y) *
      (180.0f / static_cast<float>(M_PI));
    auto c1 = cv::Point2f(0.f, 0.5f * boxWidth);
    auto c2 = cv::Point2f(boneLength, 0.5f * boxWidth);
    auto c3 = cv::Point2f(boneLength, -0.5f * boxWidth);
    auto c4 = cv::Point2f(0.f, -0.5f * boxWidth);

    c1 = spelHelper::rotatePoint2D(c1, cv::Point2f(0, 0), angle);
    c2 = spelHelper::rotatePoint2D(c2, cv::Point2f(0, 0), angle);
    c3 = spelHelper::rotatePoint2D(c3, cv::Point2f(0, 0), angle);
    c4 = spelHelper::rotatePoint2D(c4, cv::Point2f(0, 0), angle);

    auto polyCenter = 0.25*c1 + 0.25*c2 + 0.25*c3 + 0.25*c4;

    c1 = c1 - polyCenter + boxCenter;
    c2 = c2 - polyCenter + boxCenter;
    c3 = c3 - polyCenter + boxCenter;
    c4 = c4 - polyCenter + boxCenter;

    return POSERECT <cv::Point2f>(c1, c2, c3, c4);
  }

  POSERECT<cv::Point2f> BodyPart::getBodyPartRect(const cv::Point2f & parent,
    const cv::Point2f & child, const cv::Size & blockSize) const noexcept
  {
    return BodyPart::getBodyPartRect(*this, parent, child, blockSize);
  }

  POSERECT<cv::Point2f> BodyPart::getBodyPartRect(const BodyPart & bodyPart,
    const BodyJoint & parent, BodyJoint & child, const cv::Size & blockSize) 
    noexcept
  {
    return bodyPart.getBodyPartRect(parent, child, blockSize);
  }

  POSERECT<cv::Point2f> BodyPart::getBodyPartRect(const BodyJoint & parent,
    const BodyJoint & child, const cv::Size & blockSize) const noexcept
  {
    return getBodyPartRect(parent.getImageLocation(), child.getImageLocation(), blockSize);
  }
}
