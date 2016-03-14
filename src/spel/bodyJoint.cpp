#include "bodyJoint.hpp"
#include "spelObject.hpp"

namespace SPEL
{
  BodyJoint::BodyJoint(void) noexcept : BodyJoint(0, "", cv::Point2f(0.0f, 0.0f), cv::Point3f(0.0f, 0.0f, 0.0f), false)
  {    
  }
  
  BodyJoint::BodyJoint(const int id, const std::string &name,
    const cv::Point2f &imageLocation, const cv::Point3f &spaceLocation,
    const bool depth) noexcept
  {
    m_limbID = id;
    m_jointName = name;
    m_imageLocation = imageLocation;
    m_spaceLocation = spaceLocation;
    m_depthSign = depth;
  }

  BodyJoint::BodyJoint(const BodyJoint &bodyJoint) noexcept
    : m_limbID(bodyJoint.m_limbID),
    m_jointName(bodyJoint.m_jointName),
    m_imageLocation(bodyJoint.m_imageLocation),
    m_spaceLocation(bodyJoint.m_spaceLocation),
    m_depthSign(bodyJoint.m_depthSign)
  {
  }

  BodyJoint::BodyJoint(BodyJoint &&bodyJoint) noexcept
    : m_limbID(std::move(bodyJoint.m_limbID)),
    m_jointName(std::move(bodyJoint.m_jointName)),
    m_imageLocation(std::move(bodyJoint.m_imageLocation)),
    m_spaceLocation(std::move(bodyJoint.m_spaceLocation)),
    m_depthSign(std::move(bodyJoint.m_depthSign))
  {
  }

  BodyJoint::BodyJoint(const int id, const std::string & name, 
    const cv::Point2f & imageLocation) noexcept : BodyJoint(id, name, imageLocation, 
      cv::Point3f(0.0f, 0.0f, 0.0f), false)
  {    
  }

  BodyJoint::BodyJoint(const int id, const std::string & name,
    const cv::Point2f & imageLocation, const cv::Point3f & spaceLocation) noexcept : 
  BodyJoint(id, name, imageLocation, spaceLocation, false)
  {    
  }

  BodyJoint::~BodyJoint(void) noexcept
  {
  }

  int BodyJoint::getLimbID(void) const noexcept
  {
    return m_limbID;
  }

  void BodyJoint::setLimbID(const int limbID) noexcept
  {
    m_limbID = limbID;
  }

  std::string BodyJoint::getJointName(void) const noexcept
  {
    return m_jointName;
  }

  void BodyJoint::setJointName(const std::string &jointName) noexcept
  {
    m_jointName = jointName;
  }

  cv::Point2f BodyJoint::getImageLocation(void) const noexcept
  {
    return m_imageLocation;
  }

  void BodyJoint::setImageLocation(const cv::Point2f &imageLocation) noexcept
  {
    m_imageLocation = imageLocation;
  }

  cv::Point3f BodyJoint::getSpaceLocation(void) const noexcept
  {
    return m_spaceLocation;
  }

  void BodyJoint::setSpaceLocation(const cv::Point3f &spaceLocation) noexcept
  {
    m_spaceLocation = spaceLocation;
  }

  bool BodyJoint::getDepthSign(void) const noexcept
  {
    return m_depthSign;
  }

  void BodyJoint::setDepthSign(const bool depthSign) noexcept
  {
    m_depthSign = depthSign;
  }

  BodyJoint& BodyJoint::operator=(const BodyJoint& bodyJoint) noexcept
  {
    if (&bodyJoint == this) 
      return *this;

    m_limbID = bodyJoint.m_limbID;
    m_jointName = bodyJoint.m_jointName;
    m_imageLocation = bodyJoint.m_imageLocation;
    m_spaceLocation = bodyJoint.m_spaceLocation;
    m_depthSign = bodyJoint.m_depthSign;

    return *this;
  }

  BodyJoint& BodyJoint::operator=(BodyJoint&& bodyJoint) noexcept
  {
    m_limbID = std::move(bodyJoint.m_limbID);
    std::swap(m_jointName, bodyJoint.m_jointName);
    std::swap(m_imageLocation, bodyJoint.m_imageLocation);
    std::swap(m_spaceLocation, bodyJoint.m_spaceLocation);
    m_depthSign = std::move(bodyJoint.m_depthSign);

    return *this;
  }

  bool BodyJoint::operator==(const BodyJoint &bodyJoint) const noexcept
  {
    return m_limbID == bodyJoint.getLimbID();
  }

  bool BodyJoint::operator!=(const BodyJoint &bj) const noexcept
  {
    return !(*this == bj);
  }
}
