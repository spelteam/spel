// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com
#include "bodyJoint.hpp"
#include "spelObject.hpp"

namespace SPEL
{
  BodyJoint::BodyJoint(void)  : BodyJoint(0, "", cv::Point2f(0.0f, 0.0f), cv::Point3f(0.0f, 0.0f, 0.0f), false)
  {    
  }
  
  BodyJoint::BodyJoint(const int id, const std::string &name,
    const cv::Point2f &imageLocation, const cv::Point3f &spaceLocation,
    const bool depth) 
  {
    m_limbID = id;
    m_jointName = name;
    m_imageLocation = imageLocation;
    m_spaceLocation = spaceLocation;
    m_depthSign = depth;
  }

  BodyJoint::BodyJoint(const BodyJoint &bodyJoint) 
    : m_limbID(bodyJoint.m_limbID),
    m_jointName(bodyJoint.m_jointName),
    m_imageLocation(bodyJoint.m_imageLocation),
    m_spaceLocation(bodyJoint.m_spaceLocation),
    m_depthSign(bodyJoint.m_depthSign)
  {
  }

  BodyJoint::BodyJoint(BodyJoint &&bodyJoint) 
    : m_limbID(std::move(bodyJoint.m_limbID)),
    m_jointName(std::move(bodyJoint.m_jointName)),
    m_imageLocation(std::move(bodyJoint.m_imageLocation)),
    m_spaceLocation(std::move(bodyJoint.m_spaceLocation)),
    m_depthSign(std::move(bodyJoint.m_depthSign))
  {
  }

  BodyJoint::BodyJoint(const int id, const std::string & name, 
    const cv::Point2f & imageLocation)  : BodyJoint(id, name, imageLocation, 
      cv::Point3f(0.0f, 0.0f, 0.0f), false)
  {    
  }

  BodyJoint::BodyJoint(const int id, const std::string & name,
    const cv::Point2f & imageLocation, const cv::Point3f & spaceLocation)  : 
  BodyJoint(id, name, imageLocation, spaceLocation, false)
  {    
  }

  BodyJoint::~BodyJoint(void) 
  {
  }

  int BodyJoint::getLimbID(void) const 
  {
    return m_limbID;
  }

  void BodyJoint::setLimbID(const int limbID) 
  {
    m_limbID = limbID;
  }

  std::string BodyJoint::getJointName(void) const 
  {
    return m_jointName;
  }

  void BodyJoint::setJointName(const std::string &jointName) 
  {
    m_jointName = jointName;
  }

  cv::Point2f BodyJoint::getImageLocation(void) const 
  {
    return m_imageLocation;
  }

  void BodyJoint::setImageLocation(const cv::Point2f &imageLocation) 
  {
    m_imageLocation = imageLocation;
  }

  cv::Point3f BodyJoint::getSpaceLocation(void) const 
  {
    return m_spaceLocation;
  }

  void BodyJoint::setSpaceLocation(const cv::Point3f &spaceLocation) 
  {
    m_spaceLocation = spaceLocation;
  }

  bool BodyJoint::getDepthSign(void) const 
  {
    return m_depthSign;
  }

  void BodyJoint::setDepthSign(const bool depthSign) 
  {
    m_depthSign = depthSign;
  }

  BodyJoint& BodyJoint::operator=(const BodyJoint& bodyJoint) 
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

  BodyJoint& BodyJoint::operator=(BodyJoint&& bodyJoint) 
  {
    m_limbID = std::move(bodyJoint.m_limbID);
    std::swap(m_jointName, bodyJoint.m_jointName);
    std::swap(m_imageLocation, bodyJoint.m_imageLocation);
    std::swap(m_spaceLocation, bodyJoint.m_spaceLocation);
    m_depthSign = std::move(bodyJoint.m_depthSign);

    return *this;
  }

  bool BodyJoint::operator==(const BodyJoint &bodyJoint) const 
  {
    return m_limbID == bodyJoint.getLimbID();
  }

  bool BodyJoint::operator!=(const BodyJoint &bj) const 
  {
    return !(*this == bj);
  }
}
