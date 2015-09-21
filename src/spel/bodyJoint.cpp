#include "bodyJoint.hpp"
// See bodyJoint.hpp and Skeleton.hpp for more info
namespace SPEL
{
  //default constructor
  BodyJoint::BodyJoint(void) noexcept
  {
    limbID = 0;
    jointName = "";
    imageLocation = { 0.0, 0.0 };
    spaceLocation = { 0.0, 0.0, 0.0 };
    depthSign = false;
  }

  // constructor with params
  BodyJoint::BodyJoint(int id, std::string name, cv::Point2f imgLoc, cv::Point3f spaceLoc, bool depth) noexcept
  {
    limbID = id;
    jointName = name;
    imageLocation = imgLoc;
    spaceLocation = spaceLoc;
    depthSign = depth;
  }

  //copy constructor
  BodyJoint::BodyJoint(const BodyJoint &bodyJoint) noexcept
    : limbID(bodyJoint.limbID),
    jointName(bodyJoint.jointName),
    imageLocation(bodyJoint.imageLocation),
    spaceLocation(bodyJoint.spaceLocation),
    depthSign(bodyJoint.depthSign)
  {
  }

  //move constructor
  BodyJoint::BodyJoint(BodyJoint &&bodyJoint) noexcept
    : limbID(std::move(bodyJoint.limbID)),
    jointName(std::move(bodyJoint.jointName)),
    imageLocation(std::move(bodyJoint.imageLocation)),
    spaceLocation(std::move(bodyJoint.spaceLocation)),
    depthSign(std::move(bodyJoint.depthSign))
  {
  }

  BodyJoint::~BodyJoint(void) noexcept
  {
    return;
  }

  int BodyJoint::getLimbID(void) const noexcept
  {
    return limbID;
  }

  void BodyJoint::setLimbID(int _limbID) noexcept
  {
    limbID = _limbID;
  }

  std::string BodyJoint::getJointName(void) const noexcept
  {
    return jointName;
  }

  void BodyJoint::setJointName(std::string _jointName) noexcept
  {
    jointName = _jointName;
  }

  cv::Point2f BodyJoint::getImageLocation(void) const noexcept
  {
    return imageLocation;
  }

  void BodyJoint::setImageLocation(cv::Point2f _imageLocation) noexcept
  {
    imageLocation = _imageLocation;
  }

  cv::Point3f BodyJoint::getSpaceLocation(void) const noexcept
  {
    return spaceLocation;
  }

  void BodyJoint::setSpaceLocation(cv::Point3f _spaceLocation) noexcept
  {
    spaceLocation = _spaceLocation;
  }

  bool BodyJoint::getDepthSign(void) const noexcept
  {
    return depthSign;
  }

  void BodyJoint::setDepthSign(bool _depthSign) noexcept
  {
    depthSign = _depthSign;
  }

  BodyJoint& BodyJoint::operator=(const BodyJoint& bodyJoint) noexcept
  {
    if (&bodyJoint == this) 
      return *this;

    limbID = bodyJoint.limbID;
    jointName = bodyJoint.jointName;
    imageLocation = bodyJoint.imageLocation;
    spaceLocation = bodyJoint.spaceLocation;
    depthSign = bodyJoint.depthSign;

    return *this;
  }

  BodyJoint& BodyJoint::operator=(BodyJoint&& bodyJoint) noexcept
  {
    limbID = std::move(bodyJoint.limbID);
    std::swap(jointName, bodyJoint.jointName);
    std::swap(imageLocation, bodyJoint.imageLocation);
    std::swap(spaceLocation, bodyJoint.spaceLocation);
    depthSign = std::move(bodyJoint.depthSign);

    return *this;
  }

  bool BodyJoint::operator==(const BodyJoint &bj) const noexcept
  {
    return this->getLimbID() == bj.getLimbID();
  }

  bool BodyJoint::operator!=(const BodyJoint &bj) const noexcept
  {
    return !(*this == bj);
  }

}
