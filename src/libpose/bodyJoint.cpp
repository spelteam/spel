#include "bodyJoint.hpp"

BodyJoint::BodyJoint(void)
{
  setLimbID(0);
  setJointName("");
  setImageLocation({0.0, 0.0});
  setSpaceLocation({0.0, 0.0, 0.0});
  setDepthSign(false);
}

BodyJoint::BodyJoint(int id, string name, Point2f imgLoc, Point3f spaceLoc, bool depth)
{
  setLimbID(id);
  setJointName(name);
  setImageLocation(imgLoc);
  setSpaceLocation(spaceLoc);
  setDepthSign(depth);
}

int BodyJoint::getLimbID(void) const
{
  return limbID;
}

void BodyJoint::setLimbID(int _limbID)
{
  limbID = _limbID;
}

string BodyJoint::getJointName(void) const
{
  return jointName;
}

void BodyJoint::setJointName(string _jointName)
{
  jointName = _jointName;
}

Point2f BodyJoint::getImageLocation(void) const
{
  return imageLocation;
}

void BodyJoint::setImageLocation(Point2f _imageLocation)
{
  imageLocation = _imageLocation;
}

Point3f BodyJoint::getSpaceLocation(void) const
{
  return spaceLocation;
}

void BodyJoint::setSpaceLocation(Point3f _spaceLocation)
{
  spaceLocation = _spaceLocation;
}

bool BodyJoint::getDepthSign(void) const
{
  return depthSign;
}

void BodyJoint::setDepthSign(bool _depthSign)
{
  depthSign = _depthSign;
}

