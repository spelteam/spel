#include "bodyJoint.hpp"

BodyJoint::BodyJoint(void)
{
}

BodyJoint::BodyJoint(int id, string name, Point2f imgLoc)
{
  setLimbID(id);
  setJointName(name);
  setImageLocation(imgLoc);
}

int BodyJoint::getLimbID(void)
{
  return limbID;
}

void BodyJoint::setLimbID(int _limbID)
{
  limbID = _limbID;
}

string BodyJoint::getJointName(void)
{
  return jointName;
}

void BodyJoint::setJointName(string _jointName)
{
  jointName = _jointName;
}

Point2f BodyJoint::getImageLocation(void)
{
  return imageLocation;
}

void BodyJoint::setImageLocation(Point2f _imageLocation)
{
  imageLocation = _imageLocation;
}

Point3f BodyJoint::getSpaceLocation(void)
{
  return spaceLocation;
}

void BodyJoint::setSpaceLocation(Point3f _spaceLocation)
{
  spaceLocation = _spaceLocation;
}

bool BodyJoint::getDepthSign(void)
{
  return depthSign;
}

void BodyJoint::setDepthSign(bool _depthSign)
{
  depthSign = _depthSign;
}

