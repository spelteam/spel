#include "skeleton.hpp"
//See Skeleton.hpp for more info

//default constructor
Skeleton::Skeleton(void)
{
}

// constructor with params
Skeleton &Skeleton::operator=(const Skeleton &s)
{
  if (this == &s)
  {
    return *this;
  }
  this->setName(s.getName());
  this->setPartTree(s.getPartTree());
  this->setJointTree(s.getJointTree());
  this->setScale(s.getScale());
  return *this;
}

bool Skeleton::operator==(const Skeleton &s) const
{
  tree <BodyPart> src1 = this->getPartTree();
  tree <BodyPart> src2 = s.getPartTree();
  return equal(src1.begin(), src1.end(), src2.begin());
}

bool Skeleton::operator!=(const Skeleton &s) const
{
  return !(*this == s);
}

string Skeleton::getName(void) const
{
  return name;
}

void Skeleton::setName(string _name)
{
  name = _name;
}

tree <BodyPart> Skeleton::getPartTree(void) const
{
  return partTree;
}

void Skeleton::setPartTree(tree <BodyPart> _partTree)
{
  partTree = _partTree;
}

tree <BodyJoint> Skeleton::getJointTree(void) const
{
  return jointTree;
}

void Skeleton::setJointTree(tree <BodyJoint> _jointTree)
{
  jointTree = _jointTree;
}

float Skeleton::getScale(void) const
{
  return scale;
}

void Skeleton::setScale(float _scale)
{
  scale = _scale;
}

uint32_t Skeleton::getPartTreeCount(void) const
{
  return partTree.size();
}

BodyJoint *Skeleton::getBodyJoint(int jointID) const
{
  BodyJoint *joint = 0;
  for(tree <BodyJoint>::iterator i = jointTree.begin(); i != jointTree.end(); ++i)
  {
    if (i->getLimbID() == jointID)
    {
      joint = &*i;
    }
  }
  return joint;
}
