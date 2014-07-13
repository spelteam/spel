#include "skeleton.hpp"

Skeleton::Skeleton(void)
{
}

Skeleton &Skeleton::operator=(const Skeleton &s)
{
  //this->setName() =   
}

bool Skeleton::operator==(const Skeleton &s) const
{
  return equal(this->partTree.begin(), this->partTree.end(), s.partTree.begin());
}

bool Skeleton::operator!=(const Skeleton &s) const
{
  return !(*this == s);
}
