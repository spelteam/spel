#ifndef _BODYPART_HPP_
#define _BODYPART_HPP_

#include <string>
#include "bodyJoint.hpp"

using namespace std;

class BodyPart
{
  public:
    BodyPart(void);
    BodyPart(int id, string name, BodyJoint *pJoint, BodyJoint *cJoint);
  private:
    int partID;
    string partName;
    BodyJoint *parentJoint;
    BodyJoint *childJoint;
    bool isOccluded;
    float spaceLength;
};

#endif  // _BODYPART_HPP_

