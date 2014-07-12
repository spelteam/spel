#ifndef _BODYPART_HPP_
#define _BODYPART_HPP_

#include <string>
#include "bodyJoint.hpp"

using namespace std;

class BodyPart
{
  public:
    BodyPart(void);
    BodyPart(int id, string name, BodyJoint *pJoint, BodyJoint *cJoint, bool isOcc = false, float spaceLen = 0);
// get and set
    int getPartID(void);
    void setPartID(int _partID);
    string getPartName(void);
    void setPartName(string _partName);
    BodyJoint *getParentJoint(void);
    void setParentJoint(BodyJoint *_parentJoint);
    BodyJoint *getChildJoint(void);
    void setChildJoint(BodyJoint *_childJoint);
    bool getIsOccluded(void);
    void setIsOccluded(bool _isOccluded);
    float getSpaceLength(void);
    void setSpaceLength(float _spaceLength);
  private:
    int partID;
    string partName;
    BodyJoint *parentJoint;
    BodyJoint *childJoint;
    bool isOccluded;
    float spaceLength;
};

#endif  // _BODYPART_HPP_

