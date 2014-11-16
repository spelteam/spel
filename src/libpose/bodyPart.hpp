#ifndef _BODYPART_HPP_
#define _BODYPART_HPP_

#include <string>
#include "bodyJoint.hpp"
#include "poseHelper.hpp"

using namespace std;

class BodyPart
{
  public:
    BodyPart(void);
    BodyPart(int id, string name, int pJoint, int cJoint, bool isOcc = false, float spaceLen = 0);
    bool operator==(const BodyPart &bp) const;
    bool operator!=(const BodyPart &bp) const;
// get and set
    int getPartID(void) const;
    void setPartID(int _partID);
    string getPartName(void) const;
    void setPartName(string _partName);
    int getParentJoint(void) const;
    void setParentJoint(int _parentJoint);
    int getChildJoint(void) const;
    void setChildJoint(int _childJoint);
    bool getIsOccluded(void) const;
    void setIsOccluded(bool _isOccluded);
    float getSpaceLength(void) const;
    void setSpaceLength(float _spaceLength);
    POSERECT <Point2f> getPartPolygon(void);
    void setPartPolygon(POSERECT <Point2f> _partPolygon);
    float getLWRatio(void);
    void setLWRatio(float _lwRatio);
  private:
    int partID;
    string partName;
    int parentJoint;
    int childJoint;
    bool isOccluded;
    float spaceLength;
    POSERECT <Point2f> partPolygon;
    float lwRatio;
};

#endif  // _BODYPART_HPP_

