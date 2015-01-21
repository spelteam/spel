#ifndef _BODYPART_HPP_
#define _BODYPART_HPP_

#include <string>
#include "bodyJoint.hpp"
#include "poseHelper.hpp"

using namespace std;

// Objects of this class used as elements for building a skeleton model
// See Skeleton.hpp for more info

class BodyPart
{
  public:
    BodyPart(void);
    BodyPart( const BodyPart& bodyPart );
    BodyPart( BodyPart&& bodyPart );
    BodyPart(int id, string name, int pJoint, int cJoint, bool isOcc = false, float spaceLen = 0);
    BodyPart& operator=( const BodyPart& bodyPart );
    BodyPart& operator=( BodyPart&& bodyPart );
    bool operator==(const BodyPart &bp) const; // comparsion by unique index
    bool operator!=(const BodyPart &bp) const; // comparsion by address
// get and set: All these functions just give access to the object fields
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
    int partID; // identifier, must be unique within the limits of class
    string partName; // the object name, respectively to a place in a skeleton model 
    int parentJoint; // identifier of adjacent overlying joint/node (see BodyJoint.hpp)
    int childJoint; // identifier of adjacent underlying joint/node (see BodyJoint.hpp)
    bool isOccluded; // when "true" - then this body part is overlapped in a frame, used in the skeleton recovery algorithm
    float spaceLength; // the length of body part, distance between neighboring joints
    POSERECT <Point2f> partPolygon; // rectangle is used as simplified representation of body part 
    float lwRatio; // coefficient of proportionality is used for scaling
};

#endif  // _BODYPART_HPP_

