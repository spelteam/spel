#ifndef _BODYPART_HPP_
#define _BODYPART_HPP_

#include <string>
#include "bodyJoint.hpp"
#include "poseHelper.hpp"

using namespace std;

/// Objects of this class used as elements for building a skeleton model
/// See [[Skeleton]].hpp for more info

class BodyPart
{
  public:
    BodyPart(void);
    BodyPart( const BodyPart& bodyPart );
    BodyPart( BodyPart&& bodyPart );
    BodyPart(int id, string name, int pJoint, int cJoint, bool isOcc = false, float spaceLen = 0);
    BodyPart& operator=( const BodyPart& bodyPart );
    BodyPart& operator=( BodyPart&& bodyPart );
/// comparsion by unique index
    bool operator==(const BodyPart &bp) const;
/// comparsion by address
    bool operator!=(const BodyPart &bp) const;
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
    float getExpectedDistance(void) const;
    void setExpectedDistance(float _expectedDistance);
    float getSearchRadius(void) const;
    void setSearchRadius(float _searchRadius);
    POSERECT <Point2f> getPartPolygon(void);
    void setPartPolygon(POSERECT <Point2f> _partPolygon);
    float getLWRatio(void) const;
    void setLWRatio(float _lwRatio);
    float getRelativeLength(void) const;
    void setRelativeLength(float _relativeLength);
    float getRotationSearchRange(void);
    void setRotationSearchRange(float _rotationAngle);
  private:
/// identifier, must be unique within the limits of class
    int partID;
/// the object name, respectively to a place in a skeleton model 
    string partName;
/// identifier of adjacent overlying joint/node (see BodyJoint.hpp)
    int parentJoint;
/// identifier of adjacent underlying joint/node (see BodyJoint.hpp)
    int childJoint;
/// when "true" - then this body part is overlapped in a frame, used in the skeleton recovery algorithm
    bool isOccluded;
/// expected distance to parent bodypart, as a multiplier of this part's le
    float expectedDistance;
/// rectangle is used as simplified representation of body part 
    POSERECT <Point2f> partPolygon;
/// coefficient of proportionality is used for scaling
    float lwRatio;
/// 3d relative length
    float relativeLength;
    float searchRadius; //search radius for detection of this bodypart
    float rotationSearchRange; //rotation angle range to search through
};

std::ostream& operator<<(std::ostream& os, const BodyPart &bp);

#endif  // _BODYPART_HPP_

