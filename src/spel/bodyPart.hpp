#ifndef _BODYPART_HPP_
#define _BODYPART_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <string>

#include "bodyJoint.hpp"
#include "spelHelper.hpp"

namespace SPEL
{
  /// Objects of this class used as elements for building a skeleton model
  /// See [[Skeleton]].hpp for more info

  class BodyPart
  {
  public:
    BodyPart(void) noexcept;
    BodyPart(const BodyPart& bodyPart) noexcept;
    BodyPart(BodyPart&& bodyPart) noexcept;
    BodyPart(int id, std::string name, int pJoint, int cJoint, bool isOcc = false, float spaceLen = 0) noexcept;
    virtual ~BodyPart(void) noexcept;
    virtual BodyPart& operator=(const BodyPart& bodyPart) noexcept;
    virtual BodyPart& operator=(BodyPart&& bodyPart) noexcept;
    /// comparsion by unique index
    virtual bool operator==(const BodyPart &bp) const noexcept;
    /// comparsion by address
    virtual bool operator!=(const BodyPart &bp) const noexcept;
    // get and set: All these functions just give access to the object fields
    virtual int getPartID(void) const noexcept;
    virtual void setPartID(int _partID) noexcept;
    virtual std::string getPartName(void) const noexcept;
    virtual void setPartName(std::string _partName) noexcept;
    virtual int getParentJoint(void) const noexcept;
    virtual void setParentJoint(int _parentJoint) noexcept;
    virtual int getChildJoint(void) const noexcept;
    virtual void setChildJoint(int _childJoint) noexcept;
    virtual bool getIsOccluded(void) const noexcept;
    virtual void setIsOccluded(bool _isOccluded) noexcept;
    virtual float getExpectedDistance(void) const noexcept;
    virtual void setExpectedDistance(float _expectedDistance) noexcept;
    virtual POSERECT <cv::Point2f> getPartPolygon(void) const noexcept;
    virtual void setPartPolygon(POSERECT <cv::Point2f> _partPolygon) noexcept;
    virtual float getLWRatio(void) const noexcept;
    virtual void setLWRatio(float _lwRatio) noexcept;
    virtual float getRelativeLength(void) const noexcept;
    virtual void setRelativeLength(float _relativeLength) noexcept;
    //search parameters
    virtual float getSearchRadius(void) const noexcept;
    virtual void setSearchRadius(float _searchRadius) noexcept;
    virtual float getRotationSearchRange(void) const noexcept;
    virtual void setRotationSearchRange(float _rotationAngle) noexcept;
  private:
    /// identifier, must be unique within the limits of class
    int partID;
    /// the object name, respectively to a place in a skeleton model 
    std::string partName;
    /// identifier of adjacent overlying joint/node (see BodyJoint.hpp)
    int parentJoint;
    /// identifier of adjacent underlying joint/node (see BodyJoint.hpp)
    int childJoint;
    /// when "true" - then this body part is overlapped in a frame, used in the skeleton recovery algorithm
    bool isOccluded;
    /// expected distance to parent bodypart, as a multiplier of this part's length
    float expectedDistance;
    /// rectangle is used as simplified representation of body part 
    POSERECT <cv::Point2f> partPolygon;
    /// coefficient of proportionality is used for scaling
    float lwRatio;
    /// 3d relative length
    float relativeLength;
    /// search radius for detection of this bodypart
    float searchRadius;
    /// rotation angle range to search through
    float rotationSearchRange;
  };

  std::ostream& operator<<(std::ostream& os, const BodyPart &bp) noexcept;

}
#endif  // _BODYPART_HPP_
