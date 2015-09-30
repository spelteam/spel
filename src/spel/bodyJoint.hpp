#ifndef _BODYJOINT_HPP_
#define _BODYJOINT_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <string>

// OpenCV
#include <opencv2/opencv.hpp>

namespace SPEL
{
  /// Objects of this class used as elements for building a skeleton model
  /// See [[Skeleton]].hpp for more info

  class BodyJoint
  {
  public:
    BodyJoint(void) noexcept;
    BodyJoint(const BodyJoint& bodyJoint) noexcept;
    BodyJoint(BodyJoint&& bodyJoint) noexcept;
    BodyJoint(int id, std::string name, cv::Point2f imgLoc, cv::Point3f spaceLoc = { 0.0, 0.0, 0.0 }, bool depth = false) noexcept;
    virtual ~BodyJoint(void) noexcept;
    virtual BodyJoint& operator=(const BodyJoint& bodyJoint) noexcept;
    virtual BodyJoint& operator=(BodyJoint&& bodyJoint) noexcept;
    /// comparsion by unique index
    virtual bool operator==(const BodyJoint &bj) const noexcept;
    /// comparsion by address
    virtual bool operator!=(const BodyJoint &bj) const noexcept;
    // All these functions just give access to this object fields
    virtual int getLimbID(void) const noexcept;
    virtual void setLimbID(int _limbID) noexcept;
    virtual std::string getJointName(void) const noexcept;
    virtual void setJointName(std::string _jointName) noexcept;
    virtual cv::Point2f getImageLocation(void) const noexcept;
    virtual void setImageLocation(cv::Point2f _imageLocation) noexcept;
    virtual cv::Point3f getSpaceLocation(void) const noexcept;
    virtual void setSpaceLocation(cv::Point3f _spaceLocation) noexcept;
    virtual bool getDepthSign(void) const noexcept;
    virtual void setDepthSign(bool _depthSign) noexcept;
  private:
    /// identifier, must be unique within the limits of class
    int limbID;
    /// the object name, respectively to a place in a skeleton model 
    std::string jointName;
    /// coordinates  of the joint on surface of the frame
    cv::Point2f imageLocation;
    /// expansion of the model for use in 3D space
    cv::Point3f spaceLocation;
    /// can be used as a flag to switch between 2D/3D, or indicate presence of several plans in the frame
    bool depthSign;
  };
}
#endif  //_BODYJOINT_HPP_
