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
    BodyJoint(void);
    BodyJoint(const BodyJoint& bodyJoint);
    BodyJoint(BodyJoint&& bodyJoint);
    BodyJoint(int id, std::string name, cv::Point2f imgLoc, cv::Point3f spaceLoc = { 0.0, 0.0, 0.0 }, bool depth = false);
    virtual ~BodyJoint(void);
    virtual BodyJoint& operator=(const BodyJoint& bodyJoint);
    virtual BodyJoint& operator=(BodyJoint&& bodyJoint);
    /// comparsion by unique index
    virtual bool operator==(const BodyJoint &bj) const;
    /// comparsion by address
    virtual bool operator!=(const BodyJoint &bj) const;
    // All these functions just give access to this object fields
    virtual int getLimbID(void) const;
    virtual void setLimbID(int _limbID);
    virtual std::string getJointName(void) const;
    virtual void setJointName(std::string _jointName);
    virtual cv::Point2f getImageLocation(void) const;
    virtual void setImageLocation(cv::Point2f _imageLocation);
    virtual cv::Point3f getSpaceLocation(void) const;
    virtual void setSpaceLocation(cv::Point3f _spaceLocation);
    virtual bool getDepthSign(void) const;
    virtual void setDepthSign(bool _depthSign);
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
