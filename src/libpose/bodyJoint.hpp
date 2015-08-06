#ifndef _BODYJOINT_HPP_
#define _BODYJOINT_HPP_

// STL
#include <string>

// OpenCV
#include <opencv2/opencv.hpp>

namespace SPEL
{
  using namespace std;
  using namespace cv;
  /// Objects of this class used as elements for building a skeleton model
  /// See [[Skeleton]].hpp for more info

  class BodyJoint
  {
  public:
    BodyJoint(void);
    BodyJoint(const BodyJoint& bodyJoint);
    BodyJoint(BodyJoint&& bodyJoint);
    BodyJoint(int id, string name, Point2f imgLoc, Point3f spaceLoc = { 0.0, 0.0, 0.0 }, bool depth = false);
    BodyJoint& operator=(const BodyJoint& bodyJoint);
    BodyJoint& operator=(BodyJoint&& bodyJoint);
    /// comparsion by unique index
    bool operator==(const BodyJoint &bj) const;
    /// comparsion by address
    bool operator!=(const BodyJoint &bj) const;
    // All these functions just give access to this object fields
    int getLimbID(void) const;
    void setLimbID(int _limbID);
    string getJointName(void) const;
    void setJointName(string _jointName);
    Point2f getImageLocation(void) const;
    void setImageLocation(Point2f _imageLocation);
    Point3f getSpaceLocation(void) const;
    void setSpaceLocation(Point3f _spaceLocation);
    bool getDepthSign(void) const;
    void setDepthSign(bool _depthSign);
  private:
    /// identifier, must be unique within the limits of class
    int limbID;
    /// the object name, respectively to a place in a skeleton model 
    string jointName;
    /// coordinates  of the joint on surface of the frame
    Point2f imageLocation;
    /// expansion of the model for use in 3D space
    Point3f spaceLocation;
    /// can be used as a flag to switch between 2D/3D, or indicate presence of several plans in the frame
    bool depthSign;
  };
}
#endif  //_BODYJOINT_HPP_

