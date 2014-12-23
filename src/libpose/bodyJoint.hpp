#ifndef _BODYJOINT_HPP_
#define _BODYJOINT_HPP_

#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// Objects of this class used as elements for building a skeleton model
// See Skeleton.hpp for more info
class BodyJoint
{
  public:
    BodyJoint(void);
    BodyJoint(int id, string name, Point2f imgLoc, Point3f spaceLoc = {0.0, 0.0, 0.0}, bool depth = false);
    bool operator==(const BodyJoint &bj) const; // comparsion by unique index
    bool operator!=(const BodyJoint &bj) const; // comparsion by address
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
    int limbID; // identifier, must be unique within the limits of class
    string jointName; // the object name, respectively to a place in a skeleton model 
    Point2f imageLocation; // coordinates  of the joint on surface of the frame
    Point3f spaceLocation; // it is expansion of the model for use in 3D space
    bool depthSign; // can be used as a flag to switch between 2D/3D, or indicate presence of several plans in the frame
};

#endif  //_BODYJOINT_HPP_

