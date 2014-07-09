#ifndef _BODYJOINT_HPP_
#define _BODYJOINT_HPP_

#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class BodyJoint
{
  public:
    BodyJoint(void);
    BodyJoint(int id, string name, Point2f imgLoc);
// get and set
    int getLimbID(void);
    void setLimbID(int _limbID);
    string getJointName(void);
    void setJointName(string _jointName);
    Point2f getImageLocation(void);
    void setImageLocation(Point2f _imageLocation);
    Point3f getSpaceLocation(void);
    void setSpaceLocation(Point3f _spaceLocation);
    bool getDepthSign(void);
    void setDepthSign(bool _depthSign);
  private:
    int limbID;
    string jointName;
    Point2f imageLocation;
    Point3f spaceLocation;
    bool depthSign;
};

#endif  // _BODYJOINT_HPP_

