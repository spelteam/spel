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
  private:
    int limbID;
    string jointName;
    Point2f imageLocation;
    Point3f spaceLocation;
    bool depthSign;
};

#endif  // _BODYJOINT_HPP_

