#ifndef _LIBPOSE_FRAME_HPP_
#define _LIBPOSE_FRAME_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include "skeleton.hpp"

using namespace std;
using namespace cv;

enum FRAMETYPE
{
  KEYFRAME = 0x00,
  LOCKFRAME = 0x01,
  INTERPOLATIONFRAME = 0x02
};

class Frame
{
  public:
    Frame(void);
    virtual ~Frame(void);
    virtual vector <Point2f> getPartPolygon(int partID) = 0;
    int getID(void);
    void setID(int _id); 
    Mat getImage(void);
    void setImage(Mat _image);
    Mat getMask(void);
    void setMask(Mat _mask);
    Skeleton getSkeleton(void);
    void setSkeleton(Skeleton _skeleton);
    Point2f getGroundPoint(void);
    void setGroundPoint(Point2f _groundPoint);
    virtual FRAMETYPE getFrametype(void) = 0;
  private:
    int id;
    Mat image;
    Mat mask;
    Skeleton skeleton;
    Point2f groundPoint;
};

#endif  // _LIBPOSE_FRAME_HPP_

