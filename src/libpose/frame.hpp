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
    vector <Point2f> getPartPolygon(int partID);
    int getID(void);
    void setID(int _id); 
    Mat getImage(void);
    void setImage(Mat _image);
    Mat getMask(void);
    void setMask(Mat _mask);
    Skeleton getSkeleton(void);
    //TODO: [!]Refactor getters for direct access
    Skeleton* getSkeletonPtr();
    void shiftSkeleton2D(Point2f shift);
    void setSkeleton(Skeleton _skeleton);
    Point2f getGroundPoint(void);
    void setGroundPoint(Point2f _groundPoint);
    virtual FRAMETYPE getFrametype(void) = 0;
    int getParentFrameID(void);
    void setParentFrameID(int _parentFrameID);
    float Resize(uint32_t maxHeight);
    Frame *clone(Frame *dest);
  private:
    int id;
    Mat image;
    Mat mask;
    Skeleton skeleton;
    Point2f groundPoint;
    int parentFrameID; //the ID of the frame this lockframe was derived from
};

class FramePointerComparer
{
  public:
    bool operator () (Frame *frame1, Frame *frame2)
    {
      return frame1->getID() < frame2->getID();
    }
};

#endif  // _LIBPOSE_FRAME_HPP_

