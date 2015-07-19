#ifndef _LIBPOSE_FRAME_HPP_
#define _LIBPOSE_FRAME_HPP_

// STL
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

#include "skeleton.hpp"

namespace SPEL
{
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
    vector <Point2f> getPartPolygon(int partID) const;
    int getID(void) const;
    void setID(int _id);
    Mat getImage(void) const;
    void setImage(Mat _image);
    Mat getMask(void) const;
    void setMask(Mat _mask);
    Skeleton getSkeleton(void) const;
    //TODO: [!]Refactor getters for direct access
    Skeleton* getSkeletonPtr();
    void shiftSkeleton2D(Point2f shift);
    void setSkeleton(Skeleton _skeleton);
    Point2f getGroundPoint(void) const;
    void setGroundPoint(Point2f _groundPoint);
    virtual FRAMETYPE getFrametype(void) = 0;
    int getParentFrameID(void) const;
    void setParentFrameID(int _parentFrameID);
    float Resize(uint32_t maxHeight);
    Frame *clone(Frame *dest);
    Size getFrameSize(void) const;
    Size getImageSize(void) const;
    Size getMaskSize(void) const;
    static bool FramePointerComparer(Frame *frame1, Frame *frame2);
  private:
    int id;
    Mat image;
    Mat mask;
    Skeleton skeleton;
    Point2f groundPoint;
    int parentFrameID; //the ID of the frame this lockframe was derived from
    Size imageSize = Size(-1, -1);
    Size maskSize = Size(-1, -1);
  };
}
#endif  // _LIBPOSE_FRAME_HPP_

