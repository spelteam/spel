#ifndef _LIBPOSE_FRAME_HPP_
#define _LIBPOSE_FRAME_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

#include "skeleton.hpp"
#include "spelObject.hpp"

namespace SPEL
{
  enum FRAMETYPE
  {
    UNDEFINED = -1,
    KEYFRAME = 0,
    LOCKFRAME = 1,
    INTERPOLATIONFRAME = 2
  };

  class Frame
  {
  public:
    Frame(void) noexcept;
    Frame(FRAMETYPE _frametype) noexcept;
    virtual ~Frame(void) noexcept;
    virtual std::vector <cv::Point2f> getPartPolygon(int partID) const noexcept;
    virtual int getID(void) const noexcept;
    virtual void setID(int _id) noexcept;
    virtual cv::Mat getImage(void) const noexcept;
    virtual void setImage(const cv::Mat &_image);
    virtual cv::Mat getMask(void) const noexcept;
    virtual void setMask(const cv::Mat &_mask);
    virtual Skeleton getSkeleton(void) const noexcept;
    //TODO: [!]Refactor getters for direct access
    virtual Skeleton* getSkeletonPtr() noexcept;
    virtual void shiftSkeleton2D(cv::Point2f shift) noexcept;
    virtual void setSkeleton(const Skeleton &_skeleton) noexcept;
    virtual cv::Point2f getGroundPoint(void) const noexcept;
    virtual void setGroundPoint(cv::Point2f _groundPoint) noexcept;
    virtual FRAMETYPE getFrametype(void) const noexcept;
    virtual int getParentFrameID(void) const noexcept;
    virtual void setParentFrameID(int _parentFrameID) noexcept;
    virtual float Resize(uint32_t maxHeight) noexcept;
    virtual Frame *clone(Frame *dest) const noexcept;
    virtual cv::Size getFrameSize(void) const noexcept;
    virtual cv::Size getImageSize(void) const noexcept;
    virtual cv::Size getMaskSize(void) const noexcept;
    static bool FramePointerComparer(Frame *frame1, Frame *frame2) noexcept;
  private:
    int id = -1;
    cv::Mat image;
    cv::Mat mask;
    Skeleton skeleton;
    cv::Point2f groundPoint = cv::Point2f(0.0, 0.0);
    int parentFrameID = -1; //the ID of the frame this lockframe was derived from
    cv::Size imageSize = cv::Size(-1, -1);
    cv::Size maskSize = cv::Size(-1, -1);
    FRAMETYPE frametype = UNDEFINED;
  };
}
#endif  // _LIBPOSE_FRAME_HPP_
