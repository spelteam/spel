#include "frame.hpp"

namespace SPEL
{

  Frame::Frame(void)
  {
    
  }

  Frame::Frame(FRAMETYPE _frametype) : Frame()
  {
    frametype = _frametype;
  }

  Frame::~Frame(void)
  {
    image.release();
    mask.release();
  }

  int Frame::getID(void) const
  {
    return id;
  }

  void Frame::setID(int _id)
  {
    id = _id;
  }

  cv::Mat Frame::getImage(void) const
  {
    return image;
  }

  void Frame::setImage(cv::Mat _image)
  {
    cv::Size newImageSize(_image.cols, _image.rows);
    if (maskSize != cv::Size(-1, -1) && maskSize != newImageSize)
    {
      std::stringstream ss;
      ss << "Image with size [" << newImageSize.width << "][" << newImageSize.height << "] can not be loaded because mask with different size [" << maskSize.width << "][" << maskSize.height << "] was loaded";
      throw std::logic_error(ss.str());
    }
    image.release();
    image = _image.clone();
    imageSize = newImageSize;
  }

  cv::Mat Frame::getMask(void) const
  {
    return mask;
  }

  void Frame::setMask(cv::Mat _mask)
  {
    cv::Size newMaskSize(_mask.cols, _mask.rows);
    if (imageSize != cv::Size(-1, -1) && imageSize != newMaskSize)
    {
      std::stringstream ss;
      ss << "Mask with size [" << newMaskSize.width << "][" << newMaskSize.height << "] can not be loaded because image with different size [" << imageSize.width << "][" << imageSize.height << "] was loaded";
      throw std::logic_error(ss.str());
    }
    mask.release();
    mask = _mask.clone();
    maskSize = newMaskSize;
  }

  Skeleton Frame::getSkeleton(void) const
  {
    return skeleton;
  }

  Skeleton* Frame::getSkeletonPtr()
  {
    return &skeleton;
  }

  void Frame::setSkeleton(Skeleton _skeleton)
  {
    skeleton = _skeleton;
  }

  cv::Point2f Frame::getGroundPoint(void) const
  {
    return groundPoint;
  }

  void Frame::setGroundPoint(cv::Point2f _groundPoint)
  {
    groundPoint = _groundPoint;
  }

  std::vector <cv::Point2f> Frame::getPartPolygon(int partID) const
  {
    tree <BodyPart> partTree = getSkeleton().getPartTree();
    tree <BodyPart>::iterator i;
    for (i = partTree.begin(); i != partTree.end(); i++)
    {
      if (i->getPartID() == partID)
      {
        return i->getPartPolygon().asVector();
      }
    }
    return std::vector <cv::Point2f>();
  }

  void Frame::shiftSkeleton2D(cv::Point2f point) //shift in 2D and recompute 3D?
  {
    tree <BodyJoint> jointTree = skeleton.getJointTree();
    for (tree <BodyJoint>::iterator i = jointTree.begin(); i != jointTree.end(); ++i)
    {
      //add point to every joint
      cv::Point2f prevLoc = i->getImageLocation();
      cv::Point2f nextLoc = prevLoc + point;
      i->setImageLocation(nextLoc);
    }
    skeleton.setJointTree(jointTree);
    skeleton.infer3D();
  }

  int Frame::getParentFrameID(void) const
  {
    return parentFrameID;
  }

  void Frame::setParentFrameID(int _parentFrameID)
  {
    parentFrameID = _parentFrameID;
  }

  float Frame::Resize(uint32_t maxHeight)
  {
    float factor = (float)maxHeight / (float)image.rows;
    if (image.rows != maxHeight)
    {
      cv::Mat newImage;
      resize(image, newImage, cvSize(image.cols * factor, image.rows * factor));
      image.release();
      image = newImage.clone();
      tree <BodyJoint> joints = skeleton.getJointTree();
      tree <BodyPart> parts = skeleton.getPartTree();

      for (tree <BodyJoint>::iterator j = joints.begin(); j != joints.end(); ++j)
      {
        cv::Point2f location = j->getImageLocation();
        location.x *= factor;
        location.y *= factor;
        j->setImageLocation(location);
      }
      skeleton.setJointTree(joints);
      skeleton.infer3D();
      for (tree<BodyPart>::iterator p = parts.begin(); p != parts.end(); ++p)
      {
        //update the search radius to match the new scaling
        p->setSearchRadius(p->getSearchRadius()*factor);
      }
      skeleton.setPartTree(parts);
    }
    if (mask.rows != maxHeight)
    {
      cv::Mat newMask;
      float factor = (float)maxHeight / (float)mask.rows;
      resize(mask, newMask, cvSize(mask.cols * factor, mask.rows * factor));
      mask.release();
      mask = newMask.clone();
    }
    return factor;
  }

  Frame *Frame::clone(Frame *dest)
  {
    dest->setGroundPoint(groundPoint);
    dest->setID(id);
    dest->setImage(image.clone());
    dest->setMask(mask.clone());
    dest->setParentFrameID(parentFrameID);
    dest->setSkeleton(skeleton);
    dest->frametype = frametype;
    return dest;
  }

  cv::Size Frame::getFrameSize() const
  {
    if (imageSize != maskSize)
    {
      std::stringstream ss;
      ss << "Image [" << imageSize.width << "][" << imageSize.height << "] and mask [" << maskSize.width << "][" << maskSize.height << "] are loaded";
      throw std::logic_error(ss.str());
    }
    return imageSize;
  }

  cv::Size Frame::getImageSize(void) const
  {
    return imageSize;
  }

  cv::Size Frame::getMaskSize(void) const
  {
    return maskSize;
  }

  bool Frame::FramePointerComparer(Frame *frame1, Frame *frame2)
  {
    return frame1->getID() < frame2->getID();
  }

  FRAMETYPE Frame::getFrametype(void) const
  {
    return frametype;
  }

}
