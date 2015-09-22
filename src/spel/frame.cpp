#include "frame.hpp"

namespace SPEL
{

  Frame::Frame(void) noexcept
  {
    
  }

  Frame::Frame(FRAMETYPE _frametype) noexcept : Frame()
  {
    frametype = _frametype;
  }

  Frame::~Frame(void) noexcept
  {
    image.release();
    mask.release();
  }

  int Frame::getID(void) const noexcept
  {
    return id;
  }

  void Frame::setID(int _id) noexcept
  {
    id = _id;
  }

  cv::Mat Frame::getImage(void) const noexcept
  {
    return image;
  }

  void Frame::setImage(const cv::Mat &_image)
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

  cv::Mat Frame::getMask(void) const noexcept
  {
    return mask;
  }

  void Frame::setMask(const cv::Mat &_mask)
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

  Skeleton Frame::getSkeleton(void) const noexcept
  {
    return skeleton;
  }

  Skeleton* Frame::getSkeletonPtr() noexcept
  {
    return &skeleton;
  }

  void Frame::setSkeleton(const Skeleton &_skeleton) noexcept
  {
    skeleton = _skeleton;
  }

  cv::Point2f Frame::getGroundPoint(void) const noexcept
  {
    return groundPoint;
  }

  void Frame::setGroundPoint(cv::Point2f _groundPoint) noexcept
  {
    groundPoint = _groundPoint;
  }

  std::vector <cv::Point2f> Frame::getPartPolygon(int partID) const noexcept
  {
    auto partTree = getSkeleton().getPartTree();
    for (const auto &i : partTree)
    {
      if (i.getPartID() == partID)
      {
        return i.getPartPolygon().asVector();
      }
    }
    return std::vector <cv::Point2f>();
  }

  void Frame::shiftSkeleton2D(cv::Point2f point) noexcept //shift in 2D and recompute 3D?
  {
    auto jointTree = skeleton.getJointTree();
    //add point to every joint
    for (auto &i : jointTree)      
      i.setImageLocation(i.getImageLocation() + point);

    skeleton.setJointTree(jointTree);
    skeleton.infer3D();
  }

  int Frame::getParentFrameID(void) const noexcept
  {
    return parentFrameID;
  }

  void Frame::setParentFrameID(int _parentFrameID) noexcept
  {
    parentFrameID = _parentFrameID;
  }

  float Frame::Resize(uint32_t maxHeight) noexcept
  {
    auto factor = static_cast<float>(maxHeight) / static_cast<float>(image.rows);
    if (image.rows != maxHeight)
    {
      cv::Mat newImage;
      resize(image, newImage, cvSize(image.cols * factor, image.rows * factor));
      image.release();
      image = newImage.clone();
      auto joints = skeleton.getJointTree();
      auto parts = skeleton.getPartTree();

      for (auto &j : joints)
      {
        auto location = j.getImageLocation();
        location.x *= factor;
        location.y *= factor;
        j.setImageLocation(location);
      }
      skeleton.setJointTree(joints);
      skeleton.infer3D();
      //update the search radius to match the new scaling
      for (auto &p : parts)
        p.setSearchRadius(p.getSearchRadius() * factor);

      skeleton.setPartTree(parts);
    }
    if (mask.rows != maxHeight)
    {
      cv::Mat newMask;
      auto factor = static_cast<float>(maxHeight) / static_cast<float>(mask.rows);
      resize(mask, newMask, cvSize(mask.cols * factor, mask.rows * factor));
      mask.release();
      mask = newMask.clone();
    }
    return factor;
  }

  Frame *Frame::clone(Frame *dest) const noexcept
  {
    if (dest == nullptr)
      return nullptr;
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

  cv::Size Frame::getImageSize(void) const noexcept
  {
    return imageSize;
  }

  cv::Size Frame::getMaskSize(void) const noexcept
  {
    return maskSize;
  }

  bool Frame::FramePointerComparer(Frame *frame1, Frame *frame2) noexcept
  {
    return frame1->getID() < frame2->getID();
  }

  FRAMETYPE Frame::getFrametype(void) const noexcept
  {
    return frametype;
  }

}
