#include "frame.hpp"
#include "spelObject.hpp"

namespace SPEL
{
  Frame::Frame(void) noexcept
  {    
  }

  Frame::Frame(FRAMETYPE frametype) noexcept : Frame()
  {
    m_frametype = frametype;
  }

  Frame::~Frame(void) noexcept
  {
    m_image.release();
    m_mask.release();
  }

  int Frame::getID(void) const noexcept
  {
    return m_id;
  }

  void Frame::setID(int id) noexcept
  {
    m_id = id;
  }

  cv::Mat Frame::getImage(void) const noexcept
  {
    return m_image;
  }

  void Frame::setImage(const cv::Mat &image)
  {
    cv::Size newImageSize(image.cols, image.rows);
    if (maskSize != cv::Size(-1, -1) && maskSize != newImageSize)
    {
      std::stringstream ss;
      ss << "Image with size [" << newImageSize.width << "][" << 
        newImageSize.height << 
        "] can not be loaded because mask with different size [" << 
        maskSize.width << "][" << maskSize.height << "] was loaded";
      throw std::logic_error(ss.str());
    }
    m_image.release();
    m_image = image.clone();
    imageSize = newImageSize;
  }

  cv::Mat Frame::getMask(void) const noexcept
  {
    return m_mask;
  }

  void Frame::setMask(const cv::Mat &mask)
  {
    cv::Size newMaskSize(mask.cols, mask.rows);
    if (imageSize != cv::Size(-1, -1) && imageSize != newMaskSize)
    {
      std::stringstream ss;
      ss << "Mask with size [" << newMaskSize.width << "][" << 
        newMaskSize.height << 
        "] can not be loaded because image with different size [" << 
        imageSize.width << "][" << imageSize.height << "] was loaded";
      throw std::logic_error(ss.str());
    }
    m_mask.release();
    m_mask = mask.clone();
    maskSize = newMaskSize;
  }

  Skeleton Frame::getSkeleton(void) const noexcept
  {
    return m_skeleton;
  }

  Skeleton* Frame::getSkeletonPtr() noexcept
  {
    return &m_skeleton;
  }

  void Frame::setSkeleton(const Skeleton &skeleton) noexcept
  {
    m_skeleton = skeleton;
  }

  cv::Point2f Frame::getGroundPoint(void) const noexcept
  {
    return m_groundPoint;
  }

  void Frame::setGroundPoint(cv::Point2f groundPoint) noexcept
  {
    m_groundPoint = groundPoint;
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

  //shift in 2D and recompute 3D?
  void Frame::shiftSkeleton2D(cv::Point2f point) noexcept 
  {
    auto jointTree = m_skeleton.getJointTree();
    //add point to every joint
    for (auto &i : jointTree)      
      i.setImageLocation(i.getImageLocation() + point);

    m_skeleton.setJointTree(jointTree);
    m_skeleton.infer3D();
  }

  int Frame::getParentFrameID(void) const noexcept
  {
    return m_parentFrameID;
  }

  void Frame::setParentFrameID(int parentFrameID) noexcept
  {
    m_parentFrameID = parentFrameID;
  }

  float Frame::Resize(uint32_t maxHeight) noexcept
  {
    auto factor = static_cast<float>(maxHeight) / 
      static_cast<float>(m_image.rows);
    if (m_image.rows != static_cast<int>(maxHeight))
    {
      cv::Mat newImage;
      resize(m_image, newImage, cvSize(static_cast<int>(
        m_image.cols * factor), static_cast<int>(m_image.rows * factor)));
      m_image.release();
      m_image = newImage.clone();
      auto joints = m_skeleton.getJointTree();
      auto parts = m_skeleton.getPartTree();

      for (auto &j : joints)
      {
        auto location = j.getImageLocation();
        location.x *= factor;
        location.y *= factor;
        j.setImageLocation(location);
      }
      m_skeleton.setJointTree(joints);
      m_skeleton.infer3D();
      //update the search radius to match the new scaling
      for (auto &p : parts)
        p.setSearchRadius(p.getSearchRadius() * factor);

      m_skeleton.setPartTree(parts);
    }
    if (m_mask.rows != static_cast<int>(maxHeight))
    {
      cv::Mat newMask;
      auto factorMask = static_cast<float>(maxHeight) / 
        static_cast<float>(m_mask.rows);
      resize(m_mask, newMask, cvSize(static_cast<int>(m_mask.cols * 
        factorMask), static_cast<int>(m_mask.rows * factorMask)));
      m_mask.release();
      m_mask = newMask.clone();
    }
    return factor;
  }

  Frame *Frame::clone(Frame *dest) const noexcept
  {
    if (dest == nullptr)
      return nullptr;
    dest->setGroundPoint(m_groundPoint);
    dest->setID(m_id);
    dest->setImage(m_image.clone());
    dest->setMask(m_mask.clone());
    dest->setParentFrameID(m_parentFrameID);
    dest->setSkeleton(m_skeleton);
    dest->m_frametype = m_frametype;
    return dest;
  }

  cv::Size Frame::getFrameSize() const noexcept
  {
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
    return m_frametype;
  }

}
