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

  cv::Mat Frame::getImage(void)
  {
    if (m_image.empty())
    {
      if (m_imagePath.empty())
      {
        std::stringstream ss;
        ss << "Could not load image: the path is not set.";
        throw std::logic_error(ss.str());
      }
      LoadImage();
    }
    return m_image;
  }

  void Frame::setImage(const cv::Mat &image)
  {
    // prevent image from unloading
    m_imagePath = "";
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

  cv::Mat Frame::getMask(void)
  {
    if (m_mask.empty())
    {
      if (m_maskPath.empty())
      {
        std::stringstream ss;
        ss << "Could not load mask: the path is not set.";
        throw std::logic_error(ss.str());
      }
      LoadMask();
    }
    return m_mask;
  }

  void Frame::setMask(const cv::Mat &mask)
  {
    // prevent mask from unloading
    m_imagePath = "";
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

  float Frame::Resize(uint32_t maxHeight)
  {
    if (m_image.rows != m_mask.rows || m_image.cols != m_mask.cols)
    {
      std::stringstream ss;
      ss << "Image with size [" << m_image.rows << "][" <<
        m_image.cols <<
        "] can not be resized because mask with different size [" <<
        m_mask.rows << "][" << m_mask.cols << "] was loaded";
      throw std::logic_error(ss.str());
    }
    if (m_image.rows == static_cast<int>(maxHeight))
      return 0.0f;

    auto factor = static_cast<float>(maxHeight) /
      static_cast<float>(m_image.rows);

    cv::Mat newImage;
    resize(m_image, newImage, cvSize(static_cast<int>(
      m_image.cols * factor), static_cast<int>(m_image.rows * factor)));
    m_image.release();
    m_image = newImage.clone();

    cv::Mat newMask;
    auto factorMask = static_cast<float>(maxHeight) /
      static_cast<float>(m_mask.rows);
    resize(m_mask, newMask, cvSize(static_cast<int>(m_mask.cols *
      factorMask), static_cast<int>(m_mask.rows * factorMask)));
    m_mask.release();
    m_mask = newMask.clone();

    Scale(factor);

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

  void Frame::Scale(const float factor) noexcept
  {
    if (m_scaleFactor != 0.0f)
      m_scaleFactor *= factor;
    else
      m_scaleFactor = factor;

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

  void Frame::AdjustScale(void) noexcept
  {
    if (m_scaleFactor != 0.0f)
      Scale(1.0f / m_scaleFactor);
  }

  std::string Frame::GetImagePath(void) const noexcept
  {
    return m_imagePath;
  }

  void Frame::SetImagePath(const std::string & path) noexcept
  {
    m_imagePath = path;
  }

  std::string Frame::GetMaskPath(void) const noexcept
  {
    return m_maskPath;
  }

  void Frame::SetMaskPath(const std::string & path) noexcept
  {
    m_maskPath = path;
  }

  void Frame::LoadAll(void)
  {
    LoadImage();
    LoadMask();
  }

  void Frame::LoadAll(const std::string & imagePath, const std::string & maskPath)
  {
    LoadImage(imagePath);
    LoadMask(maskPath);
  }

  void Frame::LoadImage(void)
  {
    LoadImage(m_imagePath);
  }

  void Frame::LoadImage(const std::string & path)
  {
    cv::Mat image = cv::imread(path, CV_LOAD_IMAGE_COLOR);
    if (!image.data)
    {
      std::stringstream ss;
      ss << "Could not load image " << path;
      throw std::logic_error(ss.str());
    }
    setImage(image);
  }

  void Frame::LoadMask(void)
  {
    LoadMask(m_maskPath);
  }

  void Frame::LoadMask(const std::string & path)
  {
    cv::Mat mask = cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);
    if (!mask.data)
    {
      std::stringstream ss;
      ss << "Could not load mask " << path;
      throw std::logic_error(ss.str());
    }
    setMask(mask);
  }

  bool Frame::UnloadAll(const bool force) noexcept
  {
    return true && UnloadImage(force) && UnloadMask(force);
  }

  bool Frame::UnloadImage(const bool force) noexcept
  {
    if (m_imagePath.empty() && !force)
      return false;
    m_image.release();
    return true;
  }

  bool Frame::UnloadMask(const bool force) noexcept
  {
    if (m_maskPath.empty() && !force)
      return false;
    m_mask.release();
    return true;
  }

  FRAMETYPE Frame::getFrametype(void) const noexcept
  {
    return m_frametype;
  }

}
