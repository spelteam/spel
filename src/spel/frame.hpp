#ifndef _LIBPOSE_FRAME_HPP_
#define _LIBPOSE_FRAME_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

#include "skeleton.hpp"

namespace SPEL
{  
  /// <summary>
  /// Type of the frame
  /// </summary>
  enum FRAMETYPE
  {
    UNDEFINED = -1,
    KEYFRAME = 0,
    LOCKFRAME = 1,
    INTERPOLATIONFRAME = 2
  };  
  /// <summary>
  /// Class represents the frame.
  /// </summary>
  class Frame
  {
  public:
    /// <summary>
    /// Initializes a new instance of the <see cref="Frame"/> class.
    /// </summary>
    Frame(void) noexcept;
    /// <summary>
    /// Initializes a new instance of the <see cref="Frame"/> class.
    /// </summary>
    /// <param name="frametype">The frametype.</param>
    Frame(FRAMETYPE frametype) noexcept;
    /// <summary>
    /// Finalizes an instance of the <see cref="Frame"/> class.
    /// </summary>
    /// <returns></returns>
    virtual ~Frame(void) noexcept;
    /// <summary>Gets the part polygon.</summary>
    /// <param name="partID">The part identifier.</param>
    /// <returns>The part polygon.</returns>
    std::vector <cv::Point2f> getPartPolygon(int partID) const noexcept;
    /// <summary>Gets the identifier.</summary>
    /// <returns>The identifier.</returns>
    int getID(void) const noexcept;
    /// <summary>Sets the identifier.</summary>
    /// <param name="id">The identifier.</param>
    void setID(int id) noexcept;
    /// <summary>Gets the image.</summary>
    /// <returns>The image.</returns>
    cv::Mat getImage(void) const noexcept;
    /// <summary>Sets the image.</summary>
    /// <param name="image">The image.</param>
    void setImage(const cv::Mat &image);
    /// <summary>Gets the mask.</summary>
    /// <returns>The mask.</returns>
    cv::Mat getMask(void) const noexcept;
    /// <summary>Sets the mask.</summary>
    /// <param name="mask">The mask.</param>
    void setMask(const cv::Mat &mask);
    /// <summary>Gets the skeleton.</summary>
    /// <returns>The skeleton.</returns>
    Skeleton getSkeleton(void) const noexcept;
    //TODO: [!]Refactor getters for direct access
    /// <summary>Gets the pointer to the skeleton.</summary>
    /// <returns>The pointer to the skeleton.</returns>
    Skeleton* getSkeletonPtr() noexcept;
    /// <summary>Shifts the skeleton.</summary>
    /// <param name="shift">The shift.</param>
    /// <returns></returns>
    void shiftSkeleton2D(cv::Point2f shift) noexcept;
    /// <summary>Sets the skeleton.</summary>
    /// <param name="skeleton">The skeleton.</param>
    void setSkeleton(const Skeleton &skeleton) noexcept;
    /// <summary>Gets the ground point.</summary>
    /// <returns>The ground point.</returns>
    cv::Point2f getGroundPoint(void) const noexcept;
    /// <summary>Sets the ground point.</summary>
    /// <param name="groundPoint">The ground point.</param>
    void setGroundPoint(cv::Point2f groundPoint) noexcept;
    /// <summary>Gets the frametype.</summary>
    /// <returns>The frametype.</returns>
    FRAMETYPE getFrametype(void) const noexcept;
    /// <summary>Gets the parent frame identifier.</summary>
    /// <returns>The parent frame identifier.</returns>
    int getParentFrameID(void) const noexcept;
    /// <summary>Sets the parent frame identifier.</summary>
    /// <param name="parentFrameID">The parent frame identifier.</param>
    void setParentFrameID(int parentFrameID) noexcept;
    /// <summary>Resizes the frame.</summary>
    /// <param name="maxHeight">The maximum height.</param>
    /// <returns>Resize coefficient.</returns>
    float Resize(uint32_t maxHeight) noexcept;
    /// <summary>Clones the frame to the destination.</summary>
    /// <param name="dest">The destination.</param>
    /// <returns>The cloned frame.</returns>
    Frame *clone(Frame *dest) const noexcept;
    /// <summary>Gets the size of the frame.</summary>
    /// <returns>The size of the frame.</returns>
    cv::Size getFrameSize(void) const noexcept;
    /// <summary>Gets the size of the image.</summary>
    /// <returns>The size of the image.</returns>
    cv::Size getImageSize(void) const noexcept;
    /// <summary>Gets the size of the mask.</summary>
    /// <returns>The size of the mask.</returns>
    cv::Size getMaskSize(void) const noexcept;
    /// <summary>Compares the frames given by pointers.</summary>
    /// <param name="frame1">The first frame.</param>
    /// <param name="frame2">The second frame.</param>
    /// <returns>The comparison result.</returns>
    static bool FramePointerComparer(Frame *frame1, Frame *frame2) noexcept;
  private:    
    /// <summary>The frame identifier.</summary>
    int m_id = -1;    
    /// <summary>The image.</summary>
    cv::Mat m_image;    
    /// <summary>The mask.</summary>
    cv::Mat m_mask;    
    /// <summary>The skeleton.</summary>
    Skeleton m_skeleton;    
    /// <summary>The ground point/</summary>
    cv::Point2f m_groundPoint = cv::Point2f(0.0, 0.0);  
    /// <summary>
    /// The ID of the frame this lockframe was derived from.
    /// </summary>
    int m_parentFrameID = -1;    
    /// <summary>The image size.</summary>
    cv::Size imageSize = cv::Size(-1, -1);    
    /// <summary>The mask size/</summary>
    cv::Size maskSize = cv::Size(-1, -1);    
    /// <summary>The frame type.</summary>
    FRAMETYPE m_frametype = UNDEFINED;
  };
}
#endif  // _LIBPOSE_FRAME_HPP_
