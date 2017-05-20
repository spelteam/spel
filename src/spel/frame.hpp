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
    Frame(void) ;
    /// <summary>
    /// Initializes a new instance of the <see cref="Frame"/> class.
    /// </summary>
    /// <param name="frametype">The frametype.</param>
    Frame(FRAMETYPE frametype) ;
    /// <summary>
    /// Copy constructor.
    /// Initializes a new instance of the <see cref="Frame"/> class.
    /// </summary>
    /// <param name="frame">The frame.</param>
    Frame(const Frame &frame) ;
    /// <summary>
    /// Finalizes an instance of the <see cref="Frame"/> class.
    /// </summary>
    /// <returns></returns>
    virtual ~Frame(void) ;    
    /// <summary>Assignment operator.</summary>
    /// <param name="frame">The frame.</param>
    /// <returns>The frame.</returns>
    Frame& operator=(const Frame &frame) ;
    /// <summary>Gets the part polygon.</summary>
    /// <param name="partID">The part identifier.</param>
    /// <returns>The part polygon.</returns>
    std::vector <cv::Point2f> getPartPolygon(int partID) const ;
    /// <summary>Gets the identifier.</summary>
    /// <returns>The identifier.</returns>
    int getID(void) const ;
    /// <summary>Sets the identifier.</summary>
    /// <param name="id">The identifier.</param>
    void setID(int id) ;
    /// <summary>Gets the image.</summary>
    /// <returns>The image.</returns>
    cv::Mat getImage(void);
    /// <summary>Sets the image.</summary>
    /// <param name="image">The image.</param>
    /// <param name="cacheFile">
    /// if set to <c>true</c> then file will be cached.
    /// </param>
    void setImage(const cv::Mat &image, const bool cacheFile = true);
    /// <summary>Caches the image.</summary>
    void cacheImage(void);
    /// <summary>Gets the mask.</summary>
    /// <returns>The mask.</returns>
    cv::Mat getMask(void);
    /// <summary>Sets the mask.</summary>
    /// <param name="mask">The mask.</param>
    /// <param name="cacheFile">
    /// if set to <c>true</c> then file will be cached.
    /// </param>
    void setMask(const cv::Mat &mask, const bool cacheFile = true);
    /// <summary>Caches the mask.</summary>
    void cacheMask(void);
    /// <summary>Gets the skeleton.</summary>
    /// <returns>The skeleton.</returns>
    Skeleton getSkeleton(void) const ;
    //TODO: [!]Refactor getters for direct access
    /// <summary>Gets the pointer to the skeleton.</summary>
    /// <returns>The pointer to the skeleton.</returns>
    Skeleton* getSkeletonPtr() ;
    /// <summary>Shifts the skeleton.</summary>
    /// <param name="shift">The shift.</param>
    /// <returns></returns>
    void shiftSkeleton2D(cv::Point2f shift) ;
    /// <summary>Sets the skeleton.</summary>
    /// <param name="skeleton">The skeleton.</param>
    void setSkeleton(const Skeleton &skeleton) ;
    /// <summary>Gets the ground point.</summary>
    /// <returns>The ground point.</returns>
    cv::Point2f getGroundPoint(void) const ;
    /// <summary>Sets the ground point.</summary>
    /// <param name="groundPoint">The ground point.</param>
    void setGroundPoint(const cv::Point2f &groundPoint) ;
    /// <summary>Gets the frametype.</summary>
    /// <returns>The frametype.</returns>
    FRAMETYPE getFrametype(void) const ;
    /// <summary>Gets the parent frame identifier.</summary>
    /// <returns>The parent frame identifier.</returns>
    int getParentFrameID(void) const ;
    /// <summary>Sets the parent frame identifier.</summary>
    /// <param name="parentFrameID">The parent frame identifier.</param>
    void setParentFrameID(int parentFrameID) ;
    /// <summary>Resizes the frame.</summary>
    /// <param name="maxHeight">The maximum height.</param>
    /// <returns>Resize coefficient.</returns>
    float Resize(uint32_t maxHeight);
    /// <summary>Clones the frame to the destination.</summary>
    /// <param name="dest">The destination.</param>
    /// <returns>The cloned frame.</returns>
    Frame *clone(Frame *dest) const ;
    /// <summary>Gets the size of the frame.</summary>
    /// <returns>The size of the frame.</returns>
    cv::Size getFrameSize(void) ;
    /// <summary>Gets the size of the image.</summary>
    /// <returns>The size of the image.</returns>
    cv::Size getImageSize(void) ;
    /// <summary>Gets the size of the mask.</summary>
    /// <returns>The size of the mask.</returns>
    cv::Size getMaskSize(void) ;
    /// <summary>Compares the frames given by pointers.</summary>
    /// <param name="frame1">The first frame.</param>
    /// <param name="frame2">The second frame.</param>
    /// <returns>The comparison result.</returns>
    static bool FramePointerComparer(Frame *frame1, Frame *frame2) ;    
    /// <summary>Scales the skeleton with the specified factor.</summary>
    /// <param name="factor">The factor.</param>
    void Scale(const float factor) ;
    /// <summary>Adjusts the scale.</summary>
    /// <returns></returns>
    void AdjustScale(void) ;
    /// <summary>Gets the image path.</summary>
    /// <returns>The image path.</returns>
    std::string GetImagePath(void) const ;
    /// <summary>Sets the image path.</summary>
    /// <param name="path">The path.</param>
    /// <returns></returns>
    void SetImageFromPath(const std::string &path);
    /// <summary>Gets the mask path.</summary>
    /// <returns>The mask path.</returns>
    std::string GetMaskPath(void) const ;
    /// <summary>Sets the mask path.</summary>
    /// <param name="path">The path.</param>
    /// <returns></returns>
    void SetMaskFromPath(const std::string &path);
    /// <summary>Loads image and mask.</summary>
    void LoadAll(void);
    /// <summary>Loads image and mask.</summary>
    /// <param name="imagePath">The image path.</param>
    /// <param name="maskPath">The mask path.</param>
    void LoadAll(const std::string &imagePath, const std::string &maskPath);
    /// <summary>Loads the image.</summary>
    void LoadImage(void);
    /// <summary>Loads the image.</summary>
    /// <param name="path">The image path.</param>
    void LoadImage(const std::string &path);
    /// <summary>Loads the mask.</summary>
    void LoadMask(void);
    /// <summary>Loads the mask.</summary>
    /// <param name="path">The mask path.</param>
    void LoadMask(const std::string &path);
    /// <summary>Unloads image and mask.</summary>
    /// <param name="force">
    /// If true unloads image and mask even if 
    /// image path and mask path are empty.
    /// </param>
    /// <returns></returns>
    bool UnloadAll(const bool force = false) ;
    /// <summary>Unloads the image.</summary>
    /// <param name="force">
    /// If true unloads image even if image path is empty.
    /// </param>
    /// <returns></returns>
    bool UnloadImage(const bool force = false) ;
    /// <summary>Unloads the mask.</summary>
    /// <param name="force">
    /// If true unloads mask even if mask path is empty.
    /// </param>
    /// <returns></returns>
    bool UnloadMask(const bool force = false) ;
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
    /// <summary>The scale factor</summary>
    float m_scaleFactor = 0.0f;    
    /// <summary>The image path</summary>
    std::string m_imagePath = "";    
    /// <summary>The mask path</summary>
    std::string m_maskPath = "";
  };
}
#endif  // _LIBPOSE_FRAME_HPP_
