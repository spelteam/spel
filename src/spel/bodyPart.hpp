#ifndef _BODYPART_HPP_
#define _BODYPART_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <string>

#include "bodyJoint.hpp"
#include "spelHelper.hpp"

namespace SPEL
{
  /// <summary>
  /// Objects of this class used as elements for building a skeleton model.
  /// See <see cref="Skeleton" /> for more info.
  /// </summary>
  class BodyPart
  {
  public:
    /// <summary>
    /// Initializes a new instance of the <see cref="BodyPart"/> class.
    /// Default constructor.
    /// </summary>
    BodyPart(void) noexcept;
    /// <summary>
    /// Initializes a new instance of the <see cref="BodyPart"/> class.
    /// Copy constructor.
    /// </summary>
    /// <param name="bodyPart">The body part.</param>
    BodyPart(const BodyPart& bodyPart) noexcept;
    /// <summary>
    /// Initializes a new instance of the <see cref="BodyPart"/> class.
    /// Move constructor.
    /// </summary>
    /// <param name="bodyPart">The body part.</param>
    BodyPart(BodyPart&& bodyPart) noexcept;
    /// <summary>
    /// Initializes a new instance of the <see cref="BodyPart"/> class.
    /// </summary>
    /// <param name="id">The identifier.</param>
    /// <param name="name">The name.</param>
    /// <param name="parentJoint">The parent joint.</param>
    /// <param name="childJoint">The child joint.</param>
    BodyPart(const int id, const std::string &name, const int parentJoint, 
      const int childJoint) noexcept;
    /// <summary>
    /// Initializes a new instance of the <see cref="BodyPart"/> class.
    /// </summary>
    /// <param name="id">The identifier.</param>
    /// <param name="name">The name.</param>
    /// <param name="parentJoint">The parent joint.</param>
    /// <param name="childJoint">The child joint.</param>
    /// <param name="isOccluded">Is occluded?</param>
    BodyPart(const int id, const std::string &name, const int parentJoint, 
      const int childJoint, const bool isOccluded) noexcept;
    /// <summary>
    /// Initializes a new instance of the <see cref="BodyPart"/> class.
    /// </summary>
    /// <param name="id">The identifier.</param>
    /// <param name="name">The name.</param>
    /// <param name="parentJoint">The parent joint.</param>
    /// <param name="childJoint">The child joint.</param>
    /// <param name="isOccluded">Is occluded?</param>
    /// <param name="spaceLength">Length of the space.</param>
    BodyPart(const int id, const std::string &name, const int parentJoint, 
      const int childJoint, const bool isOccluded, 
      const float spaceLength) noexcept;

    /// <summary>
    /// Finalizes an instance of the <see cref="BodyPart"/> class.
    /// </summary>
    /// <returns></returns>
    ~BodyPart(void) noexcept;

    /// <summary>Copy operator.</summary>
    /// <param name="bodyPart">The body part.</param>
    /// <returns>The body part.</returns>
    BodyPart& operator=(const BodyPart& bodyPart) noexcept;
    /// <summary>Move operator.</summary>
    /// <param name="bodyPart">The body part.</param>
    /// <returns>The body part.</returns>
    BodyPart& operator=(BodyPart&& bodyPart) noexcept;
    /// <summary>
    /// Comparison operator.
    /// Comparison by unique index.
    /// </summary>
    /// <param name="bodyPart">The body part.</param>
    /// <returns>The comparison result.</returns>
    bool operator==(const BodyPart &bodyPart) const noexcept;
    /// <summary>
    /// Comparison operator.
    /// Comparison by unique index.
    /// </summary>
    /// <param name="bodyPart">The body part.</param>
    /// <returns>The comparison result.</returns>
    bool operator!=(const BodyPart &bodyPart) const noexcept;

    /// <summary>Gets the part identifier.</summary>
    /// <returns>THe part identifier.</returns>
    int getPartID(void) const noexcept;
    /// <summary>Sets the part identifier.</summary>
    /// <param name="partID">The part identifier.</param>
    /// <returns></returns>
    void setPartID(const int partID) noexcept;
    /// <summary>Gets the name of the part.</summary>
    /// <returns>The name of the part.</returns>
    std::string getPartName(void) const noexcept;
    /// <summary>Sets the name of the part.</summary>
    /// <param name="partName">Name of the part.</param>
    /// <returns></returns>    
    void setPartName(const std::string &partName) noexcept;
    /// <summary>Gets the parent joint.</summary>
    /// <returns>The parent joint identifier.</returns>
    int getParentJoint(void) const noexcept;
    /// <summary>Sets the parent joint.</summary>
    /// <param name="parentJoint">The parent joint.</param>
    /// <returns></returns>
    void setParentJoint(const int parentJoint) noexcept;
    /// <summary>Gets the child joint.</summary>
    /// <returns>The child joint identifier.</returns>
    int getChildJoint(void) const noexcept;
    /// <summary>Sets the child joint.</summary>
    /// <param name="childJoint">The child joint.</param>
    /// <returns></returns>
    void setChildJoint(const int childJoint) noexcept;
    /// <summary>Gets the occluded parameter.</summary>
    /// <returns>The occluded parameter.</returns>
    bool getIsOccluded(void) const noexcept;
    /// <summary>Sets the occluded parameter.</summary>
    /// <param name="isOccluded">The occluded parameter.</param>
    /// <returns></returns>
    void setIsOccluded(const bool isOccluded) noexcept;
    /// <summary>Gets the expected distance.</summary>
    /// <returns>The expected distance.</returns>
    float getExpectedDistance(void) const noexcept;
    /// <summary>Sets the expected distance.</summary>
    /// <param name="expectedDistance">The expected distance.</param>
    /// <returns></returns>
    void setExpectedDistance(const float expectedDistance) noexcept;
    /// <summary>Gets the part polygon.</summary>
    /// <returns>The part polygon.</returns>
    POSERECT <cv::Point2f> getPartPolygon(void) const noexcept;
    /// <summary>Sets the part polygon.</summary>
    /// <param name="partPolygon">The part polygon.</param>
    /// <returns></returns>
    void setPartPolygon(
      const POSERECT <cv::Point2f> &partPolygon) noexcept;
    /// <summary>Gets the length/width ratio.</summary>
    /// <returns>The length/width ratio.</returns>
    float getLWRatio(void) const noexcept;
    /// <summary>Sets the length/width ratio.</summary>
    /// <param name="lwRatio">The length/width ratio.</param>
    /// <returns></returns>
    void setLWRatio(const float lwRatio) noexcept;
    /// <summary>Gets the relative length.</summary>
    /// <returns>The relative length.</returns>
    float getRelativeLength(void) const noexcept;
    /// <summary>Sets the relative length.</summary>
    /// <param name="relativeLength">The relative length.</param>
    /// <returns></returns>
    void setRelativeLength(const float relativeLength) noexcept;
    /// <summary>Gets the search radius.</summary>
    /// <returns>The search radius.</returns>
    float getSearchRadius(void) const noexcept;
    /// <summary>Sets the search radius.</summary>
    /// <param name="searchRadius">The search radius.</param>
    /// <returns></returns>
    void setSearchRadius(const float searchRadius) noexcept;
    /// <summary>Gets the rotation search range.</summary>
    /// <returns>The rotation search range.</returns>
    float getRotationSearchRange(void) const noexcept;
    /// <summary>Sets the rotation search range.</summary>
    /// <param name="rotationAngle">The rotation angle.</param>
    /// <returns></returns>
    void setRotationSearchRange(const float rotationAngle) noexcept;
    /// <summary>Gets the length of the bone.</summary>
    /// <param name="begin">The begin of the bone.</param>
    /// <param name="end">The end of the bone.</param>
    /// <returns>The length of the bone.</returns>
    static float getBoneLength(const cv::Point2f &begin,
      const cv::Point2f &end) noexcept;
    /// <summary>Gets the length of the bone.</summary>
    /// <param name="parent">The parent joint.</param>
    /// <param name="child">The child joint.</param>
    /// <returns>The length of the bone.</returns>
    static float getBoneLength(const BodyJoint &parent,
      const BodyJoint &child) noexcept;
    /// <summary>Gets the width of the bone.</summary>
    /// <param name="length">The length of the bone.</param>
    /// <param name="bodyPart">The body part.</param>
    /// <returns>The width of the bone.</returns>
    static float getBoneWidth(const float length,
      const BodyPart &bodyPart) noexcept;    
    /// <summary>Gets the width of the bone.</summary>
    /// <param name="length">The length of the bone.</param>
    /// <returns>The width of the bone.</returns>
    float getBoneWidth(const float length) const noexcept;    
    /// <summary>Gets the width of the bone.</summary>
    /// <param name="begin">The begin of the bone.</param>
    /// <param name="end">The end of the bone.</param>
    /// <param name="bodyPart">The body part.</param>
    /// <returns>The width of the bone.</returns>
    static float getBoneWidth(const cv::Point2f &begin,
      const cv::Point2f &end, const BodyPart &bodyPart) noexcept;
    /// <summary>Gets the width of the bone.</summary>
    /// <param name="parent">The parent joint.</param>
    /// <param name="child">The child joint.</param>
    /// <param name="bodyPart">The body part.</param>
    /// <returns>The width of the bone.</returns>
    static float getBoneWidth(const BodyJoint &parent,
      const BodyJoint &child, const BodyPart &bodyPart) noexcept;
    /// <summary>Gets the width of the bone.</summary>
    /// <param name="begin">The begin of the bone.</param>
    /// <param name="end">The end of the bone.</param>
    /// <returns>The width of the bone.</returns>
    float getBoneWidth(const cv::Point2f &begin,
      const cv::Point2f &end) const noexcept;    
    /// <summary>Gets the width of the bone.</summary>
    /// <param name="parent">The parent joint.</param>
    /// <param name="child">The child joint.</param>
    /// <returns>The width of the bone.</returns>
    float getBoneWidth(const BodyJoint &parent,
      const BodyJoint &child) const noexcept;
    /// <summary>Gets the body part rect.</summary>
    /// <param name="bodyPart">The body part.</param>
    /// <param name="parent">The parent joint.</param>
    /// <param name="child">The child joint.</param>
    /// <returns>The body part rect.</returns>
    static POSERECT <cv::Point2f> getBodyPartRect(const BodyPart &bodyPart,
      const cv::Point2f &parent, const cv::Point2f &child) noexcept;
    /// <summary>Gets the body part rect.</summary>
    /// <param name="parent">The parent joint.</param>
    /// <param name="child">The child joint.</param>
    /// <returns>The body part rect.</returns>
    POSERECT <cv::Point2f> getBodyPartRect(const cv::Point2f &parent, 
      const cv::Point2f &child) const noexcept;
    /// <summary>Gets the body part rect.</summary>
    /// <param name="bodyPart">The body part.</param>
    /// <param name="parent">The parent joint.</param>
    /// <param name="child">The child joint.</param>
    /// <returns>The body part rect.</returns>
    static POSERECT <cv::Point2f> getBodyPartRect(const BodyPart &bodyPart,
      const BodyJoint &parent, const BodyJoint &child) noexcept;
    /// <summary>Gets the body part rect.</summary>
    /// <param name="parent">The parent joint.</param>
    /// <param name="child">The child joint.</param>
    /// <returns>The body part rect.</returns>
    POSERECT <cv::Point2f> getBodyPartRect(const BodyJoint &parent, 
      const BodyJoint &child) const noexcept;
    /// <summary>Gets the body part rect.</summary>
    /// <param name="bodyPart">The body part.</param>
    /// <param name="parent">The parent joint.</param>
    /// <param name="child">The child joint.</param>
    /// <param name="blockSize">Size of the block.</param>
    /// <returns>The body part rect.</returns>
    static POSERECT <cv::Point2f> getBodyPartRect(const BodyPart &bodyPart,
      const cv::Point2f &parent, const cv::Point2f &child, 
      const cv::Size &blockSize) noexcept;
    /// <summary>Gets the body part rect.</summary>
    /// <param name="parent">The parent joint.</param>
    /// <param name="child">The child joint.</param>
    /// <param name="blockSize">Size of the block.</param>
    /// <returns>The body part rect.</returns>
    POSERECT <cv::Point2f> getBodyPartRect(const cv::Point2f &parent, 
      const cv::Point2f &child, const cv::Size &blockSize) const noexcept;
    /// <summary>Gets the body part rect.</summary>
    /// <param name="bodyPart">The body part.</param>
    /// <param name="parent">The parent joint.</param>
    /// <param name="child">The child joint.</param>
    /// <param name="blockSize">Size of the block.</param>
    /// <returns>The body part rect.</returns>
    static POSERECT <cv::Point2f> getBodyPartRect(const BodyPart &bodyPart,
      const BodyJoint &parent, BodyJoint &child,
      const cv::Size &blockSize) noexcept;
    /// <summary>Gets the body part rect.</summary>
    /// <param name="parent">The parent joint.</param>
    /// <param name="child">The child joint.</param>
    /// <param name="blockSize">Size of the block.</param>
    /// <returns>The body part rect.</returns>
    POSERECT <cv::Point2f> getBodyPartRect(const BodyJoint &parent,
      const BodyJoint &child, const cv::Size &blockSize) const noexcept;
  private:
    /// <summary>
    /// The part identifier must be unique within the limits of class.
    /// </summary>
    int m_partID;
    /// <summary>
    /// The part name respectively to a place in a skeleton model.
    /// </summary>
    std::string m_partName;
    /// <summary>
    /// The parent joint identifier of adjacent overlying joint/node. 
    /// See <see cref="BodyJoint" /> for more info.
    /// </summary>
    int m_parentJoint;
    /// <summary>
    /// The child joint identifier of adjacent underlying joint/node.
    /// See <see cref="BodyJoint" /> for more info.
    /// </summary>
    int m_childJoint;
    /// <summary>
    /// The occluded parameter.
    /// When "true" - then this body part is overlapped in a frame.
    /// Used in the skeleton recovery algorithm.
    /// </summary>
    bool m_isOccluded;
    /// <summary>
    /// The expected distance to parent bodypart, 
    /// as a multiplier of this part's length.
    /// </summary>
    float m_expectedDistance;
    /// <summary>
    /// The part polygon rectangle is used as simplified 
    /// representation of body part.
    /// </summary>
    POSERECT <cv::Point2f> m_partPolygon;
    /// <summary>
    /// The length/width ratio coefficient of proportionality 
    /// is used for scaling.
    /// </summary>
    float m_lwRatio;
    /// <summary>
    /// The 3d relative length.
    /// </summary>
    float m_relativeLength;
    /// <summary>
    /// The search radius for detection of this bodypart.
    /// </summary>
    float m_searchRadius;
    /// <summary>
    /// The angle rotation search range to search through
    /// </summary>
    float m_rotationSearchRange;
  };
  
  std::ostream& operator<<(std::ostream& stream, 
    const BodyPart &bodyPart) noexcept;
}
#endif  // _BODYPART_HPP_
