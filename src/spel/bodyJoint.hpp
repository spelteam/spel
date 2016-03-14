#ifndef _BODYJOINT_HPP_
#define _BODYJOINT_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <string>

// OpenCV
#include <opencv2/opencv.hpp>

namespace SPEL
{
/// <summary>
/// Objects of this class used as elements for building a skeleton model.
/// See <see cref="Skeleton"/> for more details.
/// </summary>
  class BodyJoint
  {
  public:    
    /// <summary>
    /// Initializes a new instance of the <see cref="BodyJoint"/> class.
    /// Default constructor
    /// </summary>
    BodyJoint(void) noexcept; 
    /// <summary>
    /// Initializes a new instance of the <see cref="BodyJoint"/> class.
    /// copy constructor
    /// </summary>
    /// <param name="bodyJoint">The body joint.</param>
    BodyJoint(const BodyJoint& bodyJoint) noexcept;
    /// <summary>
    /// Initializes a new instance of the <see cref="BodyJoint"/> class.
    /// Move constructor
    /// </summary>
    /// <param name="bodyJoint">The body joint.</param>
    BodyJoint(BodyJoint&& bodyJoint) noexcept;
    /// <summary>
    /// Initializes a new instance of the <see cref="BodyJoint"/> class.
    /// </summary> 
    /// <param name="id">The identifier.</param>
    /// <param name="name">The name.</param>
    /// <param name="imageLocation">The image location.</param>
    BodyJoint(const int id, const std::string &name, 
      const cv::Point2f &imageLocation) noexcept;
    /// <summary>
    /// Initializes a new instance of the <see cref="BodyJoint"/> class.
    /// </summary>
    /// <param name="id">The identifier.</param>
    /// <param name="name">The name.</param>
    /// <param name="imgLoc">The image location.</param>
    /// <param name="spaceLoc">The space location.</param>
    BodyJoint(const int id, const std::string &name, 
      const cv::Point2f &imagLocation,
      const cv::Point3f &spaceLocation) noexcept;
    /// <summary>
    /// Initializes a new instance of the <see cref="BodyJoint"/> class.
    /// Constructor with params
    /// </summary>
    /// <param name="id">The identifier.</param>
    /// <param name="name">The name.</param>
    /// <param name="imgLocation">The image location.</param>
    /// <param name="spaceLocation">The space location.</param>
    /// <param name="depth">The depth.</param>
    BodyJoint(const int id, const std::string &name, 
      const cv::Point2f &imageLocation, const cv::Point3f &spaceLocation, 
      const bool depth) noexcept;

    /// <summary>
    /// Finalizes an instance of the <see cref="BodyJoint"/> class.
    /// </summary>
    /// <returns></returns>
    virtual ~BodyJoint(void) noexcept;

    /// <summary>
    /// Copy operator.
    /// </summary>
    /// <param name="bodyJoint">The body joint.</param>
    /// <returns>The body joint copy.</returns>
    BodyJoint& operator=(const BodyJoint& bodyJoint) noexcept;
    /// <summary>
     /// Move operator.
     /// </summary>
     /// <param name="bodyJoint">The body joint.</param>
     /// <returns>The body joint.</returns>
    BodyJoint& operator=(BodyJoint&& bodyJoint) noexcept;
    /// <summary>
    /// Comparison operator.
    /// Comparsion by unique index
    /// </summary>
    /// <param name="bodyJoint">The bodyJoint.</param>
    /// <returns>The comparison result.</returns>
    bool operator==(const BodyJoint &bodyJoint) const noexcept;
    /// <summary>
    /// Comparison operator.
    /// Comparsion by unique index 
    /// </summary>
    /// <param name="bodyJoint">The bodyJoint.</param>
    /// <returns>The comparison result</returns>
    bool operator!=(const BodyJoint &bodyJoint) const noexcept;

    /// <summary>
    /// Gets the limb identifier.
    /// </summary>
    /// <returns>Limb identifier.</returns>
    int getLimbID(void) const noexcept;
    /// <summary>Sets the limb identifier.</summary>
    /// <param name="limbID">The limb identifier.</param>
    /// <returns></returns>
    void setLimbID(const int limbID) noexcept;
    /// <summary>Gets the name of the joint.</summary>
    /// <returns>The joint name.</returns>
    std::string getJointName(void) const noexcept;
    /// <summary>Sets the name of the joint.</summary>
    /// <param name="jointName">Name of the joint.</param>
    /// <returns></returns>
    void setJointName(const std::string &jointName) noexcept;
    /// <summary>Gets the image location.</summary>
    /// <returns>The image location.</returns>
    cv::Point2f getImageLocation(void) const noexcept;
    /// <summary>Sets the image location.</summary>
    /// <param name="imageLocation">The image location.</param>
    /// <returns></returns>
    void setImageLocation(const cv::Point2f &imageLocation) noexcept;
    /// <summary>Gets the space location.</summary>
    /// <returns>The space location.</returns>
    cv::Point3f getSpaceLocation(void) const noexcept;
    /// <summary>Sets the space location.</summary>
    /// <param name="spaceLocation">The space location.</param>
    /// <returns></returns>
    void setSpaceLocation(const cv::Point3f &spaceLocation) noexcept;
    /// <summary>Gets the depth sign.</summary>
    /// <returns>The depth sign.</returns>
    bool getDepthSign(void) const noexcept;
    /// <summary>Sets the depth sign.</summary>
    /// <param name="depthSign">The depth sign.</param>
    /// <returns></returns>
    void setDepthSign(const bool depthSign) noexcept;
  private:
    /// <summary>
    /// The identifier, must be unique within the limits of class.
    /// </summary>
    int m_limbID;    
    /// <summary>
    /// The object name, respectively to a place in a skeleton model.
    /// </summary>
    std::string m_jointName;    
    /// <summary>
    /// The coordinates  of the joint on surface of the frame.
    /// </summary>
    cv::Point2f m_imageLocation;     
    /// <summary>
    /// The expansion of the model for use in 3D space.
    /// </summary>
    cv::Point3f m_spaceLocation;    
    /// <summary>
    /// Can be used as a flag to switch between 2D/3D, or indicate presence 
    /// of several plans in the frame.
    /// </summary>
    bool m_depthSign;
  };
}
#endif  //_BODYJOINT_HPP_
