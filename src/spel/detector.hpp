#ifndef _LIBPOSE_DETECTOR_HPP_
#define _LIBPOSE_DETECTOR_HPP_

// SPEL definitions
#include "predef.hpp"

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

// STL
#include <vector>
#include <map>
#include <string>
#include <functional>

#include "frame.hpp"
#include "limbLabel.hpp"

namespace SPEL
{  
  /// <summary>
  /// Helper class for <see cref="Detector" /> class.
  /// </summary>
  class DetectorHelper
  {
  public:
    /// <summary>
    /// Initializes a new instance of the <see cref="DetectorHelper"/> class.
    /// </summary>
    DetectorHelper(void) ;
    /// <summary>
    /// Finalizes an instance of the <see cref="DetectorHelper"/> class.
    /// </summary>
    /// <returns></returns>
    virtual ~DetectorHelper(void) ;
  };  
  /// <summary>
  /// Base Detector class with common functions
  /// </summary>
  class Detector
  {
  public:
    /// <summary>
    /// Initializes a new instance of the <see cref="Detector"/> class.
    /// </summary>
    Detector(void) ;
    /// <summary>
    /// Finalizes an instance of the <see cref="Detector"/> class.
    /// </summary>
    /// <returns></returns>
    virtual ~Detector(void) ;
    /// <summary>Gets the identifier.</summary>
    /// <returns>The identifier.</returns>
    int getID(void) const ;
    /// <summary>Sets the identifier.</summary>
    /// <param name="id">The identifier.</param>
    void setID(const int id);    
    /// <summary>Trains the specified frames.</summary>
    /// <param name="frames">The frames.</param>
    /// <param name="params">The parameters.</param>
    virtual void train(const std::vector <Frame*> &frames,
      std::map <std::string, float> params) = 0;    
    /// <summary>Detects the specified frame.</summary>
    /// <param name="frame">The frame.</param>
    /// <param name="params">The parameters.</param>
    /// <param name="limbLabels">The limb labels.</param>
    /// <returns>The limb labels.</returns>
    virtual std::map <uint32_t, std::vector <LimbLabel> > detect(
      Frame *frame, std::map <std::string, float> params, 
      const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels) 
      const = 0;
    /// <summary>Detects the specified frame.</summary>
    /// <param name="frame">The frame.</param>
    /// <param name="params">The parameters.</param>
    /// <param name="limbLabels">The limb labels.</param>
    /// <param name="detectorHelper">The detector helper.</param>
    /// <returns>The limb labels.</returns>
    std::map <uint32_t, std::vector <LimbLabel> > detect(
      Frame *frame, std::map <std::string, float> params, 
      const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels, 
      DetectorHelper *detectorHelper) const;
    /// <summary>Merges the specified limb labels.</summary>
    /// <param name="first">The first array of limb labels.</param>
    /// <param name="second">The second array of limb labels.</param>
    /// <param name="secondUnfiltered">
    /// The second array of unfiltered limb labels.
    /// </param>
    /// <returns>Merged limb labels.</returns>
    std::map <uint32_t, std::vector <LimbLabel>> merge(
      const std::map <uint32_t, std::vector <LimbLabel>> &first, 
      const std::map <uint32_t, std::vector <LimbLabel>> &second, 
      const std::map <uint32_t, std::vector <LimbLabel>> &secondUnfiltered) 
      const;
  protected:
#ifdef DEBUG
    FRIEND_TEST(DetectorTests, getFrame);
    FRIEND_TEST(DetectorTests, getBodyPartRect);
    FRIEND_TEST(DetectorTests, getBodyPartRect_withBlockSize);
    FRIEND_TEST(DetectorTests, generateLabel1);
    FRIEND_TEST(DetectorTests, generateLabel2);
    FRIEND_TEST(DetectorTests, FilterLimbLabels);
    FRIEND_TEST(DetectorTests, FilterLimbLabels_B);
#endif  // DEBUG    
    /// <summary>The frames.</summary>
    std::vector <Frame*> m_frames;    
    /// <summary>
    /// The maximum frame height.
    /// </summary>
    uint32_t maxFrameHeight;    
    /// <summary>The identifier.</summary>
    uint32_t m_id;
    /// <summary>Gets the frame.</summary>
    /// <param name="frameId">The frame identifier.</param>
    /// <returns>The frame.</returns>
    Frame *getFrame(const int32_t frameId) const ;
    /// <summary>Generates the label.</summary>
    /// <param name="bodyPart">The body part.</param>
    /// <param name="parent">The parent joint.</param>
    /// <param name="child">The child joint.</param>
    /// <param name="detectorName">Name of the detector.</param>
    /// <param name="coeff">The coefficient.</param>
    /// <param name="compare">The compare function.</param>
    /// <returns>Generated label.</returns>
    LimbLabel generateLabel(const BodyPart &bodyPart, 
      const cv::Point2f &parent, const cv::Point2f &child, 
      const std::string &detectorName, const float coeff, 
      const std::function<float()> &compare) const;    
    /// <summary>Generates the label.</summary>
    /// <param name="bodyPart">The body part.</param>
    /// <param name="workFrame">The work frame.</param>
    /// <param name="parent">The parent joint.</param>
    /// <param name="child">The child joint.</param>
    /// <param name="detectorHelper">The detector helper.</param>
    /// <param name="params">The parameters.</param>
    /// <returns>Generated label.</returns>
    virtual LimbLabel generateLabel(const BodyPart &bodyPart,
      Frame *workFrame, const cv::Point2f &parent, const cv::Point2f &child, 
      DetectorHelper *detectorHelper, std::map <std::string, float> params) 
      const = 0;
    /// <summary>Generates the label.</summary>
    /// <param name="boneLength">Length of the bone.</param>
    /// <param name="rotationAngle">The rotation angle.</param>
    /// <param name="x">The x coordinate.</param>
    /// <param name="y">The y coordinate.</param>
    /// <param name="bodyPart">The body part.</param>
    /// <param name="workFrame">The work frame.</param>
    /// <param name="detectorHelper">The detector helper.</param>
    /// <param name="params">The parameters.</param>
    /// <returns>Generated label.</returns>
    LimbLabel generateLabel(const float boneLength, 
      const float rotationAngle, const float x, const float y,
      const BodyPart &bodyPart, Frame *workFrame,
      DetectorHelper *detectorHelper, std::map <std::string, float> params) 
      const;
    /// <summary>Filters the limb labels.</summary>
    /// <param name="sortedLabels">The sorted labels.</param>
    /// <param name="uniqueLocationCandidates">The unique location candidates.</param>
    /// <param name="uniqueAngleCandidates">The unique angle candidates.</param>
    /// <returns>Filtered limb labels.</returns>
    static std::vector <LimbLabel> filterLimbLabels(
      const std::vector <LimbLabel> &sortedLabels,
      const float uniqueLocationCandidates, 
      const float uniqueAngleCandidates);    
    /// <summary>Trains the specified frames.</summary>
    /// <param name="frames">The frames.</param>
    /// <param name="params">The parameters.</param>
    /// <param name="handler">The handler.</param>
    void train(const std::vector <Frame*> &frames,
      std::map <std::string, float> params, 
      const std::function<void(Frame*, const float)> &handler);    
    /// <summary>Emplaces the default parameters.</summary>
    /// <param name="params">The parameters.</param>
    virtual void emplaceDefaultParameters(
      std::map <std::string, float> &params) const ;
  };
}
#endif  // _LIBPOSE_DETECTOR_HPP_
