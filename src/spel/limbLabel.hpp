#ifndef _LIBPOSE_LIMBLABEL_HPP_
#define _LIBPOSE_LIMBLABEL_HPP_

// SPEL definitions
#include "predef.hpp"

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

// STL
#ifdef WINDOWS
#define _USE_MATH_DEFINES
#include <math.h>
#endif  // WINDOWS
#include <vector>
#include <string>
#include <cstdint>

// OpenCV
#include <opencv2/opencv.hpp>

#include "spelObject.hpp"
#include "score.hpp"

namespace SPEL
{  
  /// <summary>
  /// This class represents the limb label of each body part
  /// </summary>
  /// <seealso cref="BodyPart" />
  class LimbLabel
  {
  public:
    /// <summary>
    /// Initializes a new instance of the <see cref="LimbLabel"/> class.
    /// </summary>
    LimbLabel(void) ;
    /// <summary>
    /// Initializes a new instance of the <see cref="LimbLabel"/> class.
    /// </summary>
    /// <param name="limbLabel">The limb label.</param>
    LimbLabel(const LimbLabel &limbLabel) ;
    /// <summary>
    /// Initializes a new instance of the <see cref="LimbLabel"/> class.
    /// </summary>
    /// <param name="limbLabel">The limb label.</param>
    LimbLabel(LimbLabel &&limbLabel) ;
    /// <summary>
    /// Initializes a new instance of the <see cref="LimbLabel"/> class.
    /// </summary>
    /// <param name="id">The identifier.</param>
    /// <param name="center">The polygon center.</param>
    /// <param name="angle">The polygon angle.</param>
    /// <param name="polygon">The polygon.</param>
    /// <param name="scores">The scores.</param>
    /// <param name="isOccluded">
    /// if set to <c>true</c> then limb label is occluded.
    /// </param>
    LimbLabel(int id, cv::Point2f center, float angle, 
      std::vector<cv::Point2f> polygon, std::vector<Score> scores, 
      bool isOccluded = false) ;
    /// <summary>
    /// Finalizes an instance of the <see cref="LimbLabel"/> class.
    /// </summary>
    ~LimbLabel(void) ;
    /// <summary>Output limb label as printable string.</summary>
    /// <returns>String representation of the limb label.</returns>
    std::string toString() const ;
    /// <summary>Copies the specified limb label.</summary>
    /// <param name="limbLabel">The limb label.</param>
    /// <returns>New limb label.</returns>
    LimbLabel & operator = (const LimbLabel &limbLabel) ;
    /// <summary>Moves the specified limb label.</summary>
    /// <param name="limbLabel">The limb label.</param>
    /// <returns>New limb label.</returns>
    LimbLabel & operator = (LimbLabel &&limbLabel) ;
    /// <summary>Comparison operator.</summary>
    /// <param name="limbLabel">The limb label.</param>
    /// <returns>The result of the comparison.</returns>
    bool operator == (const LimbLabel &limbLabel) const ;
    /// <summary>Comparison operator.</summary>
    /// <param name="limbLabel">The limb label.</param>
    /// <returns>The result of the comparison.</returns>
    bool operator != (const LimbLabel &limbLabel) const ;
    /// <summary>Comparison operator.</summary>
    /// <param name="limbLabel">The limb label.</param>
    /// <returns>The result of the comparison.</returns>
    bool operator < (const LimbLabel &limbLabel) const ;
    /// <summary>Comparison operator.</summary>
    /// <param name="limbLabel">The limb label.</param>
    /// <returns>The result of the comparison.</returns>
    bool operator > (const LimbLabel &limbLabel) const ;
    /// <summary>
    /// Compute the endpoints of the limb that this label would produce.
    /// </summary>
    /// <param name="p0">The first point.</param>
    /// <param name="p1">The second point.</param>
    void getEndpoints(cv::Point2f &p0, cv::Point2f &p1) const;
    std::pair<cv::Point2f, cv::Point2f> getEndpoints(void) const;
    /// <summary>Adds the score.</summary>
    /// <param name="detectionScore">The detection score.</param>
    void addScore(Score detectionScore) ;
    /// <summary>Gets the center of the polygon.</summary>
    /// <returns>The center of the polygon.</returns>
    cv::Point2f getCenter(void) const ;
    /// <summary>Gets the label scores.</summary>
    /// <returns>The label scores.</returns>
    std::vector<Score> getScores(void) const ;
    /// <summary>Sets the label scores.</summary>
    /// <param name="scores">The label scores.</param>
    void setScores(std::vector <Score> scores) ;
    /// <summary>Gets the limb label identifier.</summary>
    /// <returns>The limb label.</returns>
    int getLimbID(void) const ;
    /// <summary>Gets the limb label angle.</summary>
    /// <returns>The limb label angle.</returns>
    float getAngle(void) const ;
    /// <summary>Gets the limb label polygon.</summary>
    /// <returns>The limb label polygon.</returns>
    std::vector <cv::Point2f> getPolygon(void) const ;
    /// <summary>
    /// Returns <c>true</c> if limb label is occluded. 
    /// Otherwise returns <c>false</c>.
    /// </summary>
    /// <returns>
    /// Returns <c>true</c> if limb label is occluded. 
    /// Otherwise returns <c>false</c>.
    /// </returns>
    bool getIsOccluded(void) const ;
    /// <summary>Gets the average score.</summary>
    /// <param name="bNegativeToPositive">
    /// If set to <c>true</c> all negative scores are used as positive.
    /// </param>
    /// <returns>The average score.</returns>
    float getAvgScore(bool bNegativeToPositive = false) const ;    
    /// <summary>Gets the sum score.</summary>
    /// <param name="bNegativeToPositive">
    /// If set to <c>true</c> all negative scores are used as positive.
    /// </param>
    /// <returns>The sum score.</returns>
    float getSumScore(bool bNegativeToPositive = false) const ;
    /// <summary>Resizes the limb label using specified factor.</summary>
    /// <param name="factor">The factor.</param>
    void Resize(float factor) ;
    /// <summary>Determines whether the limb label contains specified point.</summary>
    /// <param name="pt">The point.</param>
    /// <returns>
    /// Returns <c>true</c> if the limb label contains specified point. 
    /// Otherwise returns <c>false</c>.
    /// </returns>
    bool containsPoint(cv::Point2f pt) const;
  private:
#ifdef DEBUG
    // Used for set isWeak to "false" in nskpsolver_tests.computeScoreCost
    FRIEND_TEST(nskpsolverTests, ScoreCostAndJointCost);
    // Used for set isWeak to "false" in nskpsolver_tests.evaluateSolution
    FRIEND_TEST(nskpsolverTests, evaluateSolution);
#endif  // DEBUG
    /// <summary>The limb label center</summary>
    cv::Point2f m_center;
    /// <summary>
    /// The identifier representing what bone this label belongs to.
    /// </summary>
    int m_limbID;
    /// <summary>The degrees rotation from y=0 (horizontal line).</summary>
    float m_angle;
    /// <summary>The detector scores.</summary>
    std::vector<Score> m_scores;
    /// <summary>
    /// The polygon for the label (already rotated, scaled etc.)
    /// from which scores are generated)
    /// </summary>
    std::vector<cv::Point2f> m_polygon;
    /// <summary>signify label is for an occluded limb.</summary>
    bool m_isOccluded;
  };

}

#endif  // _LIBPOSE_LIMBLABEL_HPP_
