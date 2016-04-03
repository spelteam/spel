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
  class LimbLabel
  {
  public:
    LimbLabel() noexcept;
    LimbLabel(const LimbLabel &ll) noexcept;
    LimbLabel(LimbLabel &&ll) noexcept;
    LimbLabel(int _id, cv::Point2f _centre, float _angle, std::vector<cv::Point2f> _polygon, std::vector<Score> _scores, bool _isOccluded = false) noexcept;
    virtual ~LimbLabel(void) noexcept;
    /// output labels as printable string
    virtual std::string toString() const noexcept;
    virtual LimbLabel & operator = (const LimbLabel &ll) noexcept;
    virtual LimbLabel & operator = (LimbLabel &&ll) noexcept;
    virtual bool operator == (const LimbLabel &ll) const noexcept;
    virtual bool operator != (const LimbLabel &ll) const noexcept;
    virtual bool operator < (const LimbLabel &ll) const noexcept;
    virtual bool operator > (const LimbLabel &ll) const noexcept;
    // compute the endpoints of the limb that this label would produce
    virtual void getEndpoints(cv::Point2f &p0, cv::Point2f &p1) const;
    virtual void addScore(Score detectionScore) noexcept;
    virtual cv::Point2f getCenter(void) const noexcept;
    ///get the label scores
    virtual std::vector<Score> getScores(void) const noexcept;
    virtual void setScores(std::vector <Score> _scores) noexcept;
    virtual int getLimbID(void) const noexcept;
    virtual float getAngle(void) const noexcept;
    virtual std::vector <cv::Point2f> getPolygon(void) const noexcept;
    virtual bool getIsOccluded(void) const noexcept;
    virtual float getAvgScore(bool bNegativeToPositive = false) const noexcept;
    virtual void Resize(float factor) noexcept;
    virtual bool containsPoint(cv::Point2f pt) const;
  private:
#ifdef DEBUG
    FRIEND_TEST(nskpsolverTests, ScoreCostAndJointCost); // Used for set isWeak to "false" in nskpsolver_tests.computeScoreCost
    FRIEND_TEST(nskpsolverTests, evaluateSolution);// Used for set isWeak to "false" in nskpsolver_tests.evaluateSolution
#endif  // DEBUG
    /// Label center 
    cv::Point2f center;
    /// identifier representing what bone this label belongs to
    int limbID;
    /// degrees rotation from y=0 (horizontal line)
    float angle;
    /// detector scores
    std::vector<Score> scores;
    /// polygon for the label (already rotated, scaled etc. - from which scores are generated)
    std::vector<cv::Point2f> polygon;
    /// signify label is for an occluded limb
    bool isOccluded;
  };

}

#endif  // _LIBPOSE_LIMBLABEL_HPP_
