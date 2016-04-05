#ifndef _IMAGEHOGSIMILARITYMATRIX_HPP_
#define _IMAGEHOGSIMILARITYMATRIX_HPP_

// SPEL definitions
#include "predef.hpp"

// SPEL
#include "imagesimilaritymatrix.hpp"
#include "hogDetector.hpp"

namespace SPEL
{
  class ImageHogSimilarityMatrix : public ImageSimilarityMatrix
  {
  public:
    ///constructors
    ImageHogSimilarityMatrix(void) noexcept;
    ImageHogSimilarityMatrix(const ImageHogSimilarityMatrix &m) noexcept;
    ImageHogSimilarityMatrix(const std::vector<Frame*> &frames);
    ImageHogSimilarityMatrix(ImageHogSimilarityMatrix &&m) noexcept;

    ///destructor
    virtual ~ImageHogSimilarityMatrix(void) noexcept;

    virtual ImageHogSimilarityMatrix & operator=(const ImageHogSimilarityMatrix &s) noexcept;
  protected:
#ifdef DEBUG
    FRIEND_TEST(ImageHogSimilarityMatrix, defaultConstructor);
    FRIEND_TEST(ImageHogSimilarityMatrix, MoveConstructor);
    FRIEND_TEST(ImageHogSimilarityMatrix, calculateROI);
    FRIEND_TEST(ImageHogSimilarityMatrix, calculateISMCell);
    FRIEND_TEST(ImageHogSimilarityMatrix, frames_ISM);
    FRIEND_TEST(ImageHogSimilarityMatrix, extendSize);
#endif  // DEBUG
      
    const cv::Size blockSize = cv::Size(16, 16);

    virtual void calculateROI(const Frame *frame, cv::Point2i &topLeft, cv::Point2i &bottomRight) const noexcept;
    virtual int8_t extendSize(int &topLeftMin, int &topLeftMax, int &bottomRightMin, int &bottomRightMax, const int maxsize, const int add) const;

    // Inherited via ImageSimilarityMatrix
    virtual void computeISMcell(const Frame * left, const Frame * right, const int maxFrameHeight);
  };
}

#endif  // _IMAGEHOGSIMILARITYMATRIX_HPP_
