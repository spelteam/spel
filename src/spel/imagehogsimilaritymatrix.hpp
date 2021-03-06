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
    ImageHogSimilarityMatrix(void) ;
    ImageHogSimilarityMatrix(const ImageHogSimilarityMatrix &m) ;
    ImageHogSimilarityMatrix(const std::vector<Frame*> &frames);
    ImageHogSimilarityMatrix(ImageHogSimilarityMatrix &&m) ;

    ///destructor
    ~ImageHogSimilarityMatrix(void) ;

    ImageHogSimilarityMatrix & operator=(const ImageHogSimilarityMatrix &s) ;
  private:
#ifdef DEBUG
    FRIEND_TEST(ImageHogSimilarityMatrix, defaultConstructor);
    FRIEND_TEST(ImageHogSimilarityMatrix, MoveConstructor);
    FRIEND_TEST(ImageHogSimilarityMatrix, calculateROI);
    FRIEND_TEST(ImageHogSimilarityMatrix, calculateISMCell);
    FRIEND_TEST(ImageHogSimilarityMatrix, frames_ISM);
    FRIEND_TEST(ImageHogSimilarityMatrix, extendSize);
#endif  // DEBUG
      
    const uint8_t nbins = 9;
    cv::Size blockSize = cv::Size(16, 16);
    cv::Size blockStride = cv::Size(8, 8);
    cv::Size cellSize = cv::Size(8, 8);
    double wndSigma = -1;
    double thresholdL2hys = 0.2;
    bool gammaCorrection = true;
    int nlevels = 64;
    cv::Size wndStride = cv::Size(8, 8);
    cv::Size padding = cv::Size(32, 32);
    int derivAperture = 1;
    int histogramNormType = cv::HOGDescriptor::L2Hys;

    void calculateROI(Frame *frame, cv::Point2i &topLeft, cv::Point2i &bottomRight) const ;
    int8_t extendSize(int &topLeftMin, int &topLeftMax, int &bottomRightMin, int &bottomRightMax, const int maxsize, const int add) const;

    // Inherited via ImageSimilarityMatrix
    void computeISMcell(Frame * left, Frame * right, const int maxFrameHeight);
  };
}

#endif  // _IMAGEHOGSIMILARITYMATRIX_HPP_
