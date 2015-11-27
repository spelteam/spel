#ifndef _IMAGEHOGSIMILARITYMATRIX_HPP_
#define _IMAGEHOGSIMILARITYMATRIX_HPP_

// SPEL definitions
#include "predef.hpp"

#include "imagesimilaritymatrix.hpp"

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
  protected:
    const uint8_t nbins = 9;
    cv::Size blockSize = cv::Size(16, 16);
    cv::Size blockStride = cv::Size(8, 8);
    cv::Size cellSize = cv::Size(8, 8);
    cv::Size wndSize = cv::Size(64, 128);
    double wndSigma = -1;
    double thresholdL2hys = 0.2;
    bool gammaCorrection = true;
    int nlevels = 64;
    cv::Size wndStride = cv::Size(8, 8);
    cv::Size padding = cv::Size(32, 32);
    int derivAperture = 1;
    int histogramNormType = cv::HOGDescriptor::L2Hys;

    // Inherited via ImageSimilarityMatrix
    virtual void computeISMcell(const Frame * left, const Frame * right, const int maxFrameHeight);
  };
}

#endif  // _IMAGEHOGSIMILARITYMATRIX_HPP_
