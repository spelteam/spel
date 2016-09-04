#ifndef _IMAGEPIXELSIMILARITYMATRIX_HPP_
#define _IMAGEPIXELSIMILARITYMATRIX_HPP_

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

// SPEL definitions
#include "predef.hpp"

#include "imagesimilaritymatrix.hpp"

namespace SPEL
{
  class ImagePixelSimilarityMatrix : public ImageSimilarityMatrix
  {
  public:
    ///constructors
    ImagePixelSimilarityMatrix(void) noexcept;
    ImagePixelSimilarityMatrix(const ImagePixelSimilarityMatrix &m) noexcept;
    ImagePixelSimilarityMatrix(const std::vector<Frame*> &frames);
    ImagePixelSimilarityMatrix(ImagePixelSimilarityMatrix &&m) noexcept;

    // Alternative MSM
    ImagePixelSimilarityMatrix(const std::vector<Frame*>& frames, int Erode, int Dilate, bool UseRGBScore = false, bool inverseScore = false);
    //

    ///destructor
    virtual ~ImagePixelSimilarityMatrix(void) noexcept;

    ImagePixelSimilarityMatrix & operator=(const ImagePixelSimilarityMatrix &s) noexcept;
  private:
#ifdef DEBUG
    FRIEND_TEST(ImageSimilarityMatrixTests_, computeISMcell);
    FRIEND_TEST(ImageSimilarityMatrixTests_, computeMSMcell);
#endif  // DEBUG
    void computeISMcell(Frame* left, Frame* right, const int maxFrameHeight);
  };
}

#endif  // _IMAGEPIXELSIMILARITYMATRIX_HPP_
