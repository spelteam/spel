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
    ///destructor
    virtual ~ImagePixelSimilarityMatrix(void) noexcept;

    virtual ImagePixelSimilarityMatrix & operator=(const ImagePixelSimilarityMatrix &s) noexcept;
  protected:
#ifdef DEBUG
    FRIEND_TEST(ImageSimilarityMatrixTests_, computeISMcell);
    FRIEND_TEST(ImageSimilarityMatrixTests_, computeMSMcell);
#endif  // DEBUG
    virtual void computeISMcell(const Frame* left, const Frame* right, const int maxFrameHeight);
  };
}

#endif  // _IMAGEPIXELSIMILARITYMATRIX_HPP_
