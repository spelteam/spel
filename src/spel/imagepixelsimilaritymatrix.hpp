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
    ImagePixelSimilarityMatrix(void) ;
    ImagePixelSimilarityMatrix(const ImagePixelSimilarityMatrix &m) ;
    ImagePixelSimilarityMatrix(const std::vector<Frame*> &frames);
    ImagePixelSimilarityMatrix(ImagePixelSimilarityMatrix &&m) ;

    // Alternative MSM
    ImagePixelSimilarityMatrix(const std::vector<Frame*>& frames, int Erode, int Dilate, bool UseRGBScore = true, bool inverseScore = false);
    //

    ///destructor
    ~ImagePixelSimilarityMatrix(void) ;

    ImagePixelSimilarityMatrix & operator=(const ImagePixelSimilarityMatrix &s) ;
  private:
#ifdef DEBUG
    FRIEND_TEST(ImageSimilarityMatrixTests_, computeISMcell);
    FRIEND_TEST(ImageSimilarityMatrixTests_, computeMSMcell);
#endif  // DEBUG
    void computeISMcell(Frame* left, Frame* right, const int maxFrameHeight);
  };
}

#endif  // _IMAGEPIXELSIMILARITYMATRIX_HPP_
