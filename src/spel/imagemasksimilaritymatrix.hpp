#ifndef _IMAGEMASKSIMILARITYMATRIX_HPP_
#define _IMAGEMASKSIMILARITYMATRIX_HPP_

// SPEL definitions
#include "predef.hpp"

#include "imagesimilaritymatrix.hpp"

namespace SPEL
{
  class ImageMaskSimilarityMatrix : public ImageSimilarityMatrix
  {
  public:
    ///constructors
    ImageMaskSimilarityMatrix(void) noexcept;
    ImageMaskSimilarityMatrix(const ImageMaskSimilarityMatrix &m) noexcept;
    ImageMaskSimilarityMatrix(const std::vector<Frame*> &frames);
    ImageMaskSimilarityMatrix(ImageMaskSimilarityMatrix &&m) noexcept;

    // Alternative MSM
    // Erode and Dilate - erode or dilate iteration count (= 0 .. 3, 0 = don't used), affects only to the shift masks center.
    ImageMaskSimilarityMatrix(const std::vector<Frame*>& frames, int Erode, int Dilate);
    //

    ///destructor
    virtual ~ImageMaskSimilarityMatrix(void) noexcept;

    virtual ImageMaskSimilarityMatrix & operator=(const ImageMaskSimilarityMatrix &s) noexcept;
  protected:
#ifdef DEBUG
    FRIEND_TEST(MaskSimilarityMatrixTests, computeISMCell);
#endif  // DEBUG
    // Inherited via ImageSimilarityMatrix
    virtual void computeISMcell(Frame * left, Frame * right, const int maxFrameHeight);
  };
}

#endif // _IMAGEMASKSIMILARITYMATRIX_HPP_
