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
    ImageMaskSimilarityMatrix(const std::vector<Frame*>& frames, int Erode, int Dilate, bool UseRGBScore = false, bool inverseScore = false);
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
