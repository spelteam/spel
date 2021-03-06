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
    ImageMaskSimilarityMatrix(void) ;
    ImageMaskSimilarityMatrix(const ImageMaskSimilarityMatrix &m) ;
    ImageMaskSimilarityMatrix(const std::vector<Frame*> &frames);
    ImageMaskSimilarityMatrix(ImageMaskSimilarityMatrix &&m) ;

    // Alternative MSM
    ImageMaskSimilarityMatrix(const std::vector<Frame*>& frames, int Erode, int Dilate, bool UseRGBScore = false, bool inverseScore = false);
    //

    ///destructor
    ~ImageMaskSimilarityMatrix(void) ;

    ImageMaskSimilarityMatrix & operator=(const ImageMaskSimilarityMatrix &s) ;
  private:
#ifdef DEBUG
    FRIEND_TEST(MaskSimilarityMatrixTests, computeISMCell);
#endif  // DEBUG
    // Inherited via ImageSimilarityMatrix
    void computeISMcell(Frame * left, Frame * right, const int maxFrameHeight);
  };
}

#endif // _IMAGEMASKSIMILARITYMATRIX_HPP_
