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
    ImageMaskSimilarityMatrix(const std::vector<Frame*> &frames) noexcept;
    ImageMaskSimilarityMatrix(ImageMaskSimilarityMatrix &&m) noexcept;

    ///destructor
    virtual ~ImageMaskSimilarityMatrix(void) noexcept;

    virtual ImageMaskSimilarityMatrix & operator=(const ImageMaskSimilarityMatrix &s) noexcept;
  protected:
    // Inherited via ImageSimilarityMatrix
    virtual void computeISMcell(const Frame * left, const Frame * right, const int maxFrameHeight);
  };
}

#endif // _IMAGEMASKSIMILARITYMATRIX_HPP_
