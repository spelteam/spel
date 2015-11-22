#ifndef _IMAGEHOGSIMILARITYMATRIX_HPP_
#define _IMAGEHOGSIMILARITYMATRIX_HPP_

// SPEL definitions
#include "predef.hpp"

#include "imagesimilaritymatrix.hpp"

namespace SPEL
{
  class ImageHogSimilarityMatrix /*: public ImageSimilarityMatrix*/
  {
  public:
    ///constructors
    ImageHogSimilarityMatrix(void) noexcept;
    ImageHogSimilarityMatrix(const ImageHogSimilarityMatrix &m) noexcept;
    ImageHogSimilarityMatrix(const std::vector<Frame*> &frames) noexcept;
    ImageHogSimilarityMatrix(ImageHogSimilarityMatrix &&m) noexcept;

    ///destructor
    virtual ~ImageHogSimilarityMatrix(void) noexcept;
  };
}

#endif  // _IMAGEHOGSIMILARITYMATRIX_HPP_
