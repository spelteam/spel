#ifndef _IMAGEPIXELSIMILARITYMATRIX_HPP_
#define _IMAGEPIXELSIMILARITYMATRIX_HPP_

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
    ImagePixelSimilarityMatrix(const std::vector<Frame*> &frames) noexcept;
    ImagePixelSimilarityMatrix(ImagePixelSimilarityMatrix &&m) noexcept;
    ///destructor
    virtual ~ImagePixelSimilarityMatrix(void) noexcept;
  protected:

    virtual void computeMSMcell(const Frame* left, const Frame* right, const int maxFrameHeight);
    virtual void computeISMcell(const Frame* left, const Frame* right, const int maxFrameHeight);
  };
}

#endif  // _IMAGEPIXELSIMILARITYMATRIX_HPP_
