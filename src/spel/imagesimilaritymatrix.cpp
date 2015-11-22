#include "imagesimilaritymatrix.hpp"

namespace SPEL
{
  ImageSimilarityMatrix::ImageSimilarityMatrix(void) noexcept
  {
    id = 0x00000000;
  }
  ImageSimilarityMatrix::~ImageSimilarityMatrix(void) noexcept
  {
  }

  uint32_t ImageSimilarityMatrix::getID(void)
  {
    return id;
  }

  void ImageSimilarityMatrix::setID(uint32_t _id)
  {
    id = _id;
  }
 
}
