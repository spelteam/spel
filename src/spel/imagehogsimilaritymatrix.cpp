#include "imagehogsimilaritymatrix.hpp"

namespace SPEL
{
  ImageHogSimilarityMatrix::ImageHogSimilarityMatrix(void) noexcept : ImageSimilarityMatrix()
  {
    id = 0x4948534D;
  }
  ImageHogSimilarityMatrix::ImageHogSimilarityMatrix(const ImageHogSimilarityMatrix & m) noexcept : ImageSimilarityMatrix(m)
  {
  }
  ImageHogSimilarityMatrix::ImageHogSimilarityMatrix(const std::vector<Frame*>& frames) : ImageHogSimilarityMatrix()
  {
    buildImageSimilarityMatrix(frames);
  }
  ImageHogSimilarityMatrix::ImageHogSimilarityMatrix(ImageHogSimilarityMatrix && m) noexcept : ImageSimilarityMatrix(std::move(m))
  {
  }
  ImageHogSimilarityMatrix::~ImageHogSimilarityMatrix(void) noexcept
  {
  }
}
