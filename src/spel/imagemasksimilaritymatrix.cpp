#include "imagemasksimilaritymatrix.hpp"

namespace SPEL
{
  ImageMaskSimilarityMatrix::ImageMaskSimilarityMatrix(void) noexcept : ImageSimilarityMatrix()
  {
  }

  ImageMaskSimilarityMatrix::ImageMaskSimilarityMatrix(const ImageMaskSimilarityMatrix & m) noexcept : ImageSimilarityMatrix(m)
  {
  }

  ImageMaskSimilarityMatrix::ImageMaskSimilarityMatrix(const std::vector<Frame*>& frames) : ImageMaskSimilarityMatrix()
  {
    buildImageSimilarityMatrix(frames);
  }

  ImageMaskSimilarityMatrix::ImageMaskSimilarityMatrix(ImageMaskSimilarityMatrix && m) noexcept : ImageSimilarityMatrix(std::move(m))
  {
  }

  ImageMaskSimilarityMatrix::~ImageMaskSimilarityMatrix(void) noexcept
  {
  }

  ImageMaskSimilarityMatrix & ImageMaskSimilarityMatrix::operator=(const ImageMaskSimilarityMatrix & s) noexcept
  {
    ImageSimilarityMatrix::operator=(s);
    return *this;
  }

  void ImageMaskSimilarityMatrix::computeISMcell(const Frame * left, const Frame * right, const int maxFrameHeight)
  {
    auto i = left->getID(), j = right->getID();
    //only do this loop if
    if (j > i)
      return;
    if (i == j)
    {
      imageShiftMatrix.at<cv::Point2f>(i, j) = cv::Point2f(0, 0);
      imageSimilarityMatrix.at<float>(i, j) = 0;
      return;
    }

    auto maskMatOne = left->getMask();
    auto maskMatTwo = right->getMask();

    //cOne and cTwo now have the centres
    auto dX = calculateDistance(maskMatOne, maskMatTwo);

    imageShiftMatrix.at<cv::Point2f>(i, j) = cv::Point2f(dX.x, dX.y); // 09.04.16 Replaced x and y
    imageShiftMatrix.at<cv::Point2f>(j, i) = cv::Point2f(-dX.x, -dX.y); // 09.04.16 Replaced x and y

    auto maskSimilarityScore = 0.0f;
    auto intersectCount = 0, unionCount = 0;
    for (auto y = 0; y < maskMatOne.rows; ++y)  // 09.04.16 Replaced x to y
    {
      for (auto x = 0; x < maskMatOne.cols; ++x)  // 09.04.16 Replaced y to x
      {
        auto mintensityOne = maskMatOne.at<uchar>(y, x); // 09.04.16 Changed "at<uchar>(j, i)" to "at<uchar>(y, x)"
        auto darkPixel = mintensityOne < 10; //if all intensities are zero

                                             //apply the transformation
        auto xTwo = x + dX.x;
        auto yTwo = y + dX.y;

        auto mintensityTwo = maskMatTwo.at<uchar>(yTwo, xTwo);

        auto blackPixel = mintensityTwo < 10; //if all intensities are zero

        if (!blackPixel && !darkPixel) //if neither is pixel is black
          intersectCount++;
        if (!blackPixel || !darkPixel)
          unionCount++;

        //maskSimilarityScore += std::abs(mOne - mTwo);
      }
    }
    maskSimilarityScore = static_cast<float>(intersectCount) / static_cast<float>(unionCount);   
    imageSimilarityMatrix.at<float>(i, j) = maskSimilarityScore;
    imageSimilarityMatrix.at<float>(j, i) = maskSimilarityScore;

    return;
  }

}
