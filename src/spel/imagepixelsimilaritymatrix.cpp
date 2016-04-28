#include "imagepixelsimilaritymatrix.hpp"

namespace SPEL
{
  ImagePixelSimilarityMatrix::ImagePixelSimilarityMatrix(void) noexcept : ImageSimilarityMatrix()
  {
    id = 0x4950534D;
  }

  ImagePixelSimilarityMatrix::ImagePixelSimilarityMatrix(const ImagePixelSimilarityMatrix & m) noexcept : ImageSimilarityMatrix(m)
  {    
  }

  ImagePixelSimilarityMatrix::ImagePixelSimilarityMatrix(const std::vector<Frame*>& frames) : ImagePixelSimilarityMatrix()
  {
    //by default use colour
    buildImageSimilarityMatrix(frames);
  }

  ImagePixelSimilarityMatrix::ImagePixelSimilarityMatrix(ImagePixelSimilarityMatrix && m) noexcept : ImageSimilarityMatrix(std::move(m))
  {
  }

  ImagePixelSimilarityMatrix::~ImagePixelSimilarityMatrix(void) noexcept
  {    
  }

  ImagePixelSimilarityMatrix & ImagePixelSimilarityMatrix::operator=(const ImagePixelSimilarityMatrix & s) noexcept
  {
    ImageSimilarityMatrix::operator=(s);
    return *this;
  }
  
  void ImagePixelSimilarityMatrix::computeISMcell(Frame* left, Frame* right, const int maxFrameHeight)
  {
    auto i = left->getID(), j = right->getID();
    //only do this loop if
    if (j > i)
      return;
    if (i == j)
    {
      imageShiftMatrix.at<cv::Point2f>(i, j) = cv::Point2f(0.0f, 0.0f);
      imageSimilarityMatrix.at<float>(i, j) = 0;
      return;
    }
    //load images, compute similarity, store to matrix
    auto imgMatOne = left->getImage();
    auto imgMatTwo = right->getImage();

    auto maskMatOne = left->getMask();
    auto maskMatTwo = right->getMask();

    auto dX = calculateDistance(maskMatOne, maskMatTwo);

    //for real image coords, these points are actually reversed
    imageShiftMatrix.at<cv::Point2f>(i, j) = cv::Point2f(dX.x, dX.y); // 14.04.16  Replaced x and y
    imageShiftMatrix.at<cv::Point2f>(j, i) = cv::Point2f(-dX.x, -dX.y); // 14.04.16  Replaced x and y

    auto similarityScore = 0.0f;
    for (auto y = 0; y < maskMatOne.rows; ++y) // 14.04.16  Replaced x and y
    {
      for (auto x = 0; x < maskMatOne.cols; ++x) // 14.04.16  Replaced x and y
      {
        auto mintensityOne = maskMatOne.at<uchar>(y, x); // 14.04.16  Replaced x and y 
        auto darkPixel = mintensityOne < 10; //if all intensities are zero
        auto mOne = 0, mTwo = 0;

        //apply the transformation
        auto xTwo = x + dX.x;
        auto yTwo = y + dX.y;

        //now check bounds
        auto blueOne = 0;
        auto greenOne = 0;
        auto redOne = 0;

        auto blueTwo = 255;
        auto greenTwo = 255;
        auto redTwo = 255;

        //compare points
        if (!darkPixel)
        {
          mOne = 1;
          auto intensityOne = imgMatOne.at<cv::Vec4b>(y, x); // 14.04.16  Replaced x and y
          blueOne = intensityOne.val[0];
          greenOne = intensityOne.val[1];
          redOne = intensityOne.val[2];
        }
        if (xTwo < imgMatTwo.cols && xTwo >= 0 && yTwo < imgMatTwo.rows && yTwo >= 0) // 14.04.16  Replaced "cols" and "rows"
        {
          auto mintensityTwo = static_cast<cv::Scalar>(maskMatTwo.at<uchar>(yTwo, xTwo)); // 14.04.16  Replaced x and y
          auto blackPixel = mintensityTwo.val[0] < 10; //if all intensities are zero

          if (!blackPixel)
          {
            mTwo = 1;
            auto intensityTwo = imgMatTwo.at<cv::Vec4b>(yTwo, xTwo); // 14.04.16  Replaced x and y
            blueTwo = intensityTwo.val[0];
            greenTwo = intensityTwo.val[1];
            redTwo = intensityTwo.val[2];
          }
        }

        if (mOne!=mTwo) //maximum penalty if they are different      - 14.04.16  Replaced "if (mOne^mTwo)" to "if (mOne!=mTwo)"
          similarityScore += pow(255, 2) + pow(255, 2) + pow(255, 2); //square of absolute difference
        if (mOne==1 && mTwo==1) //if both are in mask, or outside of mask       - 14.04.16  Replaced "else" to "if (mOne==1 && mTwo==1)"
          similarityScore += pow(redOne - redTwo, 2) + pow(greenOne - greenTwo, 2) +
          pow(blueOne - blueTwo, 2); //square of absolute difference
      }
    }

    imageSimilarityMatrix.at<float>(i, j) = similarityScore;
    imageSimilarityMatrix.at<float>(j, i) = similarityScore;

    return;
  }
 
  // Alternative MSM
  ImagePixelSimilarityMatrix::ImagePixelSimilarityMatrix(const std::vector<Frame*>& frames, int Erode, int Dilate) : ImagePixelSimilarityMatrix()
  {
    buildImageSimilarityMatrix(frames, Erode, Dilate);
  }
  //
}
