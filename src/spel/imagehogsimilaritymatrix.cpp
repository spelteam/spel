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
  ImageHogSimilarityMatrix & ImageHogSimilarityMatrix::operator=(const ImageHogSimilarityMatrix & s) noexcept
  {
    ImageSimilarityMatrix::operator=(s);
    return *this;
  }
  void ImageHogSimilarityMatrix::calculateROI(Frame * frame, cv::Point2i & topLeft, cv::Point2i & bottomRight) const noexcept
  {
    auto image = frame->getMask();

    auto xmin = image.rows;
    auto ymin = image.cols;
    auto xmax = 0;
    auto ymax = 0;

    for (auto y = 0; y < image.rows; ++y) // 09.04.16 - replaced x and y
    {
      for (auto x = 0; x < image.cols; ++x)// 09.04.16 - replaced x and y
      {
        auto darkPixel = static_cast<cv::Scalar>(image.at<uchar>(y, x))[0] < 10; // 09.04.16 - replaced x and y
        if (!darkPixel) // 09.04.16 - changed "if (darkPixel)" to "if (!darkPixel)"
        {
          if (xmin > x) xmin = x;
          if (ymin > y) ymin = y;
          if (xmax < x) xmax = x;
          if (ymax < y) ymax = y;
        }
      }
    }
    topLeft.x = xmin;
    topLeft.y = ymin;
    bottomRight.x = xmax;
    bottomRight.y = ymax;
  }
  int8_t ImageHogSimilarityMatrix::extendSize(int & topLeftMin, int & topLeftMax, int & bottomRightMin, int & bottomRightMax, const int maxsize, const int add) const
  {
    // extend ROI to the biggest one
    auto diff = (bottomRightMax - topLeftMax) - (bottomRightMin - topLeftMin);
    auto half = diff / 2;
    // try to extend left
    if (topLeftMin - half >= 0)
    {
      topLeftMin -= half;
      diff -= half;
    }
    else
    {
      diff -= topLeftMin;
      topLeftMin = 0;
    }
    // try to extend right
    if (bottomRightMin + diff < maxsize)
    {
      bottomRightMin += diff;
      diff = 0;
    }
    else
    {
      diff -= (maxsize - bottomRightMin - 1);
      bottomRightMin = maxsize - 1;
    }
    // if we still have smth to extend lets try one more time
    if (diff != 0)
    {
      if (topLeftMin - diff >= 0)
        topLeftMin -= diff;
      else
      {
        std::stringstream ss;
        ss << "Can't compare frames: ROI sizes too different";
        DebugMessage(ss.str(), 1);
        throw std::logic_error(ss.str());
      }
    }

    // try to extend ROI with the additional border cells
    if (topLeftMin - add >= 0 && topLeftMax - add >= 0 && bottomRightMin + add < maxsize && bottomRightMax + add < maxsize)
    {
      topLeftMin -= add;
      topLeftMax -= add;
      bottomRightMin += add;
      bottomRightMax += add;
      DebugMessage("Additional border cell added successfully", 5);
      return 0;
    }    
    else if (topLeftMin - (add / 2) >= 0 && topLeftMax - (add / 2) >= 0 && bottomRightMin + (add - add / 2) < maxsize && bottomRightMax + (add / add / 2) < maxsize)
    {
      topLeftMin -= (add / 2);
      topLeftMax -= (add / 2);
      bottomRightMin += (add - add / 2);
      bottomRightMax += (add - add / 2);
      DebugMessage("Additional border cell added partially", 5);
      return 1;
    }
    else
    {
      DebugMessage("Additional border cell was not set", 5);
      return -1;
    }
  }
  void ImageHogSimilarityMatrix::computeISMcell(Frame * left, Frame * right, const int maxFrameHeight)
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
    //load images, compute similarity, store to matrix
    auto imgMatOne = left->getImage();
    auto imgMatTwo = right->getImage();

    cv::Point2i topLeftOne, topLeftTwo, bottomRightOne, bottomRightTwo;

    calculateROI(left, topLeftOne, bottomRightOne);
    calculateROI(right, topLeftTwo, bottomRightTwo);

    if (bottomRightOne.x - topLeftOne.x > bottomRightTwo.x - topLeftTwo.x)
      extendSize(topLeftTwo.x, topLeftOne.x, bottomRightTwo.x, bottomRightOne.x, imgMatTwo.rows, blockSize.height);
    else if (bottomRightOne.x - topLeftOne.x < bottomRightTwo.x - topLeftTwo.x)
      extendSize(topLeftOne.x, topLeftTwo.x, bottomRightOne.x, bottomRightTwo.x, imgMatOne.rows, blockSize.height);

    if (bottomRightOne.y - topLeftOne.y > bottomRightTwo.y - topLeftTwo.y)
      extendSize(topLeftTwo.y, topLeftOne.y, bottomRightTwo.y, bottomRightOne.y, imgMatTwo.cols, blockSize.width);
    else if (bottomRightOne.y - topLeftOne.y < bottomRightTwo.y - topLeftTwo.y)
      extendSize(topLeftOne.y, topLeftTwo.y, bottomRightOne.y, bottomRightTwo.y, imgMatOne.cols, blockSize.width);

    cv::Mat imgMatNewOne, imgMatNewTwo;

    try
    {
      imgMatNewOne = imgMatOne(cv::Rect(topLeftOne.y, topLeftOne.x, bottomRightOne.y - topLeftOne.y, bottomRightOne.x - topLeftOne.x)).clone();
      imgMatNewTwo = imgMatTwo(cv::Rect(topLeftTwo.y, topLeftTwo.x, bottomRightTwo.y - topLeftTwo.y, bottomRightTwo.x - topLeftTwo.x)).clone();
    }
    catch (std::exception ex)
    {
      std::stringstream ss;
      ss << "It seems that coordinates are wrong and you need to invert x and y or width and height: " << ex.what();
      DebugMessage(ss.str(), 1);
      throw std::out_of_range(ss.str());
    }

    auto imgMatSizeOne = imgMatNewOne.size();
    auto imgMatSizeTwo = imgMatNewTwo.size();

    if (imgMatSizeOne != imgMatSizeTwo)
    {
      std::stringstream ss;
      ss << "Frames have different sizes";
      DebugMessage(ss.str(), 1);
      throw std::logic_error(ss.str());
    }

    auto maskMatOne = left->getMask();
    auto maskMatTwo = right->getMask();

    auto dX = calculateDistance(maskMatOne, maskMatTwo);

    //for real image coords, these points are actually reversed
    imageShiftMatrix.at<cv::Point2f>(i, j) = cv::Point2f(dX.x, dX.y);  // 14.06.16 Replaced x and y
    imageShiftMatrix.at<cv::Point2f>(j, i) = cv::Point2f(-dX.x, -dX.y);  // 14.06.16 Replaced x and y

    auto similarityScore = 0.0f;
    std::vector <float> descriptors;

    auto gradientStrengthsOne = HogDetector::calculateHog(imgMatNewOne, descriptors, imgMatSizeOne, blockSize, blockStride, cellSize, nbins, derivAperture, wndSigma, histogramNormType, thresholdL2hys, gammaCorrection, nlevels);
    auto gradientStrengthsTwo = HogDetector::calculateHog(imgMatNewTwo, descriptors, imgMatSizeTwo, blockSize, blockStride, cellSize, nbins, derivAperture, wndSigma, histogramNormType, thresholdL2hys, gammaCorrection, nlevels);

    if (gradientStrengthsOne.size() != gradientStrengthsTwo.size())
    {
      std::stringstream ss;
      ss << "Results have different sizes: one=" << gradientStrengthsOne.size() << "\ttwo=" << gradientStrengthsTwo.size();
      DebugMessage(ss.str(), 1);
      throw std::logic_error(ss.str());
    }
    for (auto i = 0; i < gradientStrengthsOne.size(); ++i)
    {
      if (gradientStrengthsOne.at(i).size() != gradientStrengthsTwo.at(i).size())
      {
        std::stringstream ss;
        ss << "Results have different sizes: one=" << gradientStrengthsOne.at(i).size() << "\ttwo=" << gradientStrengthsTwo.at(i).size();
        DebugMessage(ss.str(), 1);
        throw std::logic_error(ss.str());
      }
      for (auto j = 0; j < gradientStrengthsOne.at(i).size(); ++j)
      {
        if (gradientStrengthsOne.at(i).at(j).size() != gradientStrengthsTwo.at(i).at(j).size())
        {
          std::stringstream ss;
          ss << "Results have different sizes: one=" << gradientStrengthsOne.at(i).at(j).size() << "\ttwo=" << gradientStrengthsTwo.at(i).at(j).size();
          DebugMessage(ss.str(), 1);
          throw std::logic_error(ss.str());
        }
        for (auto k = 0; k < gradientStrengthsOne.at(i).at(j).size(); ++k)
          similarityScore += pow(gradientStrengthsOne.at(i).at(j).at(k) - gradientStrengthsTwo.at(i).at(j).at(k), 2);
      }
    }

    imageSimilarityMatrix.at<float>(i, j) = similarityScore;
    imageSimilarityMatrix.at<float>(j, i) = similarityScore;

    imgMatNewOne.release();
    imgMatNewTwo.release();

    return;
  }
}
