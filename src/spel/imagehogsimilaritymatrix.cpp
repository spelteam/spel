#include "imagehogsimilaritymatrix.hpp"

namespace SPEL
{
  ImageHogSimilarityMatrix::ImageHogSimilarityMatrix(void)  : ImageSimilarityMatrix()
  {
    id = 0x4948534D;
  }
  ImageHogSimilarityMatrix::ImageHogSimilarityMatrix(const ImageHogSimilarityMatrix & m)  : ImageSimilarityMatrix(m)
  {
  }
  ImageHogSimilarityMatrix::ImageHogSimilarityMatrix(const std::vector<Frame*>& frames) : ImageHogSimilarityMatrix()
  {
    buildImageSimilarityMatrix(frames);
  }
  ImageHogSimilarityMatrix::ImageHogSimilarityMatrix(ImageHogSimilarityMatrix && m)  : ImageSimilarityMatrix(std::move(m))
  {
  }
  ImageHogSimilarityMatrix::~ImageHogSimilarityMatrix(void) 
  {
  }
  ImageHogSimilarityMatrix & ImageHogSimilarityMatrix::operator=(const ImageHogSimilarityMatrix & s) 
  {
    ImageSimilarityMatrix::operator=(s);
    return *this;
  }
  void ImageHogSimilarityMatrix::calculateROI(Frame * frame, cv::Point2i & topLeft, cv::Point2i & bottomRight) const 
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

  // Added 31.07.16
  // Expand the ROI to new size 
  cv::Rect resizeROI(cv::Rect ROI, cv::Size NewROISize, cv::Size ImageSize = cv::Size(0,0))
  {
    cv::Point2f p0 = cv::Point2f(ROI.x, ROI.y) - 0.5f*cv::Point2f(NewROISize.width - ROI.width, NewROISize.height - ROI.height);

    if (p0.x < 0) DebugMessage("ROI.x0 < 0", 5);
    if (p0.y < 0) DebugMessage("ROI.y0 < 0", 5);

    if (ImageSize != cv::Size(0, 0))
    {
      if (p0.x + ROI.width > ImageSize.width)  DebugMessage("ROI.x1 > Image cols", 5);
      if (p0.y + ROI.height > ImageSize.height) DebugMessage("ROI.y1 > Image rows", 5);
    }

    return cv::Rect(p0, NewROISize);
  }

  // Added 31.07.16
  // Aligning the ROI to the HOGDetector blockSize
  cv::Rect extendROI(cv::Rect ROI, cv::Size blockSize)
  {
    int width = blockSize.width  * ceil(static_cast<float>(ROI.width) / static_cast<float>(blockSize.width));
    int height = blockSize.height  * ceil(static_cast<float>(ROI.height) / static_cast<float>(blockSize.height));

    cv::Point2f p0 = cv::Point2f(ROI.x, ROI.y) - 0.5f*cv::Point2f(width - ROI.width, height - ROI.height);
    return cv::Rect(p0, cv::Size(width, height));
  }

  // Added 31.07.16
  // Copying image from normal or bad ROI to new image
  cv::Mat copyROI(cv::Mat image, cv::Rect ROI)
  {
    cv::Mat NewImage;
    cv::Size size = image.size();
    if (ROI.x < 0 || ROI.y < 0 || ROI.x + ROI.width > size.width || ROI.y + ROI.height > size.height)
    {
      cv::Point2i p0(std::max(0, ROI.x), std::max(0, ROI.y));
      cv::Point2i p1 = cv::Point2i(std::min(size.width, ROI.x + ROI.width), std::min(size.height, ROI.y + ROI.height)) - cv::Point2i(1, 1);
      NewImage = cv::Mat::zeros(ROI.height, ROI.width, CV_8UC3);

      int x0 = 0, y0 = 0;
      if (ROI.x < 0) x0 = -ROI.x;
      if (ROI.y < 0) y0 = -ROI.y;

      for (int y = p0.y; y < p1.y; y++)
        for (int x = p0.x; x < p1.x; x++)
          NewImage.at<cv::Vec3b>(y0 + y - p0.y, x0 + x - p0.x) = image.at<cv::Vec3b>(y, x);
    }
    else
    NewImage = image(ROI).clone();

    return NewImage;
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

    cv::Mat imgMatNewOne, imgMatNewTwo;

    // Disabled 31.07.16
    /*
    if (bottomRightOne.x - topLeftOne.x > bottomRightTwo.x - topLeftTwo.x)
      extendSize(topLeftTwo.x, topLeftOne.x, bottomRightTwo.x, bottomRightOne.x, imgMatTwo.rows, blockSize.height);
    else if (bottomRightOne.x - topLeftOne.x < bottomRightTwo.x - topLeftTwo.x)
      extendSize(topLeftOne.x, topLeftTwo.x, bottomRightOne.x, bottomRightTwo.x, imgMatOne.rows, blockSize.height);

    if (bottomRightOne.y - topLeftOne.y > bottomRightTwo.y - topLeftTwo.y)
      extendSize(topLeftTwo.y, topLeftOne.y, bottomRightTwo.y, bottomRightOne.y, imgMatTwo.cols, blockSize.width);
    else if (bottomRightOne.y - topLeftOne.y < bottomRightTwo.y - topLeftTwo.y)
      extendSize(topLeftOne.y, topLeftTwo.y, bottomRightOne.y, bottomRightTwo.y, imgMatOne.cols, blockSize.width);

    try
    {
      imgMatNewOne = imgMatOne(cv::Rect(topLeftOne.x, topLeftOne.y, bottomRightOne.x - topLeftOne.x, bottomRightOne.y - topLeftOne.y )).clone(); // Repalaced x and y 30.07.16
      imgMatNewTwo = imgMatTwo(cv::Rect(topLeftTwo.x, topLeftTwo.y, bottomRightTwo.x - topLeftTwo.x, bottomRightTwo.y - topLeftTwo.y)).clone(); // Repalaced x and y 30.07.16
    }*/

    // Added 31.07.16: begin
    cv::Rect ROI1(topLeftOne, bottomRightOne);
    cv::Rect ROI2(topLeftTwo, bottomRightTwo);

    cv::Size CombinedROI_size(std::max(ROI1.width, ROI2.width), std::max(ROI1.height, ROI2.height));

    ROI1 = resizeROI(ROI1, CombinedROI_size);
    ROI2 = resizeROI(ROI2, CombinedROI_size);

    ROI1 = extendROI(ROI1, blockSize);
    ROI2 = extendROI(ROI2, blockSize);

    try
    {
      imgMatNewOne = copyROI( imgMatOne, ROI1);
      imgMatNewTwo = copyROI(imgMatTwo, ROI2);
    }
    // end
    catch (std::exception ex)
    {
      left->UnloadAll();
      right->UnloadAll();
      std::stringstream ss;
      ss << "It seems that coordinates are wrong and you need to invert x and y or width and height: " << ex.what();
      DebugMessage(ss.str(), 1);
      throw std::out_of_range(ss.str());
    }

    auto imgMatSizeOne = imgMatNewOne.size();
    auto imgMatSizeTwo = imgMatNewTwo.size();

    if (imgMatSizeOne != imgMatSizeTwo)
    {
      left->UnloadAll();
      right->UnloadAll();
      imgMatNewOne.release();
      imgMatNewTwo.release();
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
      left->UnloadAll();
      right->UnloadAll();
      imgMatNewOne.release();
      imgMatNewTwo.release();
      std::stringstream ss;
      ss << "Results have different sizes: one=" << gradientStrengthsOne.size() << "\ttwo=" << gradientStrengthsTwo.size();
      DebugMessage(ss.str(), 1);
      throw std::logic_error(ss.str());
    }
    for (auto i = 0; i < gradientStrengthsOne.size(); ++i)
    {
      if (gradientStrengthsOne.at(i).size() != gradientStrengthsTwo.at(i).size())
      {
        left->UnloadAll();
        right->UnloadAll();
        imgMatNewOne.release();
        imgMatNewTwo.release();
        std::stringstream ss;
        ss << "Results have different sizes: one=" << gradientStrengthsOne.at(i).size() << "\ttwo=" << gradientStrengthsTwo.at(i).size();
        DebugMessage(ss.str(), 1);
        throw std::logic_error(ss.str());
      }
      for (auto j = 0; j < gradientStrengthsOne.at(i).size(); ++j)
      {
        if (gradientStrengthsOne.at(i).at(j).size() != gradientStrengthsTwo.at(i).at(j).size())
        {
          left->UnloadAll();
          right->UnloadAll();
          imgMatNewOne.release();
          imgMatNewTwo.release();
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
    left->UnloadAll();
    right->UnloadAll();

    return;
  }
}
