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
  void ImageHogSimilarityMatrix::computeISMcell(const Frame * left, const Frame * right, const int maxFrameHeight)
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

    auto imgMatSizeOne = imgMatOne.size();
    auto imgMatSizeTwo = imgMatTwo.size();

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
    imageShiftMatrix.at<cv::Point2f>(i, j) = cv::Point2f(dX.y, dX.x);
    imageShiftMatrix.at<cv::Point2f>(j, i) = cv::Point2f(-dX.y, -dX.x);

    auto similarityScore = 0.0f;
    std::vector <float> descriptors;

    auto gradientStrengthsOne = HogDetector::calculateHog(imgMatOne, descriptors, imgMatSizeOne, blockSize, blockStride, cellSize, nbins, derivAperture, wndSigma, histogramNormType, thresholdL2hys, gammaCorrection, nlevels);
    auto gradientStrengthsTwo = HogDetector::calculateHog(imgMatTwo, descriptors, imgMatSizeTwo, blockSize, blockStride, cellSize, nbins, derivAperture, wndSigma, histogramNormType, thresholdL2hys, gammaCorrection, nlevels);

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

    return;
  }
}
