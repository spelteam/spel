#include "imagepixelsimilaritymatrix.hpp"

namespace SPEL
{
  ImagePixelSimilarityMatrix::ImagePixelSimilarityMatrix(void) noexcept : ImageSimilarityMatrix()
  {
    id = 0x4950534D;
  }

  ImagePixelSimilarityMatrix::ImagePixelSimilarityMatrix(const ImagePixelSimilarityMatrix & m) noexcept
  {
    //by default use colour
    imageSimilarityMatrix = m.imageSimilarityMatrix;
    imageShiftMatrix = m.imageShiftMatrix;
  }

  ImagePixelSimilarityMatrix::ImagePixelSimilarityMatrix(const std::vector<Frame*>& frames) noexcept
  {
    //by default use colour
    buildImageSimilarityMatrix(frames);
  }

  ImagePixelSimilarityMatrix::ImagePixelSimilarityMatrix(ImagePixelSimilarityMatrix && m) noexcept
    : imageSimilarityMatrix(std::move(m.imageSimilarityMatrix)),
    imageShiftMatrix(std::move(m.imageShiftMatrix))
  {
  }

  ImagePixelSimilarityMatrix::~ImagePixelSimilarityMatrix(void) noexcept
  {
    imageSimilarityMatrix.release();
    imageShiftMatrix.release();
  }

  bool ImagePixelSimilarityMatrix::operator==(const ImagePixelSimilarityMatrix & s) const noexcept
  {
    auto result = static_cast<cv::Mat>(imageSimilarityMatrix == s.imageSimilarityMatrix);

    auto res = true;

    //if every element is 1
    for (auto i = 0; i < result.rows; ++i)
    {
      for (auto j = 0; j < result.cols; ++j)
      {
        if (result.at<float>(i, j) == 0)
          res = false;
      }
    }
    return res;
  }

  bool ImagePixelSimilarityMatrix::operator!=(const ImagePixelSimilarityMatrix & s) const noexcept
  {
    return !(*this == s);
  }

  ImageSimilarityMatrix & ImagePixelSimilarityMatrix::operator=(const ImagePixelSimilarityMatrix & s) noexcept
  {
    if (&s == this)
      return *this;

    imageSimilarityMatrix = s.imageSimilarityMatrix;
    imageShiftMatrix = s.imageShiftMatrix;

    return *this;
  }

  ImageSimilarityMatrix & ImagePixelSimilarityMatrix::operator=(ImagePixelSimilarityMatrix && s) noexcept
  {
    imageSimilarityMatrix = std::move(s.imageSimilarityMatrix);
    imageShiftMatrix = std::move(s.imageShiftMatrix);
    return *this;
  }

  void ImagePixelSimilarityMatrix::computeISMcell(const Frame* left, const Frame* right, const int maxFrameHeight)
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

    auto maskMatOne = left->getMask();
    auto maskMatTwo = right->getMask();

    auto dX = calculateDistance(maskMatOne, maskMatTwo);

    //for real image coords, these points are actually reversed
    imageShiftMatrix.at<cv::Point2f>(i, j) = cv::Point2f(dX.y, dX.x);
    imageShiftMatrix.at<cv::Point2f>(j, i) = cv::Point2f(-dX.y, -dX.x);

    auto similarityScore = 0.0f;
    for (auto x = 0; x < maskMatOne.rows; ++x)
    {
      for (auto y = 0; y < maskMatOne.cols; ++y)
      {
        auto mintensityOne = maskMatOne.at<uchar>(x, y);
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
          auto intensityOne = imgMatOne.at<cv::Vec4b>(x, y);
          blueOne = intensityOne.val[0];
          greenOne = intensityOne.val[1];
          redOne = intensityOne.val[2];
        }
        if (xTwo < imgMatTwo.rows && xTwo >= 0 && yTwo < imgMatTwo.cols && yTwo >= 0)
        {
          auto mintensityTwo = static_cast<cv::Scalar>(maskMatTwo.at<uchar>(xTwo, yTwo));
          auto blackPixel = mintensityTwo.val[0] < 10; //if all intensities are zero

          if (!blackPixel)
          {
            mTwo = 1;
            auto intensityTwo = imgMatTwo.at<cv::Vec4b>(xTwo, yTwo);
            blueTwo = intensityTwo.val[0];
            greenTwo = intensityTwo.val[1];
            redTwo = intensityTwo.val[2];
          }
        }

        // maskSimilarityScore+=std::abs(mOne-mTwo);

        if (mOne^mTwo) //maximum penalty if they are different
          similarityScore += pow(255, 2) + pow(255, 2) + pow(255, 2); //square of absolute difference
        else //if both are in mask, or outside of mask
          similarityScore += pow(redOne - redTwo, 2) + pow(greenOne - greenTwo, 2) +
          pow(blueOne - blueTwo, 2); //square of absolute difference
      }
    }

    imageSimilarityMatrix.at<float>(i, j) = similarityScore;
    imageSimilarityMatrix.at<float>(j, i) = similarityScore;

    return;
  }

  cv::Point2f ImagePixelSimilarityMatrix::calculateDistance(const cv::Mat &imgMatOne, const cv::Mat &imgMatTwo) const noexcept
  {
    cv::Point2f cOne, cTwo;
    auto mSizeOne = 0.0f, mSizeTwo = 0.0f;

    for (auto x = 0; x < imgMatOne.rows; ++x)
    {
      for (auto y = 0; y < imgMatOne.cols; ++y)
      {
        auto intensity = static_cast<cv::Scalar>(imgMatOne.at<uchar>(x, y));
        auto mintensity = static_cast<cv::Scalar>(imgMatTwo.at<uchar>(x, y));

        auto darkPixel = intensity.val[0] < 10;
        auto blackPixel = mintensity.val[0] < 10; //if all intensities are zero

        if (!darkPixel) //if the pixel is non-black for maskOne
        {
          cOne += cv::Point2f(x, y);
          mSizeOne++;
        }

        if (!blackPixel) //if the pixel is non-black for maskOne
        {
          cTwo += cv::Point2f(x, y);
          mSizeTwo++;
        }
      }
    }
    cOne = cOne * (1.0 / mSizeOne);
    cTwo = cTwo * (1.0 / mSizeTwo);

    //cOne and cTwo now have the centres
    return cTwo - cOne;
  }

  void ImagePixelSimilarityMatrix::computeMSMcell(const Frame* left, const Frame* right, const int maxFrameHeight)
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

    imageShiftMatrix.at<cv::Point2f>(i, j) = cv::Point2f(dX.y, dX.x);
    imageShiftMatrix.at<cv::Point2f>(j, i) = cv::Point2f(-dX.y, -dX.x);

    auto maskSimilarityScore = 0.0f;
    auto intersectCount = 0, unionCount = 0;
    for (auto x = 0; x < maskMatOne.rows; ++x)
    {
      for (auto y = 0; y < maskMatOne.cols; ++y)
      {
        auto mintensityOne = maskMatOne.at<uchar>(j, i);
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

  //return the size of the ISM
  uint32_t ImagePixelSimilarityMatrix::size() const noexcept
  {
    return imageSimilarityMatrix.rows;
  }

  void ImagePixelSimilarityMatrix::buildMaskSimilarityMatrix(const std::vector<Frame*>& frames, const int maxFrameHeight)
  {
    //create matrices and fill with zeros
    imageSimilarityMatrix.release();
    imageShiftMatrix.release();

    imageSimilarityMatrix.create(frames.size(), frames.size(), cv::DataType<float>::type);
    imageShiftMatrix.create(frames.size(), frames.size(), cv::DataType<cv::Point2f>::type);

    for (auto i = 0; i < frames.size(); ++i)
      for (auto j = 0; j < frames.size(); ++j)
        imageSimilarityMatrix.at<float>(i, j) = 0;
    //store the futures
    std::vector<std::future<void> > futures;

    //set-up finished

    //compute mask centroid offsets
    for (auto i = 0; i < frames.size(); ++i)
    {
      for (auto j = 0; j < frames.size(); ++j)
      {
        auto left = frames[i];
        auto right = frames[j];
        futures.push_back(std::async(&ImagePixelSimilarityMatrix::computeMSMcell, this, left, right, maxFrameHeight));
      }
    }

    for (auto &e : futures)
      e.get();

    return;
  }

  void ImagePixelSimilarityMatrix::buildImageSimilarityMatrix(const std::vector<Frame*>& frames, const int maxFrameHeight)
  {
    DebugMessage("building ISM matrix", 5);
    //create matrices and fill with zeros
    imageSimilarityMatrix.create(frames.size(), frames.size(), cv::DataType<float>::type);
    imageShiftMatrix.create(frames.size(), frames.size(), cv::DataType<cv::Point2f>::type);

    for (auto i = 0; i < frames.size(); ++i)
    {
      for (auto j = 0; j < frames.size(); ++j)
      {
        imageSimilarityMatrix.at<float>(i, j) = 0;
        imageShiftMatrix.at<cv::Point2f>(i, j) = cv::Point2f(0, 0);
      }
    }

    //compute mask centroid offsets

    //store the futures
    std::vector<std::future<void> > futures;

    for (auto i = 0; i < frames.size(); ++i)
    {
      for (auto j = 0; j < frames.size(); ++j)
      {
        auto left = frames[i];
        auto right = frames[j];
        futures.push_back(std::async(&ImagePixelSimilarityMatrix::computeISMcell, this, left, right, maxFrameHeight));
      }
    }
    //get all futures

    for (auto &e : futures)
      e.get();

    return;
  }

  bool ImagePixelSimilarityMatrix::read(const std::string &filename) noexcept
  {
    std::ifstream in(filename.c_str());
    if (in.is_open())
    {
      int magic;
      in >> magic;
      if (magic != id)
      {
        DebugMessage("Wrong filetype", 1);
        return false;
      }

      int size;
      in >> size;

      imageSimilarityMatrix.release();
      imageShiftMatrix.release();

      imageSimilarityMatrix.create(size, size, cv::DataType<float>::type);
      imageShiftMatrix.create(size, size, cv::DataType<cv::Point2f>::type);

      for (auto i = 0; i < imageSimilarityMatrix.rows; ++i)
      {
        for (auto j = 0; j < imageSimilarityMatrix.cols; ++j)
        {
          float score;
          in >> score;
          imageSimilarityMatrix.at<float>(i, j) = score;
        }
      }

      //@FIX fix this if it is imported

      for (auto i = 0; i < imageShiftMatrix.rows; ++i)
      {
        for (auto j = 0; j < imageShiftMatrix.cols; ++j)
        {
          float x, y;
          in >> x >> y;
          imageShiftMatrix.at<cv::Point2f>(i, j) = cv::Point2f(x, y);
        }
      }
      return true;
    }
    else
    {
      std::stringstream ss;
      ss << "Could not open " << filename << " for reading.";
      DebugMessage(ss.str(), 1);
      return false;
    }
  }

  bool ImagePixelSimilarityMatrix::write(const std::string &filename) const noexcept
  {
    std::ofstream out(filename.c_str());
    if (out.is_open())
    {
      out << id << std::endl;

      out << imageSimilarityMatrix.rows << std::endl; //size
      for (int i = 0; i < imageSimilarityMatrix.rows; ++i)
      {
        for (int j = 0; j < imageSimilarityMatrix.cols; ++j)
        {
          out << imageSimilarityMatrix.at<float>(i, j) << " ";
        }
        out << std::endl;
      }

      for (int i = 0; i < imageShiftMatrix.rows; ++i)
      {
        for (int j = 0; j < imageShiftMatrix.cols; ++j)
        {
          out << imageShiftMatrix.at<cv::Point2f>(i, j).x << " " << imageShiftMatrix.at<cv::Point2f>(i, j).y << " ";
        }
        out << std::endl;
      }
      return true;
    }
    else
    {
      std::stringstream ss;
      ss << "Could not open " << filename << " for writing.";
      DebugMessage(ss.str(), 1);
      return false;
    }
  }

  //find the non-zero minimum in the image similarity matrix
  float ImagePixelSimilarityMatrix::min() const noexcept
  {
    auto min = FLT_MAX;
    for (auto i = 0; i < imageSimilarityMatrix.rows; ++i)
    {
      for (auto j = 0; j < imageSimilarityMatrix.cols; ++j)
      {
        auto val = imageSimilarityMatrix.at<float>(i, j);
        if (val != 0 && val < min && i != j)
          min = val;
      }
    }
    return min;
  }

  //find the non-zero minimum in the image similarity matrix
  float ImagePixelSimilarityMatrix::max() const noexcept
  {
    auto max = FLT_MIN;
    for (auto i = 0; i < imageSimilarityMatrix.rows; ++i)
    {
      for (auto j = 0; j < imageSimilarityMatrix.cols; ++j)
      {
        auto val = imageSimilarityMatrix.at<float>(i, j);
        if (val > max)
          max = val;
      }
    }
    return max;
  }

  //find the non-zero minimum in the image similarity matrix
  float ImagePixelSimilarityMatrix::mean() const
  {
    auto sum = 0.0f;
    auto count = 0;
    for (auto i = 0; i < imageSimilarityMatrix.rows; ++i)
    {
      for (auto j = 0; j < imageSimilarityMatrix.cols; ++j)
      {
        auto val = imageSimilarityMatrix.at<float>(i, j);
        if (val != 0 && i != j)
        {
          count++;
          sum += val;
        }
      }
    }
    if (count == 0)
    {
      std::stringstream ss;
      ss << "Division by zero: count can't be zero";
      DebugMessage(ss.str(), 1);
      throw std::out_of_range(ss.str());
    }
    auto mean = sum / static_cast<float>(count);
    return mean;
  }

  float ImagePixelSimilarityMatrix::stddev() const
  {
    auto mean = this->mean();
    auto sum = 0.0f;
    auto count = 0;
    for (auto i = 0; i < imageSimilarityMatrix.rows; ++i)
    {
      for (auto j = 0; j < imageSimilarityMatrix.cols; ++j)
      {
        auto val = imageSimilarityMatrix.at<float>(i, j);
        if (val != 0 && i != j)
        {
          count++;
          sum += pow(val - mean, 2);
        }
      }
    }
    if (count == 0)
    {
      std::stringstream ss;
      ss << "Division by zero: count can't be zero";
      DebugMessage(ss.str(), 1);
      throw std::out_of_range(ss.str());
    }
    auto sd = sum / static_cast<float>(count);
    return sqrt(sd);
  }

  //get ISM value at (row, col)
  float ImagePixelSimilarityMatrix::at(const int row, const int col) const
  {
    if (row >= imageSimilarityMatrix.rows)
    {
      std::stringstream ss;
      ss << "ISM contains " << imageSimilarityMatrix.rows << " rows, cannot request row " << row;
      DebugMessage(ss.str(), 1);
      throw std::out_of_range(ss.str());
    }
    if (col >= imageSimilarityMatrix.cols)
    {
      std::stringstream ss;
      ss << "ISM contains " << imageSimilarityMatrix.cols << " cols, cannot request col " << col;
      DebugMessage(ss.str(), 1);
      throw std::out_of_range(ss.str());
    }
    return imageSimilarityMatrix.at<float>(row, col);
  }

  //get ISM value at (row, col)
  cv::Point2f ImagePixelSimilarityMatrix::getShift(const int row, const int col) const
  {
    if (row >= imageShiftMatrix.rows)
    {
      std::stringstream ss;
      ss << "Shift Matrix contains " << imageShiftMatrix.rows << " rows, cannot request row " << row;
      DebugMessage(ss.str(), 1);
      throw std::out_of_range(ss.str());
    }
    if (col >= imageShiftMatrix.cols)
    {
      std::stringstream ss;
      ss << "Shift Matrix contains " << imageShiftMatrix.cols << " cols, cannot request col " << col;
      DebugMessage(ss.str(), 1);
      throw std::out_of_range(ss.str());
    }
    return imageShiftMatrix.at<cv::Point2f>(row, col);
  }

  //get cost for path through ISM
  float ImagePixelSimilarityMatrix::getPathCost(const std::vector<int> &path) const
  {
    //check that the path is valid
    for (auto i = 0; i < path.size(); ++i)
    {
      if (!(path[i] < imageSimilarityMatrix.rows))
      {
        std::stringstream ss;
        ss << "Path contains invalid node " << path[i];
        DebugMessage(ss.str(), 1);
        throw std::out_of_range(ss.str());
      }
    }
    auto cost = 0.0f;
    for (auto i = 1; i < path.size(); ++i) //get the cost from previous node to this node to the end
      cost += imageSimilarityMatrix.at<float>(path[i - 1], path[i]);
    return cost;
  }

  cv::Mat ImagePixelSimilarityMatrix::clone() const noexcept
  {
    return imageSimilarityMatrix.clone();
  }
}
