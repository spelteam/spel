#include "imagesimilaritymatrix.hpp"

namespace SPEL
{
  ImageSimilarityMatrix::ImageSimilarityMatrix(void) noexcept
  {
    id = 0x00000000;
  }

  ImageSimilarityMatrix::ImageSimilarityMatrix(const ImageSimilarityMatrix & m) noexcept
  {
    //by default use colour
    imageSimilarityMatrix = m.imageSimilarityMatrix;
    imageShiftMatrix = m.imageShiftMatrix;
  }

  ImageSimilarityMatrix::ImageSimilarityMatrix(ImageSimilarityMatrix && m) noexcept
    : imageSimilarityMatrix(std::move(m.imageSimilarityMatrix)),
    imageShiftMatrix(std::move(m.imageShiftMatrix))
  {
  }

  ImageSimilarityMatrix::~ImageSimilarityMatrix(void) noexcept
  {
    imageSimilarityMatrix.release();
    imageShiftMatrix.release();
  }

  bool ImageSimilarityMatrix::operator==(const ImageSimilarityMatrix & s) const noexcept
  {
    if (imageSimilarityMatrix.rows != s.imageSimilarityMatrix.rows || imageSimilarityMatrix.cols != s.imageSimilarityMatrix.cols)
      return false;

    //if every element is 1
    for (auto i = 0; i < imageSimilarityMatrix.rows; ++i)
    {
      for (auto j = 0; j < imageSimilarityMatrix.cols; ++j)
      {
        if (imageSimilarityMatrix.at<float>(i, j) != s.imageSimilarityMatrix.at<float>(i, j))
          return false;
      }
    }
    return true;
  }

  bool ImageSimilarityMatrix::operator!=(const ImageSimilarityMatrix & s) const noexcept
  {
    return !(*this == s);
  }

  ImageSimilarityMatrix & ImageSimilarityMatrix::operator=(const ImageSimilarityMatrix & s) noexcept
  {
    if (&s == this)
      return *this;

    imageSimilarityMatrix = s.imageSimilarityMatrix;
    imageShiftMatrix = s.imageShiftMatrix;

    return *this;
  }

  ImageSimilarityMatrix & ImageSimilarityMatrix::operator=(ImageSimilarityMatrix && s) noexcept
  {
    imageSimilarityMatrix = std::move(s.imageSimilarityMatrix);
    imageShiftMatrix = std::move(s.imageShiftMatrix);
    return *this;
  }

  uint32_t ImageSimilarityMatrix::getID(void)
  {
    return id;
  }

  void ImageSimilarityMatrix::setID(uint32_t _id)
  {
    id = _id;
  }

  //return the size of the ISM
  uint32_t ImageSimilarityMatrix::size() const noexcept
  {
    return imageSimilarityMatrix.rows;
  }

  void ImageSimilarityMatrix::buildImageSimilarityMatrix(const std::vector<Frame*>& frames, const int maxFrameHeight)
  {
    DebugMessage("building ISM matrix", 5);
    //create matrices and fill with zeros
    imageSimilarityMatrix.release();
    imageShiftMatrix.release();

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
        futures.push_back(std::async(&ImageSimilarityMatrix::computeISMcell, this, left, right, maxFrameHeight));
      }
    }
    //get all futures

    for (auto &e : futures)
      e.get();

    return;
  }

  bool ImageSimilarityMatrix::read(const std::string & filename) noexcept
  {
    std::ifstream in(filename.c_str());
    if (in.is_open())
    {
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

  bool ImageSimilarityMatrix::write(const std::string & filename) const noexcept
  {
    std::ofstream out(filename.c_str());
    if (out.is_open())
    {
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
  float ImageSimilarityMatrix::min() const noexcept
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
  float ImageSimilarityMatrix::mean() const
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

  //find the non-zero minimum in the image similarity matrix
  float ImageSimilarityMatrix::max() const noexcept
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

  float ImageSimilarityMatrix::stddev() const
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
  float ImageSimilarityMatrix::at(const int row, const int col) const
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
  cv::Point2f ImageSimilarityMatrix::getShift(const int row, const int col) const
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
  float ImageSimilarityMatrix::getPathCost(const std::vector<int>& path) const
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

  cv::Point2f ImageSimilarityMatrix::calculateDistance(const cv::Mat & imgMatOne, const cv::Mat & imgMatTwo) const noexcept
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

  cv::Mat ImageSimilarityMatrix::clone() const noexcept
  {
    return imageSimilarityMatrix.clone();
  }
 
}
