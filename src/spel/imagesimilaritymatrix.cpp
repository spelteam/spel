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

    for (auto i = 0; i < imageSimilarityMatrix.rows; ++i)
      for (auto j = 0; j < imageSimilarityMatrix.cols; ++j)
        if (imageSimilarityMatrix.at<float>(i, j) != s.imageSimilarityMatrix.at<float>(i, j))
          return false;

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
    //std::vector<std::future<void> > futures; // Disabled  30.07.16

    for (auto i = 0; i < frames.size(); ++i)
    {
      for (auto j = 0; j < frames.size(); ++j)
      {
        auto left = frames[i];
        auto right = frames[j];
        //futures.push_back(std::async(&ImageSimilarityMatrix::computeISMcell, this, left, right, maxFrameHeight)); // Disabled 30.07.16
        computeISMcell( left, right, maxFrameHeight); // Added 30.07.16 
      }
    }
    //get all futures

    //for (auto &e : futures) // Disabled 30.07.16
    //  e.get();              

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
    cv::Point2f cOne(0.0f, 0.0f), cTwo(0.0f, 0.0f); // 14.06.16 Added "(0.0f, 0.0f)"
    auto mSizeOne = 0.0f, mSizeTwo = 0.0f;

    for (auto y = 0; y < imgMatOne.rows; ++y) // 09.04.16 - replaced x and y
    {
      for (auto x = 0; x < imgMatOne.cols; ++x) // 09.04.16 - replaced x and y
      {
        auto intensity = static_cast<cv::Scalar>(imgMatOne.at<uchar>(y, x)); // 09.04.16 - replaced x and y
        auto mintensity = static_cast<cv::Scalar>(imgMatTwo.at<uchar>(y, x)); // 09.04.16 - replaced x and y

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

  //----------------------------------------------------------
  // Alternative MSM

  uchar C[3][3] = { { 7, 0, 1 },{ 6, 0, 2 },{ 5, 4, 3 } }; // Matrix for search contour operation
  cv::Point2i P[8] = { cv::Point2i(0, -1), cv::Point2i(1, -1), cv::Point2i(1, 0), cv::Point2i(1, 1), cv::Point2i(0, 1), cv::Point2i(-1, 1), cv::Point2i(-1, 0), cv::Point2i(-1, -1) }; // Matrix for search contour operation

  // Search contour iteration
  bool f(cv::Mat mask, cv::Point2i &p0, cv::Point2i &p1, cv::Point2i p00)
  {
    int Q = 9;
    cv::Size size = mask.size();
    bool color = 0;
    cv::Point2i d = p0 - p1, p = p0;
    int n = C[d.y + 1][d.x + 1];

    int  k = 0;
    while (!color)
    { 
      p0 = p;
      p = p1 + P[n];
      if (k > 7) p = p00;
      if((p.x >= 0) && (p.x < size.width) && (p.y >= 0) && (p.y < size.height))
      { color = (mask.at<uchar>(p.y, p.x) > Q); }
      else { color = 0; }
      n++;
      if (n > 7) n = 0;
      k++;
    }
    p1 = p;
    return (p1 == p00);
  }

  // Search all ROI on the mask and return coordinates of max ROI: {topLeft, bottomRight}
  std::vector<cv::Point2i> SearchROI(cv::Mat mask)
  {
    cv::Size size = mask.size();
    int cols = size.width;
    int rows = size.height;

    int Q = 10 - 1;
    int maxArea = 0;
    int N = -1;

    std::vector<std::pair<cv::Point2i, cv::Point2i>> ROI;

    for (int y = 0; y < rows; y++)
      for (int x = 0; x < cols; x++)
      {
        cv::Point2i p(x, y);
        bool b = false;
        for (int i = 0; i < ROI.size(); i++)
        {
          if((p.x >= ROI[i].first.x) && (p.x <= ROI[i].second.x) && (p.y >= ROI[i].first.y) && (p.y <= ROI[i].second.y))
          {
            x = ROI[i].second.x;
            b = true;
          }
        }
        if ((!b) && (mask.at<uchar>(y, x) > Q))
        {
          cv::Point2i  p1 = p, p0 = p + cv::Point2i(-1, 0), p00 = p;

          bool FindedStartPoint = false;
          cv::Point2i A(cols, rows), B(0, 0);

          while (!FindedStartPoint)
          {
            FindedStartPoint = f(mask, p0, p1, p00);

            if (p1.x < A.x) A.x = p1.x;
            if (p1.x > B.x) B.x = p1.x;
            if (p1.y < A.y) A.y = p1.y;
            if (p1.y > B.y) B.y = p1.y;
          }
          ROI.push_back(std::pair<cv::Point2f, cv::Point2f>(A, B));
          cv::Point2f P = B - A;
          float S = P.x*P.y;
          if ( S > maxArea)
          {
            maxArea = S;
            N = ROI.size() - 1;
          }
        }		
      }
    std::vector<cv::Point2i> temp;
    if (N > -1)
    {
      temp.push_back(ROI[N].first);
      temp.push_back(ROI[N].second);
    }

    return temp;
    }

  // Search center of mask in the ROI
  // "ROI" - end points of mask ROI: {topLeft, bottomRight}
  cv::Point2i MaskCenter(cv::Mat mask, std::vector<cv::Point2i> ROI)
  {
    uchar Q = 10 - 1;
    double N = 0;
    cv::Point2d center(0.0, 0.0);
    for (int y = ROI[0].y; y <= ROI[1].y; y++ )
      for (int x = ROI[0].x; x <= ROI[1].x; x++)
      {
        if(mask.at<uchar>(y,x) > Q)
        { 
          N++;
          center = center + cv::Point2d(x, y);
        }         
      }

    return cv::Point2i(static_cast<int>(round(center.x/ N)), static_cast<int>(round(center.y/N)));
  }
 
  // Build MaskSimilarityMatrix  
  // Test version
  // "UseRGBScore" = false  - for build MSM
  // "UseRGBScore" = true - for build ISM
  // If "inverseScore" = "false" then value "Score = 1"  correspond to 100% masks overlap, else inversed
  void ImageSimilarityMatrix::buildMSM_OnMaskArea(const std::vector<Frame*>& frames, bool UseRGBScore, bool inverseScore)
  {
    std::vector<std::vector<cv::Point2i>> ROI;
    std::vector<cv::Point2i> Center;
    int n = frames.size();

    imageSimilarityMatrix.release();
    imageShiftMatrix.release();
    imageSimilarityMatrix = cv::Mat::zeros(cv::Size(n, n), cv::DataType<float>::type);
    imageShiftMatrix = cv::Mat::zeros(cv::Size(n, n), cv::DataType<cv::Point2f>::type);

    for (unsigned int i = 0; i < frames.size(); i++)
    {
      ROI.push_back(SearchROI(frames[i]->getMask())); // Search ROI for all frames
      Center.push_back(MaskCenter(frames[i]->getMask(), ROI[i])); // Search mask center for all frames
    }

    for (unsigned int i = 0; i < frames.size(); i++)
      for (unsigned int k = 0; k < i; k++)
      {
        uchar Q = 9;
        cv::Mat mask1 = frames[i]->getMask();
        cv::Mat mask2 = frames[k]->getMask();
        cv::Point2i d = Center[k] - Center[i]; // masks centers distance 
        cv::Point2i A(std::min(ROI[i][0].x, ROI[k][0].x - d.x), std::min(ROI[i][0].y, ROI[k][0].y - d.y)); // extended ROI top left point
        cv::Point2i B(std::max(ROI[i][1].x, ROI[k][1].x - d.x), std::max(ROI[i][1].y, ROI[k][1].y - d.y)); // extended ROI bottom right point
        int /*S1 = 0, S2 = 0,*/ intersection = 0, difference = 0;
        float Score = 0;

        // Calculate Mask Similarity Matrix
        if (!UseRGBScore)
        {
          for (int y = A.y; y <= B.y; y++)
            for (int x = A.x; x <= B.x; x++)
            {
              cv::Point2i p = cv::Point2i(x, y) + d;
              if (p.x >= 0 && p.x < mask2.cols && p.y >= 0 && p.y < mask2.rows)
              {
                bool color1 = (mask1.at<uchar>(y, x) > Q);
                bool color2 = (mask2.at<uchar>(p.y, p.x) > Q);
                //if (color1) S1++; // mask1 area - don't used
                //if (color2) S2++; // mask2 area - don't used
                if (color1 && color2) intersection++;
                if (color1 != color2) difference++;
              }
            }
          Score = static_cast<float>(intersection) / static_cast<float>(difference + intersection);
        }

        // Calculate normalized  Image Similarity Matrix
        if(UseRGBScore)
        {
          double S = 0;
          cv::Mat image1 = frames[i]->getImage();
          cv::Mat image2 = frames[k]->getImage();
          for (int y = A.y; y <= B.y; y++)
            for (int x = A.x; x <= B.x; x++)
            {
              cv::Point2i p = cv::Point2i(x, y) + d;
              if (p.x >= 0 && p.x < mask2.cols && p.y >= 0 && p.y < mask2.rows)
              {
                bool color1 = (mask1.at<uchar>(y, x) > Q);
                bool color2 = (mask2.at<uchar>(p.y, p.x) > Q);
                if (color1 && color2)
                {
                  intersection++;
                  cv::Vec3b colorDistance = image1.at<cv::Vec3b>(y, x) - image2.at<cv::Vec3b>(p.y, p.x);
                  S = S + (1.0 - static_cast<double>(colorDistance[0] * colorDistance[0] + colorDistance[1] * colorDistance[1] + colorDistance[2] * colorDistance[2]) / (255.0*255.0*3.0));
                }
                if (color1 != color2)
                  difference++;
              }
            }
          Score = static_cast<float>((S)/static_cast<double>(difference + intersection));///static_cast<float>(intersection) / static_cast<float>(difference + intersection);
        }

        if (inverseScore) Score = 1.0f - Score;

        imageSimilarityMatrix.at<float>(i, k) = Score;
        imageSimilarityMatrix.at<float>(k, i) = Score;
        imageShiftMatrix.at<cv::Point2f>(i, k) = d;
        imageShiftMatrix.at<cv::Point2f>(k, i) = -d;
      }

    for (unsigned int i = 0; i < frames.size(); i++)
    {
      imageSimilarityMatrix.at<float>(i, i) = static_cast<float>(inverseScore);
      ROI[i].clear();
    }

    ROI.clear();
    Center.clear();

    for (unsigned int i = 0; i < frames.size(); i++)
      frames[i]->UnloadAll();
  }

  void ImageSimilarityMatrix::buildImageSimilarityMatrix(const std::vector<Frame*>& frames, int Erode, int Dilate, bool UseRGBScore, bool inverseScore)
  {
    if (Erode == 0 && Dilate == 0)
      buildMSM_OnMaskArea(frames, UseRGBScore, inverseScore);

    if (Erode != 0 || Dilate != 0)
    {
      std::vector<Frame*> modified_frames(frames.size());

      cv::Mat element = cv::Mat(3, 3, CV_8U, 1);
      element.at<uchar>(0, 0) = 0;
      element.at<uchar>(0, 2) = 0;
      element.at<uchar>(2, 0) = 0;
      element.at<uchar>(2, 2) = 0;

      // Copy and modify frames
      for (unsigned int i = 0; i < frames.size(); i++)
      {
        modified_frames[i] = new Frame(LOCKFRAME);
        if(UseRGBScore) modified_frames[i]->setImage(frames[i]->getImage());
        cv::Mat mask = frames[i]->getMask().clone();

        if (Erode > 0) erode(mask, mask, element, cv::Point2i(0, 0), Erode);
        if (Dilate > 0) dilate(mask, mask, element, cv::Point2i(0, 0), Dilate);
        
        modified_frames[i]->setMask(mask);
        mask.release();
        frames[i]->UnloadAll();
      }

      // Build ISM on modified frames
      buildMSM_OnMaskArea(modified_frames, UseRGBScore, inverseScore);

      // Delete modified frames
      for (unsigned int i = 0; i < modified_frames.size(); i++)
        delete modified_frames[i];
      modified_frames.clear();
    }

  }
  //----------------------------------------------------------  

}
