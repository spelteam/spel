#include "colorHistDetector.hpp"
#include "keyframe.hpp"
#include "lockframe.hpp"
#include "interpolation.hpp"
#include "spelParameters.hpp"

namespace SPEL
{
  ColorHistDetector::PartModel::PartModel(uint8_t _nBins) : nBins(_nBins)
  {
    if (_nBins == 0)
    {
      const std::string str = "nBins can't be zero";
      DebugMessage(str, 1);
      throw std::invalid_argument(str);
    }

    partHistogram.resize(nBins, std::vector<std::vector<float>>(nBins, 
      std::vector<float>(nBins, 0.0)));
    bgHistogram.resize(nBins, std::vector<std::vector<float>>(nBins, 
      std::vector<float>(nBins, 0.0)));

    sizeFG = 0;
    sizeBG = 0;
    fgNumSamples = 0;
    bgNumSamples = 0;
  }

  ColorHistDetector::PartModel::~PartModel(void)
  {
  }

  ColorHistDetector::PartModel &ColorHistDetector::PartModel::operator=(
    const PartModel &model) noexcept
  {
    this->nBins = model.nBins;
    this->partHistogram = model.partHistogram;
    this->bgHistogram = model.bgHistogram;
    this->sizeFG = model.sizeFG;
    this->sizeBG = model.sizeBG;
    this->fgNumSamples = model.fgNumSamples;
    this->bgNumSamples = model.bgNumSamples;
    this->fgSampleSizes = model.fgSampleSizes;
    this->bgSampleSizes = model.bgSampleSizes;
    this->fgBlankSizes = model.fgBlankSizes;
    return *this;
  }

  uint8_t ColorHistDetector::PartModel::calculateFactor(void) const
  {
    if (nBins == 0)
    {
      const std::string str = "nBins can't be zero";
      DebugMessage(str, 1);
      throw std::logic_error(str);
    }

    return static_cast<uint8_t> (ceil(pow(2, 8) / nBins));
  }

  float ColorHistDetector::PartModel::computePixelBelongingLikelihood(
    const uint8_t r, const uint8_t g, const uint8_t b) const
  {
    if (nBins == 0)
    {
      const std::string str = "nBins can't be zero";
      DebugMessage(str, 1);
      throw std::logic_error(str);
    }

    // Scaling of colorspace, finding the colors interval, 
    // which now gets this color
    auto factor = calculateFactor();
    try
    {
      // relative frequency of current color reiteration 
      return partHistogram.at(r / factor).at(g / factor).at(b / factor);
    }
    catch (...)
    {
      std::stringstream ss;
      ss << "Couldn't find partHistogram " << "[" << r / factor << "][" << 
        g / factor << "][" << b / factor << "]";
      DebugMessage(ss.str(), 1);
      throw std::out_of_range(ss.str());
    }
  }

  void ColorHistDetector::PartModel::setPartHistogram(
    const std::vector <cv::Point3i> &partColors)
  {
    if (nBins == 0)
    {
      const std::string str = "nBins can't be zero";
      DebugMessage(str, 1);
      throw std::logic_error(str);
    }

    // do not add sample if the number of pixels is zero
    if (partColors.size() == 0)
      return;
    // colorspace scaling coefficient
    auto factor = calculateFactor();
    sizeFG = static_cast <uint32_t> (partColors.size());
    fgNumSamples = 1;
    fgSampleSizes.clear();
    fgSampleSizes.push_back(static_cast <uint32_t> (partColors.size()));

    // clear histogram first
    if (partHistogram.size() != nBins)
      partHistogram.resize(nBins);
    for (auto r = 0; r < nBins; r++)
    {
      if (partHistogram.at(r).size() != nBins)
        partHistogram.at(r).resize(nBins);
      for (auto g = 0; g < nBins; g++)
      {
        if (partHistogram.at(r).at(g).size() != nBins)
          partHistogram.at(r).at(g).resize(nBins);
        for (auto b = 0; b < nBins; b++)
          partHistogram.at(r).at(g).at(b) = 0.0;
      }
    }

    // Scaling of colorspace, reducing the capacity and number of colour 
    // intervals that are used to construct the histogram
    for (const auto &i : partColors)
    {
      auto r = static_cast<uint8_t> (i.x / factor);
      auto g = static_cast<uint8_t> (i.y / factor);
      auto b = static_cast<uint8_t> (i.z / factor);

      if (r >= nBins || g >= nBins || b >= nBins)
      {
        std::stringstream ss;
        ss << "RGB value can't be greater " << nBins - 1 << ": r = " << r 
          << " g = " << g << " b = " << b << std::endl;
        DebugMessage(ss.str(), 1);
        throw std::out_of_range(ss.str());
      }
      // increment the frequency of interval, that this color have hit
      partHistogram.at(r).at(g).at(b)++;
    }

    // normalise the histograms
    for (auto r = 0; r < nBins; r++)
    {
      for (auto g = 0; g < nBins; g++)
      {
        for (auto b = 0; b < nBins; b++)
        {
          partHistogram.at(r).at(g).at(b) /= static_cast<float>(sizeFG);
        }
      }
    }
  }

  void ColorHistDetector::PartModel::addPartHistogram(
    const std::vector <cv::Point3i> &partColors, const uint32_t nBlankPixels)
  {
    //do not add sample if the number of pixels is zero
    if (partColors.size() == 0)
      return;
    //un-normalise
    if (partHistogram.size() != nBins)
    {
      std::stringstream ss;
      ss << "Wrond size of partHistogram. Expected: " << nBins << 
        ". Actual: " << partHistogram.size() << std::endl;
      DebugMessage(ss.str(), 1);
      throw std::logic_error(ss.str());
    }
    for (auto r = 0; r < nBins; r++)
    {
      if (partHistogram.at(r).size() != nBins)
      {
        std::stringstream ss;
        ss << "Wrond size of partHistogram[" << r << "]. Expected: " << 
          nBins << ". Actual: " << partHistogram.at(r).size() << std::endl;
        DebugMessage(ss.str(), 1);
        throw std::logic_error(ss.str());
      }
      for (auto g = 0; g < nBins; g++)
      {
        if (partHistogram.at(r).at(g).size() != nBins)
        {
          std::stringstream ss;
          ss << "Wrond size of partHistogram[" << r << "][" << g << 
            "]. Expected: " << nBins << ". Actual: " << 
            partHistogram.at(r).at(g).size() << std::endl;
          DebugMessage(ss.str(), 1);
          throw std::logic_error(ss.str());
        }
        // converting the colors relative frequency into the pixels number
        for (auto b = 0; b < nBins; b++)
          partHistogram.at(r).at(g).at(b) *= static_cast<float>(sizeFG);
      }
    }

    auto factor = calculateFactor(); // colorspace scaling coefficient
    sizeFG += static_cast <uint32_t> (partColors.size());

    if (sizeFG == 0)
    {
      const std::string str = "sizeFG can't be zero";
      DebugMessage(str, 1);
      throw std::logic_error(str);
    }

    fgNumSamples++;
    fgSampleSizes.push_back(static_cast <uint32_t> (partColors.size()));

    // Scaling of colorspace, reducing the capacity and number of 
    // colour intervals
    // Adjustment of the histogram
    for (const auto &color : partColors)
    {
      auto r = static_cast<uint8_t> (color.x / factor);
      auto g = static_cast<uint8_t> (color.y / factor);
      auto b = static_cast<uint8_t> (color.z / factor);

      if (r >= nBins || g >= nBins || b >= nBins)
      {
        std::stringstream ss;
        ss << "RGB value can't be greater " << nBins - 1 << ": r = " << r << 
          " g = " << g << " b = " << b << std::endl;
        DebugMessage(ss.str(), 1);
        throw std::out_of_range(ss.str());
      }
      // increment the frequency of interval, that this color have hit
      partHistogram.at(r).at(g).at(b)++; 
    }

    //renormalise
    for (auto r = 0; r < nBins; r++)
    {
      for (auto g = 0; g < nBins; g++)
      {
        for (auto b = 0; b < nBins; b++)
        {
          //normalise the histograms
          partHistogram.at(r).at(g).at(b) /= static_cast<float>(sizeFG);
        }
      }
    }
    // add the number of blank pixels for this model
    fgBlankSizes.push_back(nBlankPixels);
  }

  float ColorHistDetector::PartModel::getAvgSampleSizeFg(void) const
  {
    if (fgNumSamples == 0 && fgSampleSizes.size() > 0)
    {
      const std::string str = "fgNumSamples can't be zero";
      DebugMessage(str, 1);
      throw std::logic_error(str);
    }

    auto sum = 0.0f;
    for (const auto &i : fgSampleSizes)
      sum += i;
    sum /= static_cast<float>(fgNumSamples);
    return sum;
  }

  float ColorHistDetector::PartModel::getAvgSampleSizeFgBetween(
    const uint32_t s1, const uint32_t s2) const
  {
    if (s1 >= fgSampleSizes.size() || s2 >= fgSampleSizes.size())
    {
      std::stringstream ss;
      ss << "Incorrect parameter. s1: " << s1 << " s2: " << s2 << 
        " Actual size: " << fgSampleSizes.size() << std::endl;
      DebugMessage(ss.str(), 1);
      throw std::invalid_argument(ss.str());
    }
    return (fgSampleSizes.at(s1) + fgSampleSizes.at(s2)) / 2.0f;
  }

  float ColorHistDetector::PartModel::matchPartHistogramsED(
    const PartModel &partModelPrev) const
  {
    if (nBins == 0)
    {
      const std::string str = "nBins can't be zero";
      DebugMessage(str, 1);
      throw std::logic_error(str);
    }

    if (nBins != partModelPrev.nBins)
    {
      std::stringstream ss;
      ss << "Different nBins value. Expected: " << nBins << " Actual: " << 
        partModelPrev.nBins << std::endl;
      DebugMessage(ss.str(), 1);
      std::logic_error(ss.str());
    }

    auto distance = 0.0f;

    if (partHistogram.size() != nBins)
    {
      std::stringstream ss;
      ss << "Wrond size of partHistogram. Expected: " << nBins << 
        ". Actual: " << partHistogram.size() << std::endl;
      DebugMessage(ss.str(), 1);
      throw std::logic_error(ss.str());
    }
    if (partModelPrev.partHistogram.size() != nBins)
    {
      std::stringstream ss;
      ss << "Wrond size of partHistogram. Expected: " << nBins << 
        ". Actual: " << partModelPrev.partHistogram.size() << std::endl;
      DebugMessage(ss.str(), 1);
      throw std::logic_error(ss.str());
    }
    for (auto r = 0; r < nBins; r++)
    {
      if (partHistogram.at(r).size() != nBins)
      {
        std::stringstream ss;
        ss << "Wrond size of partHistogram [" << r << "]. Expected: " << 
          nBins << ". Actual: " << partHistogram.at(r).size() << std::endl;
        DebugMessage(ss.str(), 1);
        throw std::logic_error(ss.str());
      }
      if (partModelPrev.partHistogram.at(r).size() != nBins)
      {
        std::stringstream ss;
        ss << "Wrond size of partHistogram [" << r << "]. Expected: " << 
          nBins << ". Actual: " << partModelPrev.partHistogram.at(r).size() << 
          std::endl;
        DebugMessage(ss.str(), 1);
        throw std::logic_error(ss.str());
      }
      for (auto g = 0; g < nBins; g++)
      {
        if (partHistogram.at(r).at(g).size() != nBins)
        {
          std::stringstream ss;
          ss << "Wrond size of partHistogram [" << r << "][" << g << 
            "]. Expected: " << nBins << ". Actual: " << 
            partHistogram.at(r).at(g).size() << std::endl;
          DebugMessage(ss.str(), 1);
          throw std::logic_error(ss.str());
        }
        if (partModelPrev.partHistogram.at(r).at(g).size() != nBins)
        {
          std::stringstream ss;
          ss << "Wrond size of partHistogram [" << r << "][" << g << 
            "]. Expected: " << nBins << ". Actual: " << 
            partModelPrev.partHistogram.at(r).at(g).size() << std::endl;
          DebugMessage(ss.str(), 1);
          throw std::logic_error(ss.str());
        }
        for (auto b = 0; b < nBins; b++)
          // accumulation of the Euclidean distances between the points
          distance += pow(partHistogram.at(r).at(g).at(b) - 
            partModelPrev.partHistogram.at(r).at(g).at(b), 2);
      }
    }
    return sqrt(distance);
  }

  void ColorHistDetector::PartModel::addBackgroundHistogram(
    const std::vector <cv::Point3i> &bgColors)
  {
    if (bgColors.size() == 0)
      return;

    if (nBins == 0)
    {
      const std::string str = "nBins can't be zero";
      DebugMessage(str, 1);
      throw std::logic_error(str);
    }

    // unnormalise
    if (bgHistogram.size() != nBins)
    {
      std::stringstream ss;
      ss << "Wrond size of bgHistogram. Expected: " << nBins << 
        ". Actual: " << bgHistogram.size() << std::endl;
      DebugMessage(ss.str(), 1);
      throw std::logic_error(ss.str());
    }
    for (auto r = 0; r < nBins; r++)
    {
      if (bgHistogram.at(r).size() != nBins)
      {
        std::stringstream ss;
        ss << "Wrond size of bgHistogram [" << r << "]. Expected: " << 
          nBins << ". Actual: " << bgHistogram.at(r).size() << std::endl;
        DebugMessage(ss.str(), 1);
        throw std::logic_error(ss.str());
      }
      for (auto g = 0; g < nBins; g++)
      {
        if (bgHistogram.at(r).at(g).size() != nBins)
        {
          std::stringstream ss;
          ss << "Wrond size of bgHistogram[" << r << "][" << g << 
            "]. Expected: " << nBins << ". Actual: " << 
            bgHistogram.at(r).at(g).size() << std::endl;
          DebugMessage(ss.str(), 1);
          throw std::logic_error(ss.str());
        }
        for (auto b = 0; b < nBins; b++)
          bgHistogram.at(r).at(g).at(b) *= static_cast<float>(sizeBG);
      }
    }

    auto factor = calculateFactor(); // colorspace scaling coefficient
    sizeBG += static_cast <uint32_t> (bgColors.size());
    bgNumSamples++;
    bgSampleSizes.push_back(static_cast <uint32_t> (bgColors.size()));

    if (sizeBG == 0)
    {
      const std::string str = "sizeBG can't be zero";
      DebugMessage(str, 1);
      throw std::logic_error(str);
    }

    for (const auto &color : bgColors)
    {
      auto r = static_cast<uint8_t> (color.x / factor);
      auto g = static_cast<uint8_t> (color.y / factor);
      auto b = static_cast<uint8_t> (color.z / factor);

      if (r >= nBins || g >= nBins || b >= nBins)
      {
        std::stringstream ss;
        ss << "RGB value can't be greater " << nBins - 1 << ": r = " << r << 
          " g = " << g << " b = " << b << std::endl;
        DebugMessage(ss.str(), 1);
        throw std::out_of_range(ss.str());
      }

      // increment the frequency of interval, that this color have hit
      bgHistogram.at(r).at(g).at(b)++; 
    }
    // renormalise
    for (auto r = 0; r < nBins; r++)
      for (auto g = 0; g < nBins; g++)
        for (auto b = 0; b < nBins; b++)
          bgHistogram.at(r).at(g).at(b) /= static_cast<float>(sizeBG);
  }

  ColorHistDetector::ColorHistDetector(uint8_t _nBins) : nBins(_nBins)
  {
    if (_nBins == 0)
    {
      const std::string str = "nBins can't be zero";
      DebugMessage(str, 1);
      throw std::invalid_argument(str);
    }
    m_id = 0x43484400;
  }

  ColorHistDetector::~ColorHistDetector(void) noexcept
  {
  }

  void ColorHistDetector::train(const std::vector <Frame*> &_frames, 
    std::map <std::string, float> params)
  {
    if (_frames.size() == 0)
    {
      const std::string str = "No input frames";
      DebugMessage(str, 1);
      throw std::logic_error(str); // the sequence of frames is empty
    }

    // vector of pointers - presents a sequence of frames
    frames = _frames;
    // sorting frames by id
    sort(frames.begin(), frames.end(), Frame::FramePointerComparer);
    
    partModels.clear();

    params.emplace(COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().first, 
      frames.at(0)->getFrameSize().height);

    maxFrameHeight = static_cast<uint32_t>(params.at(
      COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().first));

    // Handling all frames
    for (auto &workFrame : frames)
    {
      if (workFrame->getFrametype() != KEYFRAME && 
        workFrame->getFrametype() != LOCKFRAME)
        continue;

      workFrame->Resize(maxFrameHeight);

      if (SpelObject::getDebugLevel() >= 2)
        std::cout << "Training on frame " << workFrame->getID() << std::endl;
      // Create local variables
      // the set of RGB-colours of pixel's for current body part
      std::map <int32_t, std::vector <cv::Point3i>> partPixelColours;
      // the set of RGB-colours for a pixels of background
      std::map <int32_t, std::vector <cv::Point3i>> bgPixelColours;
      // pixels outside the mask
      std::map <int32_t, int> blankPixels;
      // copy marking from current frame
      auto skeleton = workFrame->getSkeleton();
      // polygons for this frame
      std::multimap <int32_t, POSERECT <cv::Point2f>> polygons;
      // used for evaluation of overlapped polygons
      std::multimap <int32_t, float> polyDepth;
      // the skeleton body parts
      auto partTree = skeleton.getPartTree();
      // Handling all bodyparts on the frames
      for (auto &bodyPart : partTree)
      {
        // container initialization for conserve colours set 
        // for each of body parts
        partPixelColours.insert(std::pair <int32_t, 
          std::vector <cv::Point3i>>(bodyPart.getPartID(), 
            std::vector <cv::Point3i>()));
        // container initialization for conserve background colours set 
        // for each of body parts
        bgPixelColours.insert(std::pair <int32_t, 
          std::vector <cv::Point3i>>(bodyPart.getPartID(), 
            std::vector <cv::Point3i>()));
        // container initialization for counting blank pixels 
        // for each of body parts
        blankPixels.insert(std::pair <int32_t, int>(bodyPart.getPartID(), 0)); 

        // the parent node of current body part pointer 
        auto joint = skeleton.getBodyJoint(bodyPart.getParentJoint());
        if (joint == 0)
        {
          const std::string str = "Invalid parent joint";
          DebugMessage(str, 1);        
          // a joint has no marking on the frame
          throw std::logic_error(str); 
        }
        // coordinates of current joint
        auto j0 = joint->getImageLocation(); 
        // the child node of current body part pointer
        joint = skeleton.getBodyJoint(bodyPart.getChildJoint()); 
        if (joint == 0)
        {
          const std::string str = "Invalid child joint";
          DebugMessage(str, 1);        
          // a joint has no marking on the frame
          throw std::logic_error(str); 
        }
        // coordinates of current joint
        auto j1 = joint->getImageLocation(); 
        // used as estimation of the vector's direction
        auto direction = j1 - j0; 
        //bodypart tilt angle 
        auto rotationAngle = static_cast<float>(spelHelper::angle2D(1.0f, 
          0.0f, direction.x, direction.y) * (180.0f / M_PI));
        bodyPart.setRotationSearchRange(rotationAngle);
        auto poserect = bodyPart.getBodyPartRect(j0, j1);
        polygons.insert(std::pair <int32_t, POSERECT <cv::Point2f>>(
          bodyPart.getPartID(), poserect));
        polyDepth.insert(std::pair <int32_t, float>(bodyPart.getPartID(), 
          skeleton.getBodyJoint(
            bodyPart.getParentJoint())->getSpaceLocation().z));
      }
      skeleton.setPartTree(partTree);
      workFrame->setSkeleton(skeleton);
      // copy mask from the current frame
      const auto &maskMat = workFrame->getMask();
      // copy image from the current frame
      const auto &imgMat = workFrame->getImage();
      // Range over all pixels of the frame
      for (auto i = 0; i < imgMat.cols; ++i)
      {
        for (auto j = 0; j < imgMat.rows; ++j)
        {
          cv::Vec3b intensity;
          try
          {
            // copy RGB color of current pixel
            intensity = imgMat.at<cv::Vec3b>(j, i);
          }
          catch (...)
          {
            workFrame->UnloadAll();
            std::stringstream ss;
            ss << "Couldn't get imgMat value of indeces " << "[" << j << 
              "][" << i << "]";
            DebugMessage(ss.str(), 1);
            throw std::out_of_range(ss.str());
          }
          // Copy the current pixel colour components
          auto blue = intensity.val[0];
          auto green = intensity.val[1];
          auto red = intensity.val[2];
          auto mintensity = 0;
          try
          {
            // copy current pixel mask value 
            mintensity = maskMat.at<uint8_t>(j, i);
          }
          catch (...)
          {
            workFrame->UnloadAll();
            std::stringstream ss;
            ss << "Couldn't get maskMat value of indeces " << "[" << j << 
              "][" << i << "]";
            DebugMessage(ss.str(), 1);
            throw std::out_of_range(ss.str());
          }
          auto blackPixel = mintensity < 10;
          // will be equal to -1 until is not found polygon, 
          // which contains the point
          auto partHit = -1;
          auto depth = 0.0f;
          // Handling all polygons
          for (const auto &bodyPart : partTree)
          {
            auto partNumber = bodyPart.getPartID();
            auto bContainsPoint = false;
            std::vector <POSERECT <cv::Point2f>> partPolygons;
            // Copy polygons to "PartPolygons"
            transform(polygons.lower_bound(partNumber), 
              polygons.upper_bound(partNumber), back_inserter(partPolygons),
              [](auto const &pair) { return pair.second; });
            // Checking whether a pixel belongs to the current and 
            // to another polygons            
            for (const auto &partPolygon : partPolygons)
              if ((bContainsPoint = partPolygon.containsPoint(
                cv::Point2f(static_cast<float>(i), 
                  static_cast<float>(j))) > 0) == true)
                break; // was found polygon, which contain current pixel

            std::vector <float> partDepths;
            // copy "polyDepth" to "PartDepth"
            transform(polyDepth.lower_bound(partNumber), 
              polyDepth.upper_bound(partNumber), back_inserter(partDepths),
              [](auto const &pair) { return pair.second; }); 
            // Checking polygons overlapping
            for (const auto &partDepth : partDepths)
            {
              if (bContainsPoint && partHit == -1)
              {
                // store the number of the first found polygon
                partHit = partNumber; 
                depth = partDepth;
              }
              else if (bContainsPoint && partDepth < depth) 
              {
                partHit = partNumber;
                depth = partDepth;
              }
            }
          }
          // if was found polygon, that contains this pixel
          if (partHit != -1) 
          {
            // if pixel color isn't black
            if (!blackPixel) 
            {
              // add colour of this pixel to part[partHit] colours
              partPixelColours.at(partHit).push_back(
                cv::Point3i(red, green, blue));

              // For all bodyparts
              for (const auto &p : partTree)
                // if current poligon wasn't found first 
                // in the previous enumeration???
                if (p.getPartID() != partHit) 
                  // add colour of this pixel to part[partHit] 
                  // background colours
                  bgPixelColours.at(p.getPartID()).push_back(cv::Point3i(red, 
                    green, blue)); 
            }
            else
              // otherwise take stock this pixel to blank pixel counter
              ++blankPixels.at(partHit);
          }
          // if not found polygon, that contains this pixel 
          else
            for (const auto &p : partTree)
              bgPixelColours.at(p.getPartID()).push_back(cv::Point3i(red, 
                green, blue));
        }
      }

      // Create model for each bodypart
      for (const auto &bodyPart : partTree)
      {
        const auto &partNumber = bodyPart.getPartID();
        if (partModels.find(partNumber) == partModels.end())
          //add a new model to end of models list
          partModels.insert(std::pair <int32_t, PartModel>(partNumber, 
            PartModel(nBins)));

        auto &partModel = partModels.at(partNumber);
        // building histogram for current bodypart colours
        partModel.addPartHistogram(partPixelColours.at(partNumber), 
          blankPixels.at(partNumber));
        // building histograms for current bodypart background colours
        partModel.addBackgroundHistogram(bgPixelColours.at(partNumber));
        if (SpelObject::getDebugLevel() >= 2)
          std::cout << "Found part model: " << partNumber << std::endl;
      }
      workFrame->AdjustScale();
      workFrame->UnloadAll();
    }
  }
  
  // Returns a labels vector of possible body parts position
  std::map <uint32_t, std::vector <LimbLabel> > ColorHistDetector::detect(
    Frame *frame, std::map <std::string, float> params, 
    const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels) const
  {
    params.emplace(COMMON_DETECTOR_PARAMETERS::USE_CH_DETECTOR());

    auto detectorHelper = new ColorHistDetectorHelper();

    // matrix contains the probability that the particular pixel belongs 
    // to current bodypart
    auto pixelDistributions = buildPixelDistributions(frame);
    // matrix contains relative estimations that the particular 
    // pixel belongs to current bodypart
    detectorHelper->pixelLabels = buildPixelLabels(frame, pixelDistributions); 
    
    for (auto &p : pixelDistributions)
      p.second.release();

    auto result = Detector::detect(frame, params, limbLabels, detectorHelper);

    delete detectorHelper;
    frame->UnloadAll();
    return result;
  }

  // Return nBins
  uint8_t ColorHistDetector::getNBins(void) const noexcept
  {
    return nBins;
  }

  // Returns a matrix, that contains relative frequency of the pixels 
  // colors reiteration 
  std::map <int32_t, cv::Mat> ColorHistDetector::buildPixelDistributions(
    Frame *frame) const
  {
    // copy skeleton from the frame
    auto skeleton = frame->getSkeleton();
    // copy part tree from the skeleton
    auto partTree = skeleton.getPartTree();
    // copy image from the frame
    auto imgMat = frame->getImage();
    // copy mask from the frame
    auto maskMat = frame->getMask();
    auto width = imgMat.cols;
    auto height = imgMat.rows;
    auto mwidth = maskMat.cols;
    auto mheight = maskMat.rows;
    std::map <int32_t, cv::Mat> tempPixelDistributions;
    // error if mask and image sizes don't match
    if (width != mwidth || height != mheight)
    {
      const std::string str = "Mask size not equal image size";
      DebugMessage(str, 1);
      throw std::logic_error(str);
    }
    // For all bodyparts
    for (const auto &bodyPart : partTree)
    {
      // create empty matrix
      auto t = cv::Mat(height, width, cv::DataType <float>::type);
      auto partID = bodyPart.getPartID();
      try
      {
        // copy part model of current bodybart
        const auto &partModel = partModels.at(partID);
        // For all pixels
        for (auto x = 0; x < width; x++)
        {
          for (auto y = 0; y < height; y++)
          {
            const auto &intensity = imgMat.at<cv::Vec3b>(y, x);
            // Copy components of the current pixel color
            auto blue = intensity.val[0];
            auto green = intensity.val[1];
            auto red = intensity.val[2];
            // copy mask of the current pixel
            auto mintensity = maskMat.at<uint8_t>(y, x);
            // pixel is not significant if the mask value is less 
            // than this threshold
            auto blackPixel = mintensity < 10;
            // relative frequency of the current pixel color reiteration 
            t.at<float>(y, x) = blackPixel ? 0 : 
              partModel.computePixelBelongingLikelihood(red, green, blue);
          }
        }
      }
      catch (...)
      {
        std::stringstream ss;
        ss << "Maybe couldn't find partModel " << partID;
        DebugMessage(ss.str(), 1);
        throw std::logic_error(ss.str());
      }
      // add the current bodypart matrix to the set 
      tempPixelDistributions.insert(std::pair <int32_t, cv::Mat>(partID, t)); 
    }
    return tempPixelDistributions;
  }

  std::map <int32_t, cv::Mat> ColorHistDetector::buildPixelLabels(Frame *frame, 
    const std::map <int32_t, cv::Mat> &_pixelDistributions) const
  {
    // copy mask from the frame
    auto maskMat = frame->getMask();
    auto width = maskMat.cols;
    auto height = maskMat.rows;
    // copy skeleton from the frame
    auto skeleton = frame->getSkeleton();
    // copy part tree from the skeleton
    auto partTree = skeleton.getPartTree();
    std::map <int32_t, cv::Mat> _pixelLabels;
    // For all body parts
    for (const auto &bodyPart : partTree)
    {
      // create empty matrix
      auto t = cv::Mat(height, width, cv::DataType <float>::type);
      cv::Mat tt;
      try
      { 
        // Matrix, that contains relative frequency of the pixels colors 
        // reiteration for current body part
        tt = _pixelDistributions.at(bodyPart.getPartID());
      }
      catch (...)
      {
        std::stringstream ss;
        ss << "Couldn't find distributions for body part " << 
          bodyPart.getPartID();
        DebugMessage(ss.str(), 1);
        throw std::out_of_range(ss.str());
      }
      // For all pixels
      for (auto x = 0; x < width; x++)
      {
        for (auto y = 0; y < height; y++)
        {
          //copy the current pixel mask value
          auto mintensity = maskMat.at<uint8_t>(y, x);
          // pixel is not significant if the mask value is less 
          // than this threshold
          auto blackPixel = mintensity < 10;
          if (!blackPixel)
          {
            auto top = 0.0f;
            auto sum = 0.0f;
            // For all body parts
            for (const auto &i : partTree)
            {
              cv::Mat temp;
              try
              {
                // matrix of the pixels colors frequency for current body part
                temp = _pixelDistributions.at(i.getPartID());
              }
              catch (...)
              {
                std::stringstream ss;
                ss << "Couldn't find pixel distributions for body part " << 
                  i.getPartID();
                DebugMessage(ss.str(), 1);
                throw std::out_of_range(ss.str());
              }
              try
              {
                // search max value of the current bodypart 
                // pixel color frequency
                if (temp.at<float>(y, x) > top)
                  top = temp.at<float>(y, x);
                sum += temp.at<float>(y, x);
              }
              catch (...)
              {
                std::stringstream ss;
                ss << "Couldn't find value of temp " << "[" << y << "][" << 
                  x << "]";
                DebugMessage(ss.str(), 1);
                throw std::out_of_range(ss.str());
              }
            }
            try
            {
              t.at<float>(y, x) = (top == 0.0f) ? 0.0f : tt.at<float>(y, x) / 
                static_cast<float>(top);
            }
            catch (...)
            {
              std::stringstream ss;
              ss << "Couldn't find t " << "[" << y << "][" << x << 
                "] or tt [" << y << "][" << x << "]";
              DebugMessage(ss.str(), 1);
              throw std::out_of_range(ss.str());
            }
          }
          else
          {
            try
            {
              t.at<float>(y, x) = 0.0f;
            }
            catch (...)
            {
              std::stringstream ss;
              ss << "Couldn't find value of t " << "[" << y << "][" << x 
                << "]";
              DebugMessage(ss.str(), 1);
              throw std::out_of_range(ss.str());
            }
          }
        }
      }
      // insert the resulting matrix into the set "pixelLabels" 
      _pixelLabels.insert(std::pair<int32_t, cv::Mat>(
        bodyPart.getPartID(), t));
    }
    return _pixelLabels;
  }

  float ColorHistDetector::compare(const BodyPart &bodyPart, 
    Frame *frame, const std::map <int32_t, cv::Mat> &_pixelLabels, 
    const cv::Point2f &j0, const cv::Point2f &j1) const
  {
    // copy mask from the frame 
    auto maskMat = frame->getMask();
    // copy image from the frame
    auto imgMat = frame->getImage();
    // segment center
    auto boxCenter = j0 * 0.5f + j1 * 0.5f;
    // distance between joints
    auto boneLength = BodyPart::getBoneLength(j0, j1);
    // expected bodypart location area?
    auto rect = spelHelper::round(bodyPart.getBodyPartRect(j0, j1));
    auto totalPixels = 0;
    auto pixelsInMask = 0;
    auto totalPixelLabelScore = 0.0f;
    PartModel model;
    try
    {
      // copy part model for the "bodyPart"
      model = partModels.at(bodyPart.getPartID());
    }
    catch (...)
    {
      std::stringstream ss;
      ss << "Couldn't get partModel of bodyPart " << bodyPart.getPartID();
      DebugMessage(ss.str(), 1);
      throw std::out_of_range(ss.str());
    }
    // error if samples count is zero
    if (model.getAvgSampleSizeFg() == 0) 
    {
      const std::string str = "Couldn't get avgSampleSizeFg";
      DebugMessage(str, 1);
      throw std::out_of_range(str);
    }
    float xmax, ymax, xmin, ymin;
    // highlight the extreme points of the body part rect
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax); 
    cv::Mat bodyPartPixelLabels;
    try
    {
      bodyPartPixelLabels = _pixelLabels.at(bodyPart.getPartID());
    }
    catch (...)
    {
      std::stringstream ss;
      ss << "Can't get pixesLabels [" << bodyPart.getPartID() << "]";
      DebugMessage(ss.str(), 1);
      throw std::out_of_range(ss.str());
    }
    // Scan the area near the bodypart center

    auto searchXMin = static_cast<int32_t>(boxCenter.x - boneLength * 0.5f);
    auto searchXMax = static_cast<int32_t>(boxCenter.x + boneLength * 0.5f);
    auto searchYMin = static_cast<int32_t>(boxCenter.y - boneLength * 0.5f);
    auto searchYMax = static_cast<int32_t>(boxCenter.y + boneLength * 0.5f);

    for (auto i = searchXMin; i < searchXMax; i++)
    {
      for (auto j = searchYMin; j < searchYMax; j++)
      {
        // if the point is within the image
        if (i < maskMat.cols && j < maskMat.rows && i >= 0 && j >= 0) 
        {
          // if the point within the highlight area
          if (i <= xmax && i >= xmin && j <= ymax && j >= ymin) 
          {
            // if the point belongs to the rectangle
            if (rect.containsPoint(cv::Point2f(static_cast<float>(i), 
              static_cast<float>(j))) > 0) 
            {
              // counting of the contained pixels
              totalPixels++; 
              auto mintensity = 0;
              try
              {
                // copy current point mask value 
                mintensity = maskMat.at<uint8_t>(j, i); 
              }
              catch (...)
              {
                std::stringstream ss;
                ss << "Can't get maskMat [" << j << "][" << i << "]";
                DebugMessage(ss.str(), 1);               
                throw std::out_of_range(ss.str());
              }
              // pixel is not significant if the mask value is less 
              // than this threshold
              if (mintensity >= 10)
              {
                try
                {
                  if (bodyPartPixelLabels.at<float>(j, i))
                  {
                    // Accumulation of the pixel labels
                    totalPixelLabelScore += 
                      bodyPartPixelLabels.at<float>(j, i); 
                  }
                }
                catch (...)
                {
                  std::stringstream ss;
                  ss << "Can't get pixesLabels [" << bodyPart.getPartID() << 
                    "][" << j << "][" << i << "]";
                  DebugMessage(ss.str(), 1);
                  throw std::out_of_range(ss.str());
                }
                pixelsInMask++; // counting pixels within the mask
              }
            }
          }
        }
      }
    }
    const auto inMaskSuppWeight = 0.5f;
    if (totalPixelLabelScore > 0.0f && totalPixels > 10)
    {
      auto supportScore = totalPixelLabelScore / 
        static_cast<float>(totalPixels);
      auto inMaskSupportScore = totalPixelLabelScore / 
        static_cast<float>(pixelsInMask);
      return 1.0f - ((1.0f - inMaskSuppWeight) * supportScore + 
        inMaskSuppWeight * inMaskSupportScore);
    }
    const std::string str = "Dirty label!";
    DebugMessage(str, 2);
    return -1.0f;
  }

  LimbLabel ColorHistDetector::generateLabel(const BodyPart &bodyPart, 
    Frame *frame, const cv::Point2f &j0, 
    const cv::Point2f &j1, DetectorHelper *detectorHelper, 
    std::map <std::string, float> params) const
  {
    std::stringstream detectorName;
    detectorName << getID();

    params.emplace(COMMON_DETECTOR_PARAMETERS::USE_CH_DETECTOR());

    ColorHistDetectorHelper* helper = 0;
    try
    {
      helper = dynamic_cast<ColorHistDetectorHelper*> (detectorHelper);
    }
    catch (...)
    {
      const std::string str = 
        "Wrong type: detectorHelper is not ColorHistDetectorHelper";
      DebugMessage(str, 1);
      throw std::invalid_argument(str);
    }

    const auto &comparer = [&]()
    {
      return compare(bodyPart, frame, helper->pixelLabels, j0, j1);
    };

    auto label = Detector::generateLabel(bodyPart, j0, j1, detectorName.str(), 
      params.at(COMMON_DETECTOR_PARAMETERS::USE_CH_DETECTOR().first), 
      comparer);

    return label;
  }

  //Used only as prevent a warning for "const uint8_t nBins";
  std::vector <Frame*> ColorHistDetector::getFrames() const noexcept
  {
    return frames;
  }

  //Used only as prevent a warning for "const uint8_t nBins";
  ColorHistDetector &ColorHistDetector::operator=(
    const ColorHistDetector &c) noexcept
  {
    this->frames = c.getFrames();
    return *this;
  }

  ColorHistDetectorHelper::ColorHistDetectorHelper() noexcept
  {
  }

  ColorHistDetectorHelper::~ColorHistDetectorHelper(void) noexcept
  {
    for (auto &p : pixelLabels)
    p.second.release();
  }
}
