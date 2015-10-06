#include "colorHistDetector.hpp"

#define ERROR_HEADER __FILE__ << ":" << __LINE__ << ": "
namespace SPEL
{
  // PartModel Constructor 
  // Initialization "partHistogram" and "bgHistogram" with _nBins^3 elements capacity 3-D arrays 
  ColorHistDetector::PartModel::PartModel(uint8_t _nBins) : nBins(_nBins)
  {
    if (_nBins == 0)
      throw std::invalid_argument("nBins can't be zero");

    partHistogram.resize(nBins, std::vector<std::vector<float>>(nBins, std::vector<float>(nBins, 0.0)));
    bgHistogram.resize(nBins, std::vector<std::vector<float>>(nBins, std::vector<float>(nBins, 0.0)));

    sizeFG = 0;
    sizeBG = 0;
    fgNumSamples = 0;
    bgNumSamples = 0;
  }

  // Copy all fields of the "PartModel" structure
  ColorHistDetector::PartModel &ColorHistDetector::PartModel::operator=(const PartModel &model) noexcept
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
      throw std::logic_error("nBins can't be zero");

    return static_cast<uint8_t> (ceil(pow(2, 8) / nBins));
  }

  // Returns relative frequency of the RGB-color reiteration in "PartModel" 
  float ColorHistDetector::PartModel::computePixelBelongingLikelihood(const uint8_t &r, const uint8_t &g, const uint8_t &b) const
  {
    if (nBins == 0)
      throw std::logic_error("nBins can't be zero");

    // Scaling of colorspace, finding the colors interval, which now gets this color
    auto factor = calculateFactor();
    try
    {
      return partHistogram.at(r / factor).at(g / factor).at(b / factor); // relative frequency of current color reiteration 
    }
    catch (...)
    {
      std::stringstream ss;
      ss << "Couldn't find partHistogram " << "[" << (int)r / factor << "][" << (int)g / factor << "][" << (int)b / factor << "]";
      throw std::out_of_range(ss.str());
    }
  }

  // Build into the "partModel" a histogram of the color set "partColors"
  void ColorHistDetector::PartModel::setPartHistogram(const std::vector <cv::Point3i> &partColors)
  {
    if (nBins == 0)
      throw std::logic_error("nBins can't be zero");

    // do not add sample if the number of pixels is zero
    if (partColors.size() == 0)
      return;
    auto factor = calculateFactor(); // colorspace scaling coefficient
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

    // Scaling of colorspace, reducing the capacity and number of colour intervals that are used to construct the histogram
    for (auto i : partColors)
    {
      auto r = static_cast<uint8_t> (i.x / factor);
      auto g = static_cast<uint8_t> (i.y / factor);
      auto b = static_cast<uint8_t> (i.z / factor);

      if (r >= nBins || g >= nBins || b >= nBins)
      {
        std::stringstream ss;
        ss << "RGB value can't be greater " << nBins - 1 << ": r = " << r << " g = " << g << " b = " << b << std::endl;
        throw std::out_of_range(ss.str());
      }

      partHistogram.at(r).at(g).at(b)++; // increment the frequency of interval, that this color have hit
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

  // Take stock of the additional set of colors in the histogram
  void ColorHistDetector::PartModel::addPartHistogram(const std::vector <cv::Point3i> &partColors, const uint32_t &nBlankPixels)
  {
    if (partColors.size() == 0) //do not add sample if the number of pixels is zero
      return;
    //un-normalise
    if (partHistogram.size() != nBins)
    {
      std::stringstream ss;
      ss << "Wrond size of partHistogram. Expected: " << nBins << ". Actual: " << partHistogram.size() << std::endl;
      throw std::logic_error(ss.str());
    }
    for (auto r = 0; r < nBins; r++)
    {
      if (partHistogram.at(r).size() != nBins)
      {
        std::stringstream ss;
        ss << "Wrond size of partHistogram[" << r << "]. Expected: " << nBins << ". Actual: " << partHistogram.at(r).size() << std::endl;
        throw std::logic_error(ss.str());
      }
      for (auto g = 0; g < nBins; g++)
      {
        if (partHistogram.at(r).at(g).size() != nBins)
        {
          std::stringstream ss;
          ss << "Wrond size of partHistogram[" << r << "][" << g << "]. Expected: " << nBins << ". Actual: " << partHistogram.at(r).at(g).size() << std::endl;
          throw std::logic_error(ss.str());
        }
        for (auto b = 0; b < nBins; b++)
          partHistogram.at(r).at(g).at(b) *= static_cast<float>(sizeFG); // converting the colors relative frequency into the pixels number
      }
    }

    auto factor = calculateFactor(); // colorspace scaling coefficient
    sizeFG += static_cast <uint32_t> (partColors.size());

    if (sizeFG == 0)
      std::logic_error("sizeFG can't be zero");

    fgNumSamples++;
    fgSampleSizes.push_back(static_cast <uint32_t> (partColors.size()));

    // Scaling of colorspace, reducing the capacity and number of colour intervals
    // Adjustment of the histogram
    for (auto color : partColors)
    {
      auto r = static_cast<uint8_t> (color.x / factor);
      auto g = static_cast<uint8_t> (color.y / factor);
      auto b = static_cast<uint8_t> (color.z / factor);

      if (r >= nBins || g >= nBins || b >= nBins)
      {
        std::stringstream ss;
        ss << "RGB value can't be greater " << nBins - 1 << ": r = " << r << " g = " << g << " b = " << b << std::endl;
        throw std::out_of_range(ss.str());
      }

      partHistogram.at(r).at(g).at(b)++; // increment the frequency of interval, that this color have hit
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

    fgBlankSizes.push_back(nBlankPixels); // add the number of blank pixels for this model
  }

  // Totalization the number of used samples
  float ColorHistDetector::PartModel::getAvgSampleSizeFg(void) const
  {
    if (fgNumSamples == 0 && fgSampleSizes.size() > 0)
      std::logic_error("fgNumSamples can't be zero");

    auto sum = 0.0f;
    for (const auto &i : fgSampleSizes)
      sum += i;
    sum /= static_cast<float>(fgNumSamples);
    return sum;
  }

  // Averaging the number of samples, that united from two sets
  float ColorHistDetector::PartModel::getAvgSampleSizeFgBetween(const uint32_t &s1, const uint32_t &s2) const
  {
    if (s1 >= fgSampleSizes.size() || s2 >= fgSampleSizes.size())
    {
      std::stringstream ss;
      ss << "Incorrect parameter. s1: " << s1 << " s2: " << s2 << " Actual size: " << fgSampleSizes.size() << std::endl;
      throw std::invalid_argument(ss.str());
    }
    return (fgSampleSizes.at(s1) + fgSampleSizes.at(s2)) / 2.0f;
  }

  //TODO (Vitaliy Koshura): Need unit test
  // Euclidean distance between part histograms
  float ColorHistDetector::PartModel::matchPartHistogramsED(const PartModel &partModelPrev) const
  {
    if (nBins == 0)
      throw std::logic_error("nBins can't be zero");

    if (nBins != partModelPrev.nBins)
    {
      std::stringstream ss;
      ss << "Different nBins value. Expected: " << nBins << " Actual: " << partModelPrev.nBins << std::endl;
      std::logic_error(ss.str());
    }

    auto distance = 0.0f;

    if (partHistogram.size() != nBins)
    {
      std::stringstream ss;
      ss << "Wrond size of partHistogram. Expected: " << nBins << ". Actual: " << partHistogram.size() << std::endl;
      throw std::logic_error(ss.str());
    }
    if (partModelPrev.partHistogram.size() != nBins)
    {
      std::stringstream ss;
      ss << "Wrond size of partHistogram. Expected: " << nBins << ". Actual: " << partModelPrev.partHistogram.size() << std::endl;
      throw std::logic_error(ss.str());
    }
    for (auto r = 0; r < nBins; r++)
    {
      if (partHistogram.at(r).size() != nBins)
      {
        std::stringstream ss;
        ss << "Wrond size of partHistogram [" << r << "]. Expected: " << nBins << ". Actual: " << partHistogram.at(r).size() << std::endl;
        throw std::logic_error(ss.str());
      }
      if (partModelPrev.partHistogram.at(r).size() != nBins)
      {
        std::stringstream ss;
        ss << "Wrond size of partHistogram [" << r << "]. Expected: " << nBins << ". Actual: " << partModelPrev.partHistogram.at(r).size() << std::endl;
        throw std::logic_error(ss.str());
      }
      for (auto g = 0; g < nBins; g++)
      {
        if (partHistogram.at(r).at(g).size() != nBins)
        {
          std::stringstream ss;
          ss << "Wrond size of partHistogram [" << r << "][" << g << "]. Expected: " << nBins << ". Actual: " << partHistogram.at(r).at(g).size() << std::endl;
          throw std::logic_error(ss.str());
        }
        if (partModelPrev.partHistogram.at(r).at(g).size() != nBins)
        {
          std::stringstream ss;
          ss << "Wrond size of partHistogram [" << r << "][" << g << "]. Expected: " << nBins << ". Actual: " << partModelPrev.partHistogram.at(r).at(g).size() << std::endl;
          throw std::logic_error(ss.str());
        }
        for (auto b = 0; b < nBins; b++)
          // accumulation of the Euclidean distances between the points
          distance += pow(partHistogram.at(r).at(g).at(b) - partModelPrev.partHistogram.at(r).at(g).at(b), 2);
      }
    }
    return sqrt(distance);
  }

  // Background histogram
  void ColorHistDetector::PartModel::addBackgroundHistogram(const std::vector <cv::Point3i> &bgColors)
  {
    if (bgColors.size() == 0)
      return;

    if (nBins == 0)
      throw std::logic_error("nBins can't be zero");

    // unnormalise
    if (bgHistogram.size() != nBins)
    {
      std::stringstream ss;
      ss << "Wrond size of bgHistogram. Expected: " << nBins << ". Actual: " << bgHistogram.size() << std::endl;
      throw std::logic_error(ss.str());
    }
    for (auto r = 0; r < nBins; r++)
    {
      if (bgHistogram.at(r).size() != nBins)
      {
        std::stringstream ss;
        ss << "Wrond size of bgHistogram [" << r << "]. Expected: " << nBins << ". Actual: " << bgHistogram.at(r).size() << std::endl;
        throw std::logic_error(ss.str());
      }
      for (auto g = 0; g < nBins; g++)
      {
        if (bgHistogram.at(r).at(g).size() != nBins)
        {
          std::stringstream ss;
          ss << "Wrond size of bgHistogram[" << r << "][" << g << "]. Expected: " << nBins << ". Actual: " << bgHistogram.at(r).at(g).size() << std::endl;
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
      std::logic_error("sizeBG can't be zero");

    for (const auto &color : bgColors)
    {
      auto r = static_cast<uint8_t> (color.x / factor);
      auto g = static_cast<uint8_t> (color.y / factor);
      auto b = static_cast<uint8_t> (color.z / factor);

      if (r >= nBins || g >= nBins || b >= nBins)
      {
        std::stringstream ss;
        ss << "RGB value can't be greater " << nBins - 1 << ": r = " << r << " g = " << g << " b = " << b << std::endl;
        throw std::out_of_range(ss.str());
      }

      bgHistogram.at(r).at(g).at(b)++; // increment the frequency of interval, that this color have hit
    }
    // renormalise
    for (auto r = 0; r < nBins; r++)
      for (auto g = 0; g < nBins; g++)
        for (auto b = 0; b < nBins; b++)
          bgHistogram.at(r).at(g).at(b) /= static_cast<float>(sizeBG);
  }

  // Constructor with initialization of constant field "nBins"
  ColorHistDetector::ColorHistDetector(uint8_t _nBins) : nBins(_nBins)
  {
    if (_nBins == 0)
    {
      std::stringstream ss;
      ss << "nBins can't be zero";
      throw std::invalid_argument(ss.str());
    }
    id = 0x434844;
  }

  ColorHistDetector::~ColorHistDetector(void) noexcept
  {
  }

  // Returns unique ID of "ColorHistDetector" object
  int ColorHistDetector::getID(void) const noexcept
  {
    return id;
  }

  // Change ID of "ColorHistDetector" object
  void ColorHistDetector::setID(const int &_id) noexcept
  {
    id = _id;
  }

  // Builds a histograms of all polygons for pre-marked frames
  void ColorHistDetector::train(const std::vector <Frame*> &_frames, std::map <std::string, float> params)
  {
    frames = _frames; // vector of pointers - presents a sequence of frames
    sort(frames.begin(), frames.end(), Frame::FramePointerComparer); // sorting frames by id

    if (frames.size() == 0)
      throw std::logic_error("No input frames"); // the sequence of frames is empty
    partModels.clear();
    // Find skeleton from first keyframe or lockframe
    Skeleton skeleton;

    params.emplace(COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().first, frames.at(0)->getFrameSize().height);

    maxFrameHeight = static_cast<uint32_t>(params.at(COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().first));

    auto bFind = false; // flag, indicate the presence of marked frame in the sequence
    for (auto f : frames)
    {
      if (f->getFrametype() == KEYFRAME || f->getFrametype() == LOCKFRAME)
      {
        skeleton = f->getSkeleton();
        bFind = true; // marked frame was found
        break;
      }
    }
    if (bFind == false)
    {
      if (debugLevel >= 1)
        std::cerr << ERROR_HEADER << "No neither keyframes nor lockframes" << std::endl;
      throw std::logic_error("No neither keyframes nor lockframes");
    }

    tree <BodyPart> partTree;
    // Handling all frames
    for (const auto &frameNum : frames)
    {
      if (frameNum->getFrametype() != KEYFRAME && frameNum->getFrametype() != LOCKFRAME)
        continue; // skip unmarked frames

      Frame *workFrame = nullptr;
      if (frameNum->getFrametype() == KEYFRAME)
        workFrame = new Keyframe();
      else if (frameNum->getFrametype() == LOCKFRAME)
        workFrame = new Lockframe();
      else if (frameNum->getFrametype() == INTERPOLATIONFRAME)
        workFrame = new Interpolation();

      if (workFrame == nullptr)
      {
        std::stringstream ss;
        ss << "Unknown frame found";
        if (debugLevel >= 1)
          std::cerr << ERROR_HEADER << ss.str() << std::endl;
        throw std::logic_error(ss.str());
      }

      workFrame = frameNum->clone(workFrame);

      workFrame->Resize(maxFrameHeight);

      if (debugLevel >= 2)
        std::cout << "Training on frame " << workFrame->getID() << std::endl;
      // Create local variables
      std::map <int32_t, std::vector <cv::Point3i>> partPixelColours; // the set of RGB-colours of pixel's for current body part
      std::map <int32_t, std::vector <cv::Point3i>> bgPixelColours; // the set of RGB-colours for a pixels of background
      std::map <int32_t, int> blankPixels;  // pixels outside the mask
      skeleton = workFrame->getSkeleton(); // copy marking from current frame
      std::multimap <int32_t, POSERECT <cv::Point2f>> polygons;  // polygons for this frame
      std::multimap <int32_t, float> polyDepth; // used for evaluation of overlapped polygons
      partTree = skeleton.getPartTree(); // the skeleton body parts
      // Handling all bodyparts on the frames
      for (auto &bodyPart : partTree)
      {
        partPixelColours.insert(std::pair <int32_t, std::vector <cv::Point3i>>(bodyPart.getPartID(), std::vector <cv::Point3i>())); // container initialization for conserve colours set for each of body parts
        bgPixelColours.insert(std::pair <int32_t, std::vector <cv::Point3i>>(bodyPart.getPartID(), std::vector <cv::Point3i>())); // container initialization for conserve background colours set for each of body parts
        blankPixels.insert(std::pair <int32_t, int>(bodyPart.getPartID(), 0)); // container initialization for counting blank pixels for each of body parts

        auto joint = skeleton.getBodyJoint(bodyPart.getParentJoint()); // the parent node of current body part pointer 
        if (joint == 0)
        {
          if (debugLevel >= 1)
            std::cerr << ERROR_HEADER << "Invalid parent joint" << std::endl;
          break; // a joint has no marking on the frame
        }
        auto j0 = joint->getImageLocation(); // coordinates of current joint
        joint = 0;
        joint = skeleton.getBodyJoint(bodyPart.getChildJoint()); // the child node of current body part pointer
        if (joint == 0)
        {
          if (debugLevel >= 1)
            std::cerr << ERROR_HEADER << "Invalid child joint" << std::endl;
          break; // a joint has no marking on the frame
        }
        auto j1 = joint->getImageLocation(); // coordinates of current joint
        auto direction = j1 - j0; // used as estimation of the vector's direction
        auto rotationAngle = static_cast<float>(spelHelper::angle2D(1.0f, 0.0f, direction.x, direction.y) * (180.0f / M_PI)); //bodypart tilt angle 
        bodyPart.setRotationSearchRange(rotationAngle);
        auto poserect = getBodyPartRect(bodyPart, j0, j1);
        polygons.insert(std::pair <int32_t, POSERECT <cv::Point2f>>(bodyPart.getPartID(), poserect));
        polyDepth.insert(std::pair <int32_t, float>(bodyPart.getPartID(), skeleton.getBodyJoint(bodyPart.getParentJoint())->getSpaceLocation().z));
      }
      skeleton.setPartTree(partTree);
      workFrame->setSkeleton(skeleton);
      auto maskMat = workFrame->getMask(); // copy mask from the current frame
      auto imgMat = workFrame->getImage(); // copy image from the current frame
      // Range over all pixels of the frame
      for (auto i = 0; i < imgMat.cols; i++)
      {
        for (auto j = 0; j < imgMat.rows; j++)
        {
          cv::Vec3b intensity;
          try
          {
            intensity = imgMat.at<cv::Vec3b>(j, i);  // copy RGB color of current pixel
          }
          catch (...)
          {
            std::stringstream ss;
            ss << "Couldn't get imgMat value of indeces " << "[" << j << "][" << i << "]";
            if (debugLevel >= 1)
              std::cerr << ERROR_HEADER << ss.str() << std::endl;
            throw std::out_of_range(ss.str());
          }
          // Copy the current pixel colour components
          auto blue = intensity.val[0];
          auto green = intensity.val[1];
          auto red = intensity.val[2];
          auto mintensity = 0;
          try
          {
            mintensity = maskMat.at<uint8_t>(j, i);  // copy current pixel mask value 
          }
          catch (...)
          {
            std::stringstream ss;
            ss << "Couldn't get maskMat value of indeces " << "[" << j << "][" << i << "]";
            if (debugLevel >= 1)
              std::cerr << ERROR_HEADER << ss.str() << std::endl;
            throw std::out_of_range(ss.str());
          }
          auto blackPixel = mintensity < 10;
          auto partHit = -1; // will be equal to -1 until is not found polygon, which contains the point
          auto depth = 0.0f;
          // Handling all poligons
          for (const auto &bodyPart : partTree)
          {
            auto partNumber = bodyPart.getPartID();
            auto bContainsPoint = false;
            std::vector <POSERECT <cv::Point2f>> partPolygons;
            // Copy poligons to "PartPoligons"
            auto lower = polygons.lower_bound(partNumber), upper = polygons.upper_bound(partNumber);
            transform(lower, upper, back_inserter(partPolygons), [](auto const &pair) { return pair.second; });
            // Checking whether a pixel belongs to the current and to another polygons            
            for (auto partPolygon : partPolygons)
              if ((bContainsPoint = partPolygon.containsPoint(cv::Point2f(static_cast<float>(i), static_cast<float>(j))) > 0) == true)
                break; // was found polygon, which contain current pixel

            std::vector <float> partDepths;
            auto lowerP = polyDepth.lower_bound(partNumber), upperP = polyDepth.upper_bound(partNumber);
            transform(lowerP, upperP, back_inserter(partDepths), [](auto const &pair) { return pair.second; }); // copy "polyDepth" to "PartDepth"
            // Checkig polygons overlapping
            for (const auto &partDepth : partDepths)
            {
              if (bContainsPoint && partHit == -1)
              {
                partHit = partNumber; // store the number of the first found polygon
                depth = partDepth;
              }
              else if (bContainsPoint && partDepth < depth) // How, for float tempDepthSign?/////////////////
              {
                partHit = partNumber;
                depth = partDepth;
              }
            }
          }
          if (partHit != -1) // if was found polygon, that contains this pixel
          {
            if (!blackPixel) // if pixel color isn't black
            {
              partPixelColours.at(partHit).push_back(cv::Point3i(red, green, blue)); // add colour of this pixel to part[partHit] colours

              // For all bodyparts
              for (const auto &p : partTree)
                if (p.getPartID() != partHit) // if current poligon wasn't found first in the previous enumeration???
                  bgPixelColours.at(p.getPartID()).push_back(cv::Point3i(red, green, blue)); // add colour of this pixel to part[partHit] background colours
            }
            else
              blankPixels.at(partHit)++; // otherwise take stock this pixel to blank pixel counter
          }
          else // if  not found polygon, that contains this pixel 
            for (const auto &p : partTree)
              bgPixelColours.at(p.getPartID()).push_back(cv::Point3i(red, green, blue));
        }
      }

      // Create model for each bodypart
      for (const auto &bodyPart : partTree)
      {
        auto partNumber = bodyPart.getPartID();
        if (partModels.find(partNumber) == partModels.end())
          partModels.insert(std::pair <int32_t, PartModel>(partNumber, PartModel(nBins))); //add a new model to end of models list

        auto partModel = partModels.at(partNumber);
        auto partPixelColoursVector = partPixelColours.at(partNumber); // copy part color set for current bodypart
        auto blankPixelsCount = blankPixels.at(partNumber);  // copy blanck pixel count for current bodypart
        auto bgPixelColoursVector = bgPixelColours.at(partNumber); // copy background color set for current bodypart

        partModel.addPartHistogram(partPixelColoursVector, blankPixelsCount); // building histogram for current bodypart colours
        partModel.addBackgroundHistogram(bgPixelColoursVector); // building histograms for current bodypart background colours
        partModels.at(partNumber) = partModel; // copy result to part models set
        if (debugLevel >= 2)
          std::cout << "Found part model: " << partNumber << std::endl;
      }
      delete workFrame;
    }
  }
  
  // Returns a labels vector of possible body parts position
  std::map <uint32_t, std::vector <LimbLabel> > ColorHistDetector::detect(const Frame *frame, std::map <std::string, float> params, const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels) const 
  {
    params.emplace(COMMON_DETECTOR_PARAMETERS::USE_CH_DETECTOR());

    auto detectorHelper = new ColorHistDetectorHelper();

    detectorHelper->pixelDistributions = buildPixelDistributions(frame); // matrix contains the probability that the particular pixel belongs to current bodypart
    detectorHelper->pixelLabels = buildPixelLabels(frame, detectorHelper->pixelDistributions); // matrix contains relative estimations that the particular pixel belongs to current bodypart

    auto result = Detector::detect(frame, params, limbLabels, detectorHelper);

    delete detectorHelper;

    return result;
  }

  // Return nBins
  uint8_t ColorHistDetector::getNBins(void) const noexcept
  {
    return nBins;
  }

  // Returns a matrix, that contains relative frequency of the pixels colors reiteration 
  std::map <int32_t, cv::Mat> ColorHistDetector::buildPixelDistributions(const Frame *frame) const 
  {
    auto skeleton = frame->getSkeleton(); // copy skeleton from the frame
    auto partTree = skeleton.getPartTree(); // copy part tree from the skeleton
    auto imgMat = frame->getImage(); // copy image from the frame
    auto maskMat = frame->getMask(); // copy mask from the frame
    auto width = imgMat.cols;
    auto height = imgMat.rows;
    auto mwidth = maskMat.cols;
    auto mheight = maskMat.rows;
    std::map <int32_t, cv::Mat> tempPixelDistributions;
    if (width != mwidth || height != mheight) // error if mask and image sizes don't match
    {
      std::stringstream ss;
      ss << "Mask size not equal image size";
      if (debugLevel >= 1)
        std::cerr << ERROR_HEADER << ss.str() << std::endl;
      throw std::logic_error(ss.str());
    }
    // For all bodyparts
    for (const auto &bodyPart : partTree)
    {
      auto t = cv::Mat(height, width, cv::DataType <float>::type); // create empty matrix
      auto partID = bodyPart.getPartID();
      try
      {
        auto partModel = partModels.at(partID); // copy part model of current bodybart
        // For all pixels
        for (auto x = 0; x < width; x++)
        {
          for (auto y = 0; y < height; y++)
          {
            auto intensity = imgMat.at<cv::Vec3b>(y, x);
            // Copy components of the current pixel color
            auto blue = intensity.val[0];
            auto green = intensity.val[1];
            auto red = intensity.val[2];
            auto mintensity = maskMat.at<uint8_t>(y, x); // copy mask of the current pixel
            auto blackPixel = mintensity < 10; // pixel is not significant if the mask value is less than this threshold

            t.at<float>(y, x) = blackPixel ? 0 : partModel.computePixelBelongingLikelihood(red, green, blue); // relative frequency of the current pixel color reiteration 
          }
        }
      }
      catch (...)
      {
        std::stringstream ss;
        ss << "Maybe couldn't find partModel " << partID;
        if (debugLevel >= 1)
          std::cerr << ERROR_HEADER << ss.str() << std::endl;
        throw std::logic_error(ss.str());
      }
      tempPixelDistributions.insert(std::pair <int32_t, cv::Mat>(partID, t)); // add the current bodypart matrix to the set 
    }
    return tempPixelDistributions;
  }

  std::map <int32_t, cv::Mat> ColorHistDetector::buildPixelLabels(const Frame *frame, const std::map <int32_t, cv::Mat> &_pixelDistributions) const 
  {
    auto maskMat = frame->getMask(); // copy mask from the frame
    auto width = maskMat.cols;
    auto height = maskMat.rows;
    auto skeleton = frame->getSkeleton(); // copy skeleton from the frame
    auto partTree = skeleton.getPartTree(); // copy part tree from the skeleton
    std::map <int32_t, cv::Mat> _pixelLabels;
    // For all body parts
    for (const auto &bodyPart : partTree)
    {
      auto t = cv::Mat(height, width, cv::DataType <float>::type); // create empty matrix
      cv::Mat tt;
      try
      { // Matrix, that contains relative frequency of the pixels colors reiteration for current body part
        tt = _pixelDistributions.at(bodyPart.getPartID());
      }
      catch (...)
      {
        std::stringstream ss;
        ss << "Couldn't find distributions for body part " << bodyPart.getPartID();
        if (debugLevel >= 1)
          std::cerr << ERROR_HEADER << ss.str() << std::endl;
        throw std::out_of_range(ss.str());
      }
      // For all pixels
      for (auto x = 0; x < width; x++)
      {
        for (auto y = 0; y < height; y++)
        {
          auto mintensity = maskMat.at<uint8_t>(y, x); //copy the current pixel mask value
          auto blackPixel = mintensity < 10; // pixel is not significant if the mask value is less than this threshold
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
                temp = _pixelDistributions.at(i.getPartID()); // matrix of the pixels colors frequency for current body part
              }
              catch (...)
              {
                std::stringstream ss;
                ss << "Couldn't find pixel distributions for body part " << i.getPartID();
                if (debugLevel >= 1)
                  std::cerr << ERROR_HEADER << ss.str() << std::endl;
                throw std::out_of_range(ss.str());
              }
              try
              {
                if (temp.at<float>(y, x) > top) // search max value of the current bodypart pixel color frequency
                  top = temp.at<float>(y, x);
                sum += temp.at<float>(y, x);
              }
              catch (...)
              {
                std::stringstream ss;
                ss << "Couldn't find value of temp " << "[" << y << "][" << x << "]";
                if (debugLevel >= 1)
                  std::cerr << ERROR_HEADER << ss.str() << std::endl;
                throw std::out_of_range(ss.str());
              }
            }
            try
            {
              t.at<float>(y, x) = (top == 0) ? 0 : tt.at<float>(y, x) / static_cast<float>(top);
            }
            catch (...)
            {
              std::stringstream ss;
              ss << "Couldn't find t " << "[" << y << "][" << x << "] or tt [" << y << "][" << x << "]";
              if (debugLevel >= 1)
                std::cerr << ERROR_HEADER << ss.str() << std::endl;
              throw std::out_of_range(ss.str());
            }
          }
          else
          {
            try
            {
              t.at<float>(y, x) = 0;
            }
            catch (...)
            {
              std::stringstream ss;
              ss << "Couldn't find value of t " << "[" << y << "][" << x << "]";
              if (debugLevel >= 1)
                std::cerr << ERROR_HEADER << ss.str() << std::endl;
              throw std::out_of_range(ss.str());
            }
          }
        }
      }
      _pixelLabels.insert(std::pair<int32_t, cv::Mat>(bodyPart.getPartID(), t)); // insert the resulting matrix into the set "pixelLabels" 
    }
    return _pixelLabels;
  }

  float ColorHistDetector::compare(const BodyPart &bodyPart, const Frame *frame, const std::map <int32_t, cv::Mat> &_pixelDistributions, const std::map <int32_t, cv::Mat> &_pixelLabels, const cv::Point2f &j0, const cv::Point2f &j1) const
  {
    auto maskMat = frame->getMask(); // copy mask from the frame 
    auto imgMat = frame->getImage(); // copy image from the frame
    auto boxCenter = j0 * 0.5 + j1 * 0.5; // segment center
    auto boneLength = getBoneLength(j0, j1); // distance between joints
    auto rect = getBodyPartRect(bodyPart, j0, j1); // expected bodypart location area?
    auto totalPixels = 0;
    auto pixelsInMask = 0;
    auto totalPixelLabelScore = 0.0f;
    auto pixDistAvg = 0.0f;
    auto pixDistNum = 0.0f;
    PartModel model;
    try
    {
      model = partModels.at(bodyPart.getPartID()); // copy part model for the "bodyPart"
    }
    catch (...)
    {
      std::stringstream ss;
      ss << "Couldn't get partModel of bodyPart " << bodyPart.getPartID();
      if (debugLevel >= 1)
        std::cerr << ERROR_HEADER << ss.str() << std::endl;
      throw std::out_of_range(ss.str());
    }
    if (model.getAvgSampleSizeFg() == 0) // error if samples count is zero
    {
      std::stringstream ss;
      ss << "Couldn't get avgSampleSizeFg";
      if (debugLevel >= 2)
        std::cerr << ERROR_HEADER << ss.str() << std::endl;
      throw std::out_of_range(ss.str());
    }
    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax); // highlight the extreme points of the body part rect
    cv::Mat bodyPartPixelDistribution;
    try
    {
      bodyPartPixelDistribution = _pixelDistributions.at(bodyPart.getPartID());
    }
    catch (...)
    {
      std::stringstream ss;
      ss << "Can't get pixesDistribution [" << bodyPart.getPartID() << "]";
      if (debugLevel >= 2)
        std::cerr << ERROR_HEADER << ss.str() << std::endl;
      throw std::out_of_range(ss.str());
    }
    cv::Mat bodyPartLixelLabels;
    try
    {
      bodyPartLixelLabels = _pixelLabels.at(bodyPart.getPartID());
    }
    catch (...)
    {
      std::stringstream ss;
      ss << "Can't get pixesLabels [" << bodyPart.getPartID() << "]";
      if (debugLevel >= 2)
        std::cerr << ERROR_HEADER << ss.str() << std::endl;
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
        if (i < maskMat.cols && j < maskMat.rows && i >= 0 && j >= 0) // if the point is within the image
        {
          if (i <= xmax && i >= xmin && j <= ymax && j >= ymin) // if the point within the highlight area
          {
            if (rect.containsPoint(cv::Point2f(static_cast<float>(i), static_cast<float>(j))) > 0) // if the point belongs to the rectangle
            {
              totalPixels++; // counting of the contained pixels
              auto mintensity = 0;
              try
              {
                mintensity = maskMat.at<uint8_t>(j, i); // copy current point mask value 
              }
              catch (...)
              {
                std::stringstream ss;
                ss << "Can't get maskMat [" << j << "][" << i << "]";
                if (debugLevel >= 2)
                  std::cerr << ERROR_HEADER << ss.str() << std::endl;
                throw std::out_of_range(ss.str());
              }
              auto blackPixel = mintensity < 10; // pixel is not significant if the mask value is less than this threshold
              if (!blackPixel)
              {
                try
                {
                  pixDistAvg += bodyPartPixelDistribution.at<float>(j, i); // Accumulation the "distributions" of contained pixels
                }
                catch (...)
                {
                  std::stringstream ss;
                  ss << "Can't get pixesDistribution [" << bodyPart.getPartID() << "][" << j << "][" << i << "]";
                  if (debugLevel >= 2)
                    std::cerr << ERROR_HEADER << ss.str() << std::endl;
                  throw std::out_of_range(ss.str());
                }
                pixDistNum++; // counting of the all scanned pixels
                try
                {
                  if (bodyPartLixelLabels.at<float>(j, i))
                  {
                    totalPixelLabelScore += bodyPartLixelLabels.at<float>(j, i); // Accumulation of the pixel labels
                  }
                }
                catch (...)
                {
                  std::stringstream ss;
                  ss << "Can't get pixesLabels [" << bodyPart.getPartID() << "][" << j << "][" << i << "]";
                  if (debugLevel >= 2)
                    std::cerr << ERROR_HEADER << ss.str() << std::endl;
                  throw std::out_of_range(ss.str());
                }
                pixelsInMask++; // counting pixels within the mask
              }
            }
          }
        }
      }
    }
    auto supportScore = 0.0f;
    auto inMaskSupportScore = 0.0f;
    pixDistAvg /= static_cast<float>(pixDistNum);  // average "distributions"
    auto inMaskSuppWeight = 0.5f;
    if (totalPixelLabelScore > 0 && totalPixels > 10)
    {
      supportScore = static_cast<float>(totalPixelLabelScore) / static_cast<float>(totalPixels);
      inMaskSupportScore = static_cast<float>(totalPixelLabelScore) / static_cast<float>(pixelsInMask);
      return 1.0f - ((1.0f - inMaskSuppWeight) * supportScore + inMaskSuppWeight * inMaskSupportScore);
    }
    std::stringstream ss;
    ss << "Dirty label!";
    if (debugLevel >= 2)
      std::cerr << ERROR_HEADER << ss.str() << std::endl;
    throw std::logic_error(ss.str());
  }

  LimbLabel ColorHistDetector::generateLabel(const BodyPart &bodyPart, const Frame *frame, const cv::Point2f &j0, const cv::Point2f &j1, DetectorHelper *detectorHelper, std::map <std::string, float> params) const
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
      std::stringstream ss;
      ss << "Wrong type: detectorHelper is not ColorHistDetectorHelper";
      throw std::invalid_argument(ss.str());
    }

    auto comparer = [&]() -> float 
    {
      return compare(bodyPart, frame, helper->pixelDistributions, helper->pixelLabels, j0, j1);
    };

    auto label = Detector::generateLabel(bodyPart, j0, j1, detectorName.str(), params.at(COMMON_DETECTOR_PARAMETERS::USE_CH_DETECTOR().first), comparer);

    return label;
  }

  //Used only as prevent a warning for "const uint8_t nBins";
  std::vector <Frame*> ColorHistDetector::getFrames() const noexcept
  {
    return frames;
  }

  //Used only as prevent a warning for "const uint8_t nBins";
  ColorHistDetector &ColorHistDetector::operator=(const ColorHistDetector &c) noexcept
  {
    this->frames = c.getFrames();
    return *this;
  }

  ColorHistDetectorHelper::ColorHistDetectorHelper() noexcept
  {
  }

  ColorHistDetectorHelper::~ColorHistDetectorHelper(void) noexcept
  {
    for (auto &p : pixelDistributions)
    p.second.release();
    for (auto &p : pixelLabels)
    p.second.release();
  }

}
