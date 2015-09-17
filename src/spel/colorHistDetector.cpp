#include "colorHistDetector.hpp"

#define ERROR_HEADER __FILE__ << ":" << __LINE__ << ": "
namespace SPEL
{
  // PartModel Constructor 
  // Initialization "partHistogram" and "bgHistogram" with _nBins^3 elements capacity 3-D arrays 
  ColorHistDetector::PartModel::PartModel(uint8_t _nBins) noexcept : nBins(_nBins)
  {
    partHistogram.clear();
    bgHistogram.clear();

    partHistogram.resize(nBins, std::vector<std::vector<float>>(nBins, std::vector<float>(nBins, 0.0)));
    bgHistogram.resize(nBins, std::vector<std::vector<float>>(nBins, std::vector<float>(nBins, 0.0)));

    sizeFG = 0;
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

  // Constructor with initialization of constant field "nBins"
  ColorHistDetector::ColorHistDetector(uint8_t _nBins) noexcept : nBins(_nBins)
  {
    id = 0x434844;
  }

  ColorHistDetector::~ColorHistDetector(void) noexcept
  {
    for (auto &p : pixelDistributions)
      p.second.release();
    for (auto &p : pixelLabels)
      p.second.release();
  }

  // Returns unique ID of "ColorHistDetector" object
  int ColorHistDetector::getID(void) const noexcept
  {
    return id;
  }

  // Change ID of "ColorHistDetector" object
  void ColorHistDetector::setID(int _id) noexcept
  {
    id = _id;
  }

  // Builds a histograms of all polygons for pre-marked frames
  void ColorHistDetector::train(std::vector <Frame*> _frames, std::map <std::string, float> params)
  {
    frames = _frames; // vector of pointers - presents a sequence of frames
    sort(frames.begin(), frames.end(), Frame::FramePointerComparer); // sorting frames by id

    params.emplace(COMMON_SPEL_PARAMETERS::DEBUG_LEVEL());

    debugLevelParam = static_cast <uint8_t> (params.at(COMMON_SPEL_PARAMETERS::DEBUG_LEVEL().first));

    if (frames.size() == 0)
      throw std::logic_error("No input frames"); // the sequence of frames is empty
    partModels.clear();
    // Find skeleton from first keyframe or lockframe
    Skeleton skeleton;

    params.emplace(COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().first, frames.at(0)->getFrameSize().height);

    maxFrameHeight = params.at(COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().first);

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
      if (debugLevelParam >= 1)
        std::cerr << ERROR_HEADER << "No neither keyframes nor lockframes" << std::endl;
      throw std::logic_error("No neither keyframes nor lockframes");
    }

    tree <BodyPart> partTree;
    // Handling all frames
    for (const auto &frameNum : frames)
    {
      if (frameNum->getFrametype() != KEYFRAME && frameNum->getFrametype() != LOCKFRAME)
        continue; // skip unmarked frames

      auto originalSize = frameNum->getFrameSize().height;

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
        if (debugLevelParam >= 1)
          std::cerr << ERROR_HEADER << ss.str() << std::endl;
        throw std::logic_error(ss.str());
      }

      workFrame = frameNum->clone(workFrame);

      workFrame->Resize(maxFrameHeight);

      if (debugLevelParam >= 2)
        std::cerr << "Training on frame " << workFrame->getID() << std::endl;
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
          if (debugLevelParam >= 1)
            std::cerr << ERROR_HEADER << "Invalid parent joint" << std::endl;
          break; // a joint has no marking on the frame
        }
        auto j0 = joint->getImageLocation(); // coordinates of current joint
        joint = 0;
        joint = skeleton.getBodyJoint(bodyPart.getChildJoint()); // the child node of current body part pointer
        if (joint == 0)
        {
          if (debugLevelParam >= 1)
            std::cerr << ERROR_HEADER << "Invalid child joint" << std::endl;
          break; // a joint has no marking on the frame
        }
        auto j1 = joint->getImageLocation(); // coordinates of current joint
        auto direction = j1 - j0; // used as estimation of the vector's direction
        auto rotationAngle = spelHelper::angle2D(1.0f, 0.0f, direction.x, direction.y) * (180.0f / M_PI); //bodypart tilt angle 
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
            if (debugLevelParam >= 1)
              std::cerr << ERROR_HEADER << ss.str() << std::endl;
            throw std::logic_error(ss.str());
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
            if (debugLevelParam >= 1)
              std::cerr << ERROR_HEADER << ss.str() << std::endl;
            throw std::logic_error(ss.str());
          }
          auto blackPixel = mintensity < 10;
          auto partHit = -1; // will be equal to -1 until is not found polygon, which contains the point
          auto depth = 0.0f;
          // Handling all poligons
          for (const auto &bodyPart : partTree)
          {
            auto partNumber = bodyPart.getPartID();
            auto bContainsPoint = false;
            try
            {
              std::vector <POSERECT <cv::Point2f>> partPolygons;
              // Copy poligons to "PartPoligons"
              auto lower = polygons.lower_bound(partNumber), upper = polygons.upper_bound(partNumber);
              transform(lower, upper, back_inserter(partPolygons), [](auto const &pair) { return pair.second; });
              // Checking whether a pixel belongs to the current and to another polygons            
              for (auto partPolygon : partPolygons)
                if ((bContainsPoint = partPolygon.containsPoint(cv::Point2f(static_cast<float>(i), static_cast<float>(j))) > 0) == true)
                  break; // was found polygon, which contain current pixel
            }
            catch (...)
            {
              std::stringstream ss;
              ss << "There is no such polygon for body part " << partNumber;
              if (debugLevelParam >= 1)
                std::cerr << ERROR_HEADER << ss.str() << std::endl;
              throw std::logic_error(ss.str());
            }
            try
            {
              std::vector <float> partDepths;
              auto lower = polyDepth.lower_bound(partNumber), upper = polyDepth.upper_bound(partNumber);
              transform(lower, upper, back_inserter(partDepths), [](auto const &pair) { return pair.second; }); // copy "polyDepth" to "PartDepth"
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
            catch (...)
            {
              std::stringstream ss;
              ss << "There is no such polyDepth parameter for body part " << partNumber;
              if (debugLevelParam >= 1)
                std::cerr << ERROR_HEADER << ss.str() << std::endl;
              throw std::logic_error(ss.str());
            }
          }
          if (partHit != -1) // if was found polygon, that contains this pixel
          {
            if (!blackPixel) // if pixel color isn't black
            {
              try
              {
                partPixelColours.at(partHit).push_back(cv::Point3i(red, green, blue)); // add colour of this pixel to part[partHit] colours
              }
              catch (...)
              {
                std::stringstream ss;
                ss << "There is no partPixelColours for body part " << partHit;
                if (debugLevelParam >= 1)
                  std::cerr << ERROR_HEADER << ss.str() << std::endl;
                throw std::logic_error(ss.str());
              }
              // For all bodyparts
              for (const auto &p : partTree)
              {
                if (p.getPartID() != partHit) // if current poligon wasn't found first in the previous enumeration???
                {
                  try
                  {
                    bgPixelColours.at(p.getPartID()).push_back(cv::Point3i(red, green, blue)); // add colour of this pixel to part[partHit] background colours
                  }
                  catch (...)
                  {
                    std::stringstream ss;
                    ss << "There is no such bgPixelColours for body part " << p.getPartID();
                    if (debugLevelParam >= 1)
                      std::cerr << ERROR_HEADER << ss.str() << std::endl;
                    throw std::logic_error(ss.str());
                  }
                }
              }
            }
            else
            {
              try
              {
                blankPixels.at(partHit)++; // otherwise take stock this pixel to blank pixel counter
              }
              catch (...)
              {
                std::stringstream ss;
                ss << "There is no such blankPixels for body part " << partHit;
                if (debugLevelParam >= 1)
                  std::cerr << ERROR_HEADER << ss.str() << std::endl;
                throw std::logic_error(ss.str());
              }
            }
          }
          else // if  not found polygon, that contains this pixel 
          { // For all bodyparts
            for (const auto &p : partTree)
            {
              try
              {
                bgPixelColours.at(p.getPartID()).push_back(cv::Point3i(red, green, blue));
              }
              catch (...)
              {
                std::stringstream ss;
                ss << "There is no such bgPixelColours for body part " << p.getPartID();
                if (debugLevelParam >= 1)
                  std::cerr << ERROR_HEADER << ss.str() << std::endl;
                throw std::logic_error(ss.str());
              }
            }
          }
        }
      }

      // Create model for each bodypart
      for (const auto &bodyPart : partTree)
      {
        auto partNumber = bodyPart.getPartID();
        if (partModels.find(partNumber) == partModels.end())
          partModels.insert(std::pair <int32_t, PartModel>(partNumber, PartModel(nBins))); //add a new model to end of models list

        try
        {
          auto partModel = partModels.at(partNumber);
          std::vector <cv::Point3i> partPixelColoursVector; // temporary variable
          std::vector <cv::Point3i> bgPixelColoursVector; // temporary variable
          int blankPixelsCount;
          try
          {
            partPixelColoursVector = partPixelColours.at(partNumber); // copy part color set for current bodypart
          }
          catch (...)
          {
            std::stringstream ss;
            ss << "There is no such partPixelColours for body part " << partNumber;
            if (debugLevelParam >= 1)
              std::cerr << ERROR_HEADER << ss.str() << std::endl;
            throw std::logic_error(ss.str());
          }
          try
          {
            blankPixelsCount = blankPixels.at(partNumber);  // copy blanck pixel count for current bodypart
          }
          catch (...)
          {
            std::stringstream ss;
            ss << "There is no such blankPixels for body part " << partNumber;
            if (debugLevelParam >= 1)
              std::cerr << ERROR_HEADER << ss.str() << std::endl;
            throw std::logic_error(ss.str());
          }
          try
          {
            bgPixelColoursVector = bgPixelColours.at(partNumber); // copy background color set for current bodypart
          }
          catch (...)
          {
            std::stringstream ss;
            ss << "There is no such bgPixelColours for body part " << partNumber;
            if (debugLevelParam >= 1)
              std::cerr << ERROR_HEADER << ss.str() << std::endl;
            throw std::logic_error(ss.str());
          }
          addPartHistogram(partModel, partPixelColoursVector, blankPixelsCount); // building histogram for current bodypart colours
          addBackgroundHistogram(partModel, bgPixelColoursVector); // building histograms for current bodypart background colours
          partModels.at(partNumber) = partModel; // copy result to part models set
          if (debugLevelParam >= 2)
            std::cerr << "Found part model: " << partNumber << std::endl;
        }
        catch (...)
        {
          std::stringstream ss;
          ss << "Could not find part model " << partNumber;
          if (debugLevelParam >= 1)
            std::cerr << ERROR_HEADER << ss.str() << std::endl;
          throw std::logic_error(ss.str());
        }

      }
      delete workFrame;
    }
  }

  // Returns a labels vector of possible body parts position
  std::map <uint32_t, std::vector <LimbLabel> > ColorHistDetector::detect(Frame *frame, std::map <std::string, float> params, std::map <uint32_t, std::vector <LimbLabel>> limbLabels)
  {
    params.emplace(COMMON_DETECTOR_PARAMETERS::USE_CH_DETECTOR());

    //now set actual param values
    useCSdet = params.at(COMMON_DETECTOR_PARAMETERS::USE_CH_DETECTOR().first);

    pixelDistributions = buildPixelDistributions(frame); // matrix contains the probability that the particular pixel belongs to current bodypart
    pixelLabels = buildPixelLabels(frame, pixelDistributions); // matrix contains relative estimations that the particular pixel belongs to current bodypart

    auto result = Detector::detect(frame, params, limbLabels);

    for (auto &var : pixelDistributions)
      var.second.release();

    pixelDistributions.clear();

    for (auto &var : pixelLabels)
      var.second.release();

    pixelLabels.clear();

    return result;
  }

  // Return nBins
  uint8_t ColorHistDetector::getNBins(void) const noexcept
  {
    return nBins;
  }

  // Returns relative frequency of the RGB-color reiteration in "PartModel" 
  float ColorHistDetector::computePixelBelongingLikelihood(const PartModel &partModel, uint8_t r, uint8_t g, uint8_t b)
  { // Scaling of colorspace, finding the colors interval, which now gets this color
    auto factor = static_cast<uint8_t> (ceil(pow(2, 8) / partModel.nBins));
    try
    {
      return partModel.partHistogram.at(r / factor).at(g / factor).at(b / factor); // relative frequency of current color reiteration 
    }
    catch (...)
    {
      std::stringstream ss;
      ss << "Couldn't find partHistogram " << "[" << (int)r / factor << "][" << (int)g / factor << "][" << (int)b / factor << "]";
      if (debugLevelParam >= 1)
        std::cerr << ERROR_HEADER << ss.str() << std::endl;
      throw std::logic_error(ss.str());
    }
  }

  // Build into the "partModel" a histogram of the color set "partColors"
  void ColorHistDetector::setPartHistogram(PartModel &partModel, const std::vector <cv::Point3i> &partColors)
  {
    // do not add sample if the number of pixels is zero
    if (partColors.size() == 0)
      return;
    auto factor = static_cast<uint8_t> (ceil(pow(2, 8) / partModel.nBins)); // colorspace scaling coefficient
    partModel.sizeFG = static_cast <uint32_t> (partColors.size());
    partModel.fgNumSamples = 1;
    partModel.fgSampleSizes.clear();
    partModel.fgSampleSizes.push_back(static_cast <uint32_t> (partColors.size()));

    // clear histogram first
    for (auto r = 0; r < partModel.nBins; r++)
    {
      for (auto g = 0; g < partModel.nBins; g++)
      {
        for (auto b = 0; b < partModel.nBins; b++)
        {
          try
          {
            partModel.partHistogram.at(r).at(g).at(b) = 0.0;
          }
          catch (...)
          {
            std::stringstream ss;
            ss << "Couldn't find partHistogram " << "[" << r << "][" << g << "][" << b << "]";
            if (debugLevelParam >= 1)
              std::cerr << ERROR_HEADER << ss.str() << std::endl;
            throw std::logic_error(ss.str());
          }
        }
      }
    }
    // Scaling of colorspace, reducing the capacity and number of colour intervals that are used to construct the histogram
    for (auto i : partColors)
    {
      auto r = static_cast<uint8_t> (i.x / factor);
      auto g = static_cast<uint8_t> (i.y / factor);
      auto b = static_cast<uint8_t> (i.z / factor);
      try
      {
        partModel.partHistogram.at(r).at(g).at(b)++; // increment the frequency of interval, that this color have hit
      }
      catch (...)
      {
        std::stringstream ss;
        ss << "Couldn't find partHistogram " << "[" << r << "][" << g << "][" << b << "]";
        if (debugLevelParam >= 1)
          std::cerr << ERROR_HEADER << ss.str() << std::endl;
        throw std::logic_error(ss.str());
      }
    }
    for (auto r = 0; r < partModel.nBins; r++)
    {
      for (auto g = 0; g < partModel.nBins; g++)
      {
        for (auto b = 0; b < partModel.nBins; b++)
        {
          // normalise the histograms
          try
          {
            partModel.partHistogram.at(r).at(g).at(b) /= partModel.sizeFG;
          }
          catch (...)
          {
            std::stringstream ss;
            ss << "Couldn't find partHistogram " << "[" << r << "][" << g << "][" << b << "]";
            if (debugLevelParam >= 1)
              std::cerr << ERROR_HEADER << ss.str() << std::endl;
            throw std::logic_error(ss.str());
          }
        }
      }
    }
  }

  // Take stock of the additional set of colors in the histogram
  void ColorHistDetector::addPartHistogram(PartModel &partModel, const std::vector <cv::Point3i> &partColors, uint32_t nBlankPixels)
  {
    if (partColors.size() == 0) //do not add sample if the number of pixels is zero
      return;
    //un-normalise
    for (auto r = 0; r < partModel.nBins; r++)
    {
      for (auto g = 0; g < partModel.nBins; g++)
      {
        for (auto b = 0; b < partModel.nBins; b++)
        {
          try
          {
            partModel.partHistogram.at(r).at(g).at(b) *= partModel.sizeFG; // converting the colors relative frequency into the pixels number
          }
          catch (...)
          {
            std::stringstream ss;
            ss << "Couldn't find partHistogram " << "[" << r << "][" << g << "][" << b << "]";
            if (debugLevelParam >= 1)
              std::cerr << ERROR_HEADER << ss.str() << std::endl;
            throw std::logic_error(ss.str());
          }
        }
      }
    }

    auto factor = static_cast<int>(ceil(pow(2, 8) / partModel.nBins)); // colorspace scaling coefficient
    partModel.sizeFG += static_cast <uint32_t> (partColors.size());
    partModel.fgNumSamples++;
    partModel.fgSampleSizes.push_back(static_cast <uint32_t> (partColors.size()));

    // Scaling of colorspace, reducing the capacity and number of colour intervals
    // Adjustment of the histogram
    for (auto color : partColors)
    {
      auto r = static_cast<uint8_t> (color.x / factor);
      auto g = static_cast<uint8_t> (color.y / factor);
      auto b = static_cast<uint8_t> (color.z / factor);
      try
      {
        partModel.partHistogram.at(r).at(g).at(b)++; // increment the frequency of interval, that this color have hit
      }
      catch (...)
      {
        std::stringstream ss;
        ss << "Couldn't find partHistogram " << "[" << r << "][" << g << "][" << b << "]";
        if (debugLevelParam >= 1)
          std::cerr << ERROR_HEADER << ss.str() << std::endl;
        throw std::logic_error(ss.str());
      }
    }

    //renormalise
    for (auto r = 0; r < partModel.nBins; r++)
    {
      for (auto g = 0; g < partModel.nBins; g++)
      {
        for (auto b = 0; b < partModel.nBins; b++)
        {
          //normalise the histograms
          try
          {
            partModel.partHistogram.at(r).at(g).at(b) /= partModel.sizeFG;
          }
          catch (...)
          {
            std::stringstream ss;
            ss << "Couldn't find partHistogram " << "[" << r << "][" << g << "][" << b << "]";
            if (debugLevelParam >= 1)
              std::cerr << ERROR_HEADER << ss.str() << std::endl;
            throw std::logic_error(ss.str());
          }
        }
      }
    }

    partModel.fgBlankSizes.push_back(nBlankPixels); // add the number of blank pixels for this model
  }

  // Totalization the number of used samples
  float ColorHistDetector::getAvgSampleSizeFg(const PartModel &partModel) noexcept
  {
    auto sum = 0.0f;
    for (const auto &i : partModel.fgSampleSizes)
      sum += i;
    sum /= partModel.fgNumSamples;
    return sum;
  }

  // Averaging the number of samples, that united from two sets
  float ColorHistDetector::getAvgSampleSizeFgBetween(const PartModel &partModel, uint32_t s1, uint32_t s2) noexcept
  {
    if (s1 >= partModel.fgSampleSizes.size() || s2 >= partModel.fgSampleSizes.size())
      return 0;
    return (partModel.fgSampleSizes.at(s1) + partModel.fgSampleSizes.at(s2)) / 2.0f;
  }

  //TODO (Vitaliy Koshura): Need unit test
  // Euclidean distance between part histograms
  float ColorHistDetector::matchPartHistogramsED(const PartModel &partModelPrev, const PartModel &partModel)
  {
    auto distance = 0.0f;
    for (auto r = 0; r < partModel.nBins; r++)
    {
      for (auto g = 0; g < partModel.nBins; g++)
      {
        for (auto b = 0; b < partModel.nBins; b++)
        {
          // accumulation of the Euclidean distances between the points
          try
          {
            distance += pow(partModel.partHistogram.at(r).at(g).at(b) - partModelPrev.partHistogram.at(r).at(g).at(b), 2);
          }
          catch (...)
          {
            std::stringstream ss;
            ss << "Couldn't find partHistogram " << "[" << r << "][" << g << "][" << b << "]";
            if (debugLevelParam >= 1)
              std::cerr << ERROR_HEADER << ss.str() << std::endl;
            throw std::logic_error(ss.str());
          }
        }
      }
    }
    return sqrt(distance);
  }

  // Background histogram
  void ColorHistDetector::addBackgroundHistogram(PartModel &partModel, const std::vector <cv::Point3i> &bgColors)
  {
    if (bgColors.size() == 0)
      return;
    // unnormalise
    for (auto r = 0; r < partModel.nBins; r++)
    {
      for (auto g = 0; g < partModel.nBins; g++)
      {
        for (auto b = 0; b < partModel.nBins; b++)
        {
          try
          {
            partModel.bgHistogram.at(r).at(g).at(b) *= partModel.sizeBG;
          }
          catch (...)
          {
            std::stringstream ss;
            ss << "Couldn't find bgHistogram " << "[" << r << "][" << g << "][" << b << "]";
            if (debugLevelParam >= 1)
              std::cerr << ERROR_HEADER << ss.str() << std::endl;
            throw std::logic_error(ss.str());
          }
        }
      }
    }
    auto factor = static_cast<uint32_t>(ceil(pow(2, 8) / partModel.nBins)); // colorspace scaling coefficient
    partModel.sizeBG += static_cast <uint32_t> (bgColors.size());
    partModel.bgNumSamples++;
    partModel.bgSampleSizes.push_back(static_cast <uint32_t> (bgColors.size()));
    for (const auto &color : bgColors)
    {
      try
      {
        partModel.bgHistogram.at(color.x / factor).at(color.y / factor).at(color.z / factor)++; // increment the frequency of interval, that this color have hit
      }
      catch (...)
      {
        std::stringstream ss;
        ss << "Couldn't find bgHistogram " << "[" << color.x / factor << "][" << color.y / factor << "][" << color.z / factor << "]";
        if (debugLevelParam >= 1)
          std::cerr << ERROR_HEADER << ss.str() << std::endl;
        throw std::logic_error(ss.str());
      }
    }
    // renormalise
    for (auto r = 0; r < partModel.nBins; r++)
    {
      for (auto g = 0; g < partModel.nBins; g++)
      {
        for (auto b = 0; b < partModel.nBins; b++)
        {
          try
          {
            partModel.bgHistogram.at(r).at(g).at(b) /= static_cast<float>(partModel.sizeBG);
          }
          catch (...)
          {
            std::stringstream ss;
            ss << "Couldn't find bgHistogram " << "[" << r << "][" << g << "][" << b << "]";
            if (debugLevelParam >= 1)
              std::cerr << ERROR_HEADER << ss.str() << std::endl;
            throw std::logic_error(ss.str());
          }
        }
      }
    }
  }

  // Returns a matrix, that contains relative frequency of the pixels colors reiteration 
  std::map <int32_t, cv::Mat> ColorHistDetector::buildPixelDistributions(Frame *frame)
  {
    auto skeleton = frame->getSkeleton(); // copy skeleton from the frame
    auto partTree = skeleton.getPartTree(); // copy part tree from the skeleton
    auto imgMat = frame->getImage(); // copy image from the frame
    auto maskMat = frame->getMask(); // copy mask from the frame
    auto width = imgMat.cols;
    auto height = imgMat.rows;
    auto mwidth = maskMat.cols;
    auto mheight = maskMat.rows;
    std::map <int32_t, cv::Mat> pixelDistributions;
    if (width != mwidth || height != mheight) // error if mask and image sizes don't match
    {
      std::stringstream ss;
      ss << "Mask size not equal image size";
      if (debugLevelParam >= 1)
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

            t.at<float>(y, x) = blackPixel ? 0 : computePixelBelongingLikelihood(partModel, red, green, blue); // relative frequency of the current pixel color reiteration 
          }
        }
      }
      catch (...)
      {
        std::stringstream ss;
        ss << "Maybe couldn't find partModel " << partID;
        if (debugLevelParam >= 1)
          std::cerr << ERROR_HEADER << ss.str() << std::endl;
        throw std::logic_error(ss.str());
      }
      pixelDistributions.insert(std::pair <int32_t, cv::Mat>(partID, t)); // add the current bodypart matrix to the set 
    }
    return pixelDistributions;
  }

  std::map <int32_t, cv::Mat> ColorHistDetector::buildPixelLabels(Frame *frame, std::map <int32_t, cv::Mat> pixelDistributions)
  {
    auto maskMat = frame->getMask(); // copy mask from the frame
    auto width = maskMat.cols;
    auto height = maskMat.rows;
    auto skeleton = frame->getSkeleton(); // copy skeleton from the frame
    auto partTree = skeleton.getPartTree(); // copy part tree from the skeleton
    std::map <int32_t, cv::Mat> pixelLabels;
    // For all body parts
    for (const auto &bodyPart : partTree)
    {
      auto t = cv::Mat(height, width, cv::DataType <float>::type); // create empty matrix
      cv::Mat tt;
      try
      { // Matrix, that contains relative frequency of the pixels colors reiteration for current body part
        tt = pixelDistributions.at(bodyPart.getPartID());
      }
      catch (...)
      {
        std::stringstream ss;
        ss << "Couldn't find distributions for body part " << bodyPart.getPartID();
        if (debugLevelParam >= 1)
          std::cerr << ERROR_HEADER << ss.str() << std::endl;
        throw std::logic_error(ss.str());
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
                temp = pixelDistributions.at(i.getPartID()); // matrix of the pixels colors frequency for current body part
              }
              catch (...)
              {
                std::stringstream ss;
                ss << "Couldn't find pixel distributions for body part " << i.getPartID();
                if (debugLevelParam >= 1)
                  std::cerr << ERROR_HEADER << ss.str() << std::endl;
                throw std::logic_error(ss.str());
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
                if (debugLevelParam >= 1)
                  std::cerr << ERROR_HEADER << ss.str() << std::endl;
                throw std::logic_error(ss.str());
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
              if (debugLevelParam >= 1)
                std::cerr << ERROR_HEADER << ss.str() << std::endl;
              throw std::logic_error(ss.str());
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
              if (debugLevelParam >= 1)
                std::cerr << ERROR_HEADER << ss.str() << std::endl;
              throw std::logic_error(ss.str());
            }
          }
        }
      }
      pixelLabels.insert(std::pair<int32_t, cv::Mat>(bodyPart.getPartID(), t)); // insert the resulting matrix into the set "pixelLabels" 
    }
    return pixelLabels;
  }

  float ColorHistDetector::compare(void)
  {
    if (comparer_bodyPart == 0 || comparer_frame == 0 || comparer_j0 == 0 || comparer_j1 == 0)
    {
      std::stringstream ss;
      ss << "Compare parameters are invalid: " << (comparer_bodyPart == 0 ? "comparer_bodyPart == 0 " : "") << (comparer_frame == 0 ? "comparer_frame == 0 " : "") << (comparer_j0 == 0 ? "comparer_j0 == 0" : "") << (comparer_j1 == 0 ? "comparer_j1 == 0" : "") << std::endl;
      if (debugLevelParam >= 1)
        std::cerr << ERROR_HEADER << ss.str() << std::endl;
      throw std::logic_error(ss.str());
    }
    try
    {
      return compare(*comparer_bodyPart, *comparer_frame, pixelDistributions, pixelLabels, *comparer_j0, *comparer_j1);
    }
    catch (std::logic_error ex)
    {
      if (debugLevelParam >= 1)
      {
        std::string frameType;
        if ((*comparer_frame)->getFrametype() == KEYFRAME)
          frameType = "Keyframe";
        else if ((*comparer_frame)->getFrametype() == LOCKFRAME)
          frameType = "Lockframe";
        else
          frameType = "Interpolation";
        std::cerr << ERROR_HEADER << "Dirty Label: " << " Frame(" << frameType << "): " << (*comparer_frame)->getID() << " Part: " << comparer_bodyPart->getPartID() << " " << ex.what() << std::endl;
      }
      return -1.0f;
    }
  }

  float ColorHistDetector::compare(BodyPart bodyPart, Frame *frame, std::map <int32_t, cv::Mat> pixelDistributions, std::map <int32_t, cv::Mat> pixelLabels, cv::Point2f j0, cv::Point2f j1)
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
      if (debugLevelParam >= 1)
        std::cerr << ERROR_HEADER << ss.str() << std::endl;
      throw std::logic_error(ss.str());
    }
    if (getAvgSampleSizeFg(model) == 0) // error if samples count is zero
    {
      std::stringstream ss;
      ss << "Couldn't get avgSampleSizeFg";
      if (debugLevelParam >= 2)
        std::cerr << ERROR_HEADER << ss.str() << std::endl;
      throw std::logic_error(ss.str());
    }
    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax); // highlight the extreme points of the body part rect
    cv::Mat bodyPartPixelDistribution;
    try
    {
      bodyPartPixelDistribution = pixelDistributions.at(bodyPart.getPartID());
    }
    catch (...)
    {
      std::stringstream ss;
      ss << "Can't get pixesDistribution [" << bodyPart.getPartID() << "]";
      if (debugLevelParam >= 2)
        std::cerr << ERROR_HEADER << ss.str() << std::endl;
      throw std::logic_error(ss.str());
    }
    cv::Mat bodyPartLixelLabels;
    try
    {
      bodyPartLixelLabels = pixelLabels.at(bodyPart.getPartID());
    }
    catch (...)
    {
      std::stringstream ss;
      ss << "Can't get pixesLabels [" << bodyPart.getPartID() << "]";
      if (debugLevelParam >= 2)
        std::cerr << ERROR_HEADER << ss.str() << std::endl;
      throw std::logic_error(ss.str());
    }
    // Scan the area near the bodypart center

    auto searchXMin = boxCenter.x - boneLength * 0.5f;
    auto searchXMax = boxCenter.x + boneLength * 0.5f;
    auto searchYMin = boxCenter.y - boneLength * 0.5f;
    auto searchYMax = boxCenter.y + boneLength * 0.5f;

    for (auto i = searchXMin; i < searchXMax; i++)
    {
      for (auto j = searchYMin; j < searchYMax; j++)
      {
        if (i < maskMat.cols && j < maskMat.rows && i >= 0 && j >= 0) // if the point is within the image
        {
          if (i <= xmax && i >= xmin && j <= ymax && j >= ymin) // if the point within the highlight area
          {
            if (rect.containsPoint(cv::Point2f(i, j)) > 0) // if the point belongs to the rectangle
            {
              totalPixels++; // counting of the contained pixels
              auto mintensity = 0;
              try
              {
                mintensity = maskMat.at<uint8_t>(static_cast<int32_t>(j), static_cast<int32_t>(i)); // copy current point mask value 
              }
              catch (...)
              {
                std::stringstream ss;
                ss << "Can't get maskMat [" << (int32_t)j << "][" << (int32_t)i << "]";
                if (debugLevelParam >= 2)
                  std::cerr << ERROR_HEADER << ss.str() << std::endl;
                throw std::logic_error(ss.str());
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
                  ss << "Can't get pixesDistribution [" << bodyPart.getPartID() << "][" << (int32_t)j << "][" << (int32_t)i << "]";
                  if (debugLevelParam >= 2)
                    std::cerr << ERROR_HEADER << ss.str() << std::endl;
                  throw std::logic_error(ss.str());
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
                  ss << "Can't get pixesLabels [" << bodyPart.getPartID() << "][" << (int32_t)j << "][" << (int32_t)i << "]";
                  if (debugLevelParam >= 2)
                    std::cerr << ERROR_HEADER << ss.str() << std::endl;
                  throw std::logic_error(ss.str());
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
    if (debugLevelParam >= 2)
      std::cerr << ERROR_HEADER << ss.str() << std::endl;
    throw std::logic_error(ss.str());
  }

  LimbLabel ColorHistDetector::generateLabel(BodyPart bodyPart, Frame *frame, cv::Point2f j0, cv::Point2f j1)
  {
    std::stringstream detectorName;
    detectorName << getID();

    comparer_bodyPart = &bodyPart;
    comparer_frame = &frame;
    comparer_j0 = &j0;
    comparer_j1 = &j1;

    auto label = Detector::generateLabel(bodyPart, j0, j1, detectorName.str(), useCSdet);

    comparer_bodyPart = 0;
    comparer_frame = 0;
    comparer_j0 = 0;
    comparer_j1 = 0;

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

}
