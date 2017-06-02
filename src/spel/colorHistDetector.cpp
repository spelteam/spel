// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com
#include "colorHistDetector.hpp"
#include "keyframe.hpp"
#include "lockframe.hpp"
#include "interpolation.hpp"
#include "spelParameters.hpp"
#include "spelHelper.hpp"

namespace SPEL
{
  ColorHistDetector::PartModel::PartModel(uint8_t _nBins) : nBins(_nBins)
  {
    ColorHistDetector::checknBins(nBins);

    partHistogram.resize(nBins, std::vector<std::vector<float>>(nBins,
      std::vector<float>(nBins, 0.0)));
    bgHistogram.resize(nBins, std::vector<std::vector<float>>(nBins,
      std::vector<float>(nBins, 0.0)));

    sizeFG = 0;
    sizeBG = 0;
    fgNumSamples = 0;
    bgNumSamples = 0;
  }

  ColorHistDetector::PartModel::PartModel(const PartModel & model)
  {
    nBins = model.nBins;
    partHistogram = model.partHistogram;
    bgHistogram = model.bgHistogram;
    sizeFG = model.sizeFG;
    sizeBG = model.sizeBG;
    fgNumSamples = model.fgNumSamples;
    bgNumSamples = model.bgNumSamples;
    fgSampleSizes = model.fgSampleSizes;
    bgSampleSizes = model.bgSampleSizes;
    fgBlankSizes = model.fgBlankSizes;
  }

  ColorHistDetector::PartModel::~PartModel(void)
  {
  }

  ColorHistDetector::PartModel &ColorHistDetector::PartModel::operator=(
    const PartModel &model)
  {
    if (&model == this)
      return *this;

    nBins = model.nBins;
    partHistogram = model.partHistogram;
    bgHistogram = model.bgHistogram;
    sizeFG = model.sizeFG;
    sizeBG = model.sizeBG;
    fgNumSamples = model.fgNumSamples;
    bgNumSamples = model.bgNumSamples;
    fgSampleSizes = model.fgSampleSizes;
    bgSampleSizes = model.bgSampleSizes;
    fgBlankSizes = model.fgBlankSizes;
    return *this;
  }

  uint8_t ColorHistDetector::PartModel::calculateFactor(void) const
  {
    checknBins();

    return static_cast<uint8_t> (ceil(pow(2, 8) / nBins));
  }

  float ColorHistDetector::PartModel::computePixelBelongingLikelihood(
    const uint8_t r, const uint8_t g, const uint8_t b) const
  {
    checknBins();

    // Scaling of colorspace, finding the colors interval, 
    // which now gets this color
    const auto factor = calculateFactor();
    // relative frequency of current color reiteration 
    return partHistogram[r / factor][g / factor][b / factor];
  }

  void ColorHistDetector::PartModel::setPartHistogram(
    const std::vector <cv::Point3i> &partColors)
  {
    checknBins();

    // do not add sample if the number of pixels is zero
    if (partColors.size() == 0)
      return;
    // colorspace scaling coefficient
    const auto factor = calculateFactor();
    sizeFG = static_cast <uint32_t> (partColors.size());
    fgNumSamples = 1;
    fgSampleSizes.clear();
    fgSampleSizes.push_back(static_cast <uint32_t> (partColors.size()));

    // clear histogram first
    if (partHistogram.size() != nBins)
      partHistogram.resize(nBins);
    for (auto &r : partHistogram)
    {
      if (r.size() != nBins)
        r.resize(nBins);
      for (auto &g : r)
      {
        if (g.size() != nBins)
          g.resize(nBins);
        for (auto &b : g)
          b = 0.0f;
      }
    }

    // Scaling of colorspace, reducing the capacity and number of colour 
    // intervals that are used to construct the histogram
    for (const auto &i : partColors)
    {
      const auto r = static_cast<uint8_t> (i.x / factor);
      const auto g = static_cast<uint8_t> (i.y / factor);
      const auto b = static_cast<uint8_t> (i.z / factor);

      if (r >= nBins || g >= nBins || b >= nBins)
      {
        std::stringstream ss;
        ss << "RGB value can't be greater " << nBins - 1 << ": r = " << r
          << " g = " << g << " b = " << b << std::endl;
        DebugMessage(ss.str(), 1);
        throw std::out_of_range(ss.str());
      }
      // increment the frequency of interval, that this color have hit
      ++(partHistogram[r][g][b]);
    }

    const auto fg = static_cast<float>(sizeFG);

    // normalise the histograms
    for (auto &r : partHistogram)
      for (auto &g : r)
        for (auto &b : g)
          b /= fg;
  }

  void ColorHistDetector::PartModel::addPartHistogram(
    const std::vector <cv::Point3i> &partColors, const uint32_t nBlankPixels)
  {
    //do not add sample if the number of pixels is zero
    if (partColors.size() == 0)
      return;

    auto fg = static_cast<float>(sizeFG);

    //un-normalise
    // converting the colors relative frequency into the pixels number
    for (auto &r : partHistogram)
      for (auto &g : r)
        for (auto &b : g)
          b *= fg;

    const auto factor = calculateFactor(); // colorspace scaling coefficient
    sizeFG += static_cast <uint32_t> (partColors.size());
    fg = static_cast<float>(sizeFG);

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
      const auto r = static_cast<uint8_t> (color.x / factor);
      const auto g = static_cast<uint8_t> (color.y / factor);
      const auto b = static_cast<uint8_t> (color.z / factor);

      if (r >= nBins || g >= nBins || b >= nBins)
      {
        std::stringstream ss;
        ss << "RGB value can't be greater " << nBins - 1 << ": r = " << r <<
          " g = " << g << " b = " << b << std::endl;
        DebugMessage(ss.str(), 1);
        throw std::out_of_range(ss.str());
      }
      // increment the frequency of interval, that this color have hit
      ++(partHistogram[r][g][b]);
    }

    //renormalise
    for (auto &r : partHistogram)
      for (auto &g : r)
        for (auto &b : g)
          b /= fg;
    // add the number of blank pixels for this model
    fgBlankSizes.push_back(nBlankPixels);
  }

  float ColorHistDetector::PartModel::getAvgSampleSizeFg(void) const
  {
    if (fgNumSamples == 0)
      return 0.0f;

    auto sum = 0.0f;
    for (const auto &i : fgSampleSizes)
      sum += i;
    sum /= static_cast<float>(fgNumSamples);
    return sum;
  }

  float ColorHistDetector::PartModel::getAvgSampleSizeFgBetween(
    const size_t s1, const size_t s2) const
  {
    if (s1 >= fgSampleSizes.size() || s2 >= fgSampleSizes.size())
    {
      std::stringstream ss;
      ss << "Incorrect parameter. s1: " << s1 << " s2: " << s2 <<
        " Actual size: " << fgSampleSizes.size() << std::endl;
      DebugMessage(ss.str(), 1);
      throw std::invalid_argument(ss.str());
    }
    return (fgSampleSizes[s1] + fgSampleSizes[s2]) / 2.0f;
  }

  float ColorHistDetector::PartModel::matchPartHistogramsED(
    const PartModel &partModelPrev) const
  {
    checknBins();

    if (nBins != partModelPrev.nBins)
    {
      std::stringstream ss;
      ss << "Different nBins value. Expected: " << nBins << " Actual: " <<
        partModelPrev.nBins << std::endl;
      DebugMessage(ss.str(), 1);
      throw std::logic_error(ss.str());
    }

    auto distance = 0.0f;

    for (auto r = 0; r < nBins; r++)
    {
      const auto &rc = partHistogram[r];
      const auto &rp = partModelPrev.partHistogram[r];
      for (auto g = 0; g < nBins; g++)
      {
        const auto &gc = rc[g];
        const auto &gp = rp[g];
        for (auto b = 0; b < nBins; b++)
          // accumulation of the Euclidean distances between the points
          distance += pow(gc[b] - gp[b], 2);
      }
    }
    return sqrt(distance);
  }

  void ColorHistDetector::PartModel::addBackgroundHistogram(
    const std::vector <cv::Point3i> &bgColors)
  {
    if (bgColors.size() == 0)
      return;

    checknBins();

    // unnormalise
    auto bg = static_cast<float>(sizeBG);
    for (auto &r : bgHistogram)
      for (auto &g : r)
        for (auto &b : g)
          b *= bg;

    const auto factor = calculateFactor(); // colorspace scaling coefficient
    sizeBG += static_cast <uint32_t> (bgColors.size());
    bg = static_cast<float>(sizeBG);
    ++bgNumSamples;
    bgSampleSizes.push_back(static_cast <uint32_t> (bgColors.size()));

    if (sizeBG == 0)
    {
      const std::string str = "sizeBG can't be zero";
      DebugMessage(str, 1);
      throw std::logic_error(str);
    }

    for (const auto &color : bgColors)
    {
      const auto r = static_cast<uint8_t> (color.x / factor);
      const auto g = static_cast<uint8_t> (color.y / factor);
      const auto b = static_cast<uint8_t> (color.z / factor);

      if (r >= nBins || g >= nBins || b >= nBins)
      {
        std::stringstream ss;
        ss << "RGB value can't be greater " << nBins - 1 << ": r = " << r <<
          " g = " << g << " b = " << b << std::endl;
        DebugMessage(ss.str(), 1);
        throw std::out_of_range(ss.str());
      }

      // increment the frequency of interval, that this color have hit
      ++(bgHistogram[r][g][b]);
    }
    // renormalise
    for (auto &r : bgHistogram)
      for (auto &g : r)
        for (auto &b : g)
          b /= bg;
  }

  void ColorHistDetector::PartModel::checknBins(void) const
  {
    ColorHistDetector::checknBins(nBins);
  }

  ColorHistDetector::ColorHistDetector(uint8_t _nBins) : nBins(_nBins)
  {
    ColorHistDetector::checknBins(nBins);
    m_id = 0x43484400;
  }

  ColorHistDetector::ColorHistDetector(const ColorHistDetector & detector)
    : nBins(detector.nBins)
  {
    m_frames = detector.getFrames();
  }

  ColorHistDetector::~ColorHistDetector(void)
  {
  }

  void ColorHistDetector::train(const std::vector <Frame*> &frames,
    std::map <std::string, float> params)
  {
    partModels.clear();
    emplaceDefaultParameters(params);

    Detector::train(frames, params, [&](auto frame, const auto) {
      this->train(frame);
    });
  }

  // Returns a labels vector of possible body parts position
  std::map <uint32_t, std::vector <LimbLabel> > ColorHistDetector::detect(
    Frame *frame, std::map <std::string, float> params,
    std::map <uint32_t, std::vector <LimbLabel>> &limbLabels) const
  {
    emplaceDefaultParameters(params);

    auto detectorHelper = new ColorHistDetectorHelper();

    // matrix contains the probability that the particular pixel belongs 
    // to current bodypart
    auto pixelDistributions = buildPixelDistributions(frame);
    // matrix contains relative estimations that the particular 
    // pixel belongs to current bodypart
    detectorHelper->pixelLabels = buildPixelLabels(frame, pixelDistributions);

    for (auto &p : pixelDistributions)
      p.second.release();

    const auto result = Detector::detect(frame, params, limbLabels, detectorHelper);

    delete detectorHelper;
    frame->UnloadAll();
    return result;
  }

  // Return nBins
  uint8_t ColorHistDetector::getNBins(void) const
  {
    return nBins;
  }

  // Returns a matrix, that contains relative frequency of the pixels 
  // colors reiteration 
  std::map <int32_t, cv::Mat> ColorHistDetector::buildPixelDistributions(
    Frame *frame) const
  {
    // copy skeleton from the frame
    const auto skeleton = frame->getSkeleton();
    // copy part tree from the skeleton
    const auto partTree = skeleton.getPartTree();
    // copy image from the frame
    const auto imgMat = frame->getImage();
    // copy mask from the frame
    const auto maskMat = frame->getMask();
    const auto width = imgMat.cols;
    const auto height = imgMat.rows;
    const auto mwidth = maskMat.cols;
    const auto mheight = maskMat.rows;
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
      const auto partID = bodyPart.getPartID();
      // copy part model of current bodybart
      const auto &partModel = partModels.at(partID);
      // For all pixels
      for (auto x = 0; x < width; ++x)
      {
        for (auto y = 0; y < height; ++y)
        {
          const auto &intensity = imgMat.at<cv::Vec3b>(y, x);
          // Copy components of the current pixel color
          const auto blue = intensity.val[0];
          const auto green = intensity.val[1];
          const auto red = intensity.val[2];
          // copy mask of the current pixel
          const auto mintensity = maskMat.at<uint8_t>(y, x);
          // pixel is not significant if the mask value is less 
          // than this threshold
          const auto blackPixel = mintensity < 10;
          // relative frequency of the current pixel color reiteration 
          t.at<float>(y, x) = blackPixel ? 0 :
            partModel.computePixelBelongingLikelihood(red, green, blue);
        }
      }
      // add the current bodypart matrix to the set 
      tempPixelDistributions.insert(std::make_pair(partID, t));
    }
    return tempPixelDistributions;
  }

  std::map <int32_t, cv::Mat> ColorHistDetector::buildPixelLabels(Frame *frame,
    const std::map <int32_t, cv::Mat> &_pixelDistributions) const
  {
    // copy mask from the frame
    const auto maskMat = frame->getMask();
    const auto width = maskMat.cols;
    const auto height = maskMat.rows;
    // copy skeleton from the frame
    const auto skeleton = frame->getSkeleton();
    // copy part tree from the skeleton
    const auto partTree = skeleton.getPartTree();
    std::map <int32_t, cv::Mat> _pixelLabels;
    // For all body parts
    for (const auto &bodyPart : partTree)
    {
      // create empty matrix
      auto t = cv::Mat(height, width, cv::DataType <float>::type);
      // Matrix, that contains relative frequency of the pixels colors 
      // reiteration for current body part
      const auto tt = _pixelDistributions.at(bodyPart.getPartID());
      // For all pixels
      for (auto x = 0; x < width; x++)
      {
        for (auto y = 0; y < height; y++)
        {
          //copy the current pixel mask value
          const auto mintensity = maskMat.at<uint8_t>(y, x);
          // pixel is not significant if the mask value is less 
          // than this threshold
          const auto blackPixel = mintensity < 10;
          if (!blackPixel)
          {
            auto top = 0.0f;
            auto sum = 0.0f;
            // For all body parts
            for (const auto &i : partTree)
            {
              // matrix of the pixels colors frequency for current body part
              const auto temp = _pixelDistributions.at(i.getPartID());
              // search max value of the current bodypart 
              // pixel color frequency
              const auto val = temp.at<float>(y, x);
              if (val > top)
                top = val;
              sum += val;
            }
            t.at<float>(y, x) = (spelHelper::compareFloat(top, 0.0f) == 0) ?
              0.0f : tt.at<float>(y, x) / top;
          }
          else
            t.at<float>(y, x) = 0.0f;
        }
      }
      // insert the resulting matrix into the set "pixelLabels" 
      _pixelLabels.insert(std::make_pair(bodyPart.getPartID(), t));
    }
    return _pixelLabels;
  }

  float ColorHistDetector::compare(Frame *frame, 
    const spelRECT<cv::Point2f> &rect, const cv::Mat &bodyPartPixelLabels) 
    const
  {
    // copy mask from the frame 
    const auto maskMat = frame->getMask();
    auto totalPixels = 0;
    auto pixelsInMask = 0;
    auto totalPixelLabelScore = 0.0f;
    float xmax, ymax, xmin, ymin;
    // highlight the extreme points of the body part rect
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);

    const auto imin = static_cast<int>(xmin);
    const auto imax = static_cast<int>(xmax);
    const auto jmin = static_cast<int>(ymin);
    const auto jmax = static_cast<int>(ymax);

    for (auto i = imin; i <= imax; ++i)
    {
      for (auto j = jmin; j <= jmax; ++j)
      {
        // if the point is within the image
        if (i < maskMat.cols && j < maskMat.rows && i >= 0 && j >= 0)
        {

          // if the point belongs to the rectangle
          if (rect.containsPoint(cv::Point2f(static_cast<float>(i),
            static_cast<float>(j))) > 0)
          {
            // counting of the contained pixels
            ++totalPixels;
            // pixel is not significant if the mask value is less 
            // than this threshold
            if (maskMat.at<uint8_t>(j, i) >= 10)
            {
              totalPixelLabelScore += bodyPartPixelLabels.at<float>(j, i);
              ++pixelsInMask; // counting pixels within the mask
            }
          }
        }
      }
    }
    const auto inMaskSuppWeight = 0.5f;
    if (totalPixelLabelScore > 0.0f && totalPixels > 10)
    {
      const auto supportScore = totalPixelLabelScore /
        static_cast<float>(totalPixels);
      const auto inMaskSupportScore = totalPixelLabelScore /
        static_cast<float>(pixelsInMask);
      return 1.0f - ((1.0f - inMaskSuppWeight) * supportScore +
        inMaskSuppWeight * inMaskSupportScore);
    }
    const std::string str = "Dirty label!";
    DebugMessage(str, 2);
    return -1.0f;
  }

  float ColorHistDetector::compare(const BodyPart &bodyPart,
    Frame *frame, const std::map <int32_t, cv::Mat> &_pixelLabels,
    const cv::Point2f &j0, const cv::Point2f &j1) const
  {
    const auto rect = spelHelper::round(bodyPart.getBodyPartRect(j0, j1));
    const auto bodyPartPixelLabels = _pixelLabels.at(bodyPart.getPartID());
    return compare(frame, rect, bodyPartPixelLabels);
  }

  float ColorHistDetector::compare(Frame * frame, 
    const std::map<int32_t, cv::Mat>& _pixelLabels, const LimbLabel & label) 
    const
  {
    const auto rect = spelHelper::round(label.getRect());
    const auto bodyPartPixelLabels = _pixelLabels.at(label.getLimbID());
    return compare(frame, rect, bodyPartPixelLabels);
  }

  void ColorHistDetector::train(Frame * frame)
  {
    // Create local variables
    // the set of RGB-colours of pixel's for current body part
    std::map <int32_t, std::vector <cv::Point3i>> partPixelColours;
    // the set of RGB-colours for a pixels of background
    std::map <int32_t, std::vector <cv::Point3i>> bgPixelColours;
    // pixels outside the mask
    std::map <int32_t, int> blankPixels;
    // copy marking from current frame
    auto skeleton = frame->getSkeleton();
    // polygons for this frame
    std::multimap <int32_t, spelRECT <cv::Point2f>> polygons;
    // used for evaluation of overlapped polygons
    std::multimap <int32_t, float> polyDepth;
    // the skeleton body parts
    auto partTree = skeleton.getPartTree();
    // Handling all bodyparts on the frames
    for (auto &bodyPart : partTree)
    {
      // container initialization for conserve colours set 
      // for each of body parts
      partPixelColours.insert(std::make_pair(bodyPart.getPartID(),
        std::vector <cv::Point3i>()));
      // container initialization for conserve background colours set 
      // for each of body parts
      bgPixelColours.insert(std::make_pair(bodyPart.getPartID(),
        std::vector <cv::Point3i>()));
      // container initialization for counting blank pixels 
      // for each of body parts
      blankPixels.insert(std::make_pair(bodyPart.getPartID(), 0));

      // the parent node of current body part pointer 
      auto joint = skeleton.getBodyJoint(bodyPart.getParentJoint());
      if (joint == nullptr)
      {
        const std::string str = "Invalid parent joint";
        DebugMessage(str, 1);
        // a joint has no marking on the frame
        throw std::logic_error(str);
      }
      // coordinates of current joint
      const auto j0 = joint->getImageLocation();
      // the child node of current body part pointer
      joint = skeleton.getBodyJoint(bodyPart.getChildJoint());
      if (joint == nullptr)
      {
        const std::string str = "Invalid child joint";
        DebugMessage(str, 1);
        // a joint has no marking on the frame
        throw std::logic_error(str);
      }
      // coordinates of current joint
      const auto j1 = joint->getImageLocation();
      // used as estimation of the vector's direction
      const auto direction = j1 - j0;
      //bodypart tilt angle 
      const auto rotationAngle = spelHelper::getAngle(direction);
      bodyPart.setRotationSearchRange(rotationAngle);
      const auto poserect = bodyPart.getBodyPartRect(j0, j1);
      polygons.insert(std::make_pair(bodyPart.getPartID(), poserect));
      polyDepth.insert(std::make_pair(bodyPart.getPartID(),
        skeleton.getBodyJoint(
          bodyPart.getParentJoint())->getSpaceLocation().z));
    }
    skeleton.setPartTree(partTree);
    frame->setSkeleton(skeleton);
    // copy mask from the current frame
    const auto &maskMat = frame->getMask();
    // copy image from the current frame
    const auto &imgMat = frame->getImage();
    // Range over all pixels of the frame
    for (auto i = 0; i < imgMat.cols; ++i)
    {
      for (auto j = 0; j < imgMat.rows; ++j)
      {
        // copy RGB color of current pixel
        const auto intensity = imgMat.at<cv::Vec3b>(j, i);

        // Copy the current pixel colour components
        const auto blue = intensity.val[0];
        const auto green = intensity.val[1];
        const auto red = intensity.val[2];
        // copy current pixel mask value 
        const auto mintensity = maskMat.at<uint8_t>(j, i);

        const auto blackPixel = mintensity < 10;
        // will be equal to -1 until is not found polygon, 
        // which contains the point
        auto partHit = -1;
        auto depth = 0.0f;
        // Handling all polygons
        for (const auto &bodyPart : partTree)
        {
          const auto partNumber = bodyPart.getPartID();
          auto bContainsPoint = false;
          std::vector <spelRECT <cv::Point2f>> partPolygons;
          // Copy polygons to "PartPolygons"
          transform(polygons.lower_bound(partNumber),
            polygons.upper_bound(partNumber), back_inserter(partPolygons),
            [](auto const &pair) { return pair.second; });
          // Checking whether a pixel belongs to the current and 
          // to another polygons
          const auto fi = static_cast<float>(i);
          const auto fj = static_cast<float>(j);
          for (const auto &partPolygon : partPolygons)
            if ((bContainsPoint = (partPolygon.containsPoint(
              cv::Point2f(fi, fj)) > 0)) == true)
              break; // was found polygon, which contain current pixel

          std::vector <float> partDepths;
          // copy "polyDepth" to "PartDepth"
          transform(polyDepth.lower_bound(partNumber),
            polyDepth.upper_bound(partNumber), back_inserter(partDepths),
            [](auto const &pair) { return pair.second; });
          // Checking polygons overlapping
          for (const auto &partDepth : partDepths)
          {
            if (bContainsPoint && (partHit == -1 || partDepth < depth))
            {
              // store the number of the first found polygon
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
        partModels.insert(std::make_pair(partNumber, PartModel(nBins)));

      auto &partModel = partModels.at(partNumber);
      // building histogram for current bodypart colours
      partModel.addPartHistogram(partPixelColours.at(partNumber),
        blankPixels.at(partNumber));
      // building histograms for current bodypart background colours
      partModel.addBackgroundHistogram(bgPixelColours.at(partNumber));
      if (SpelObject::getDebugLevel() >= 2)
        std::cout << "Found part model: " << partNumber << std::endl;
    }
    frame->AdjustScale();
    frame->UnloadAll();
  }

  LimbLabel ColorHistDetector::generateLabel(const BodyPart &bodyPart,
    Frame *frame, const cv::Point2f &j0,
    const cv::Point2f &j1, DetectorHelper *detectorHelper,
    std::map <std::string, float> params) const
  {
    std::stringstream detectorName;
    detectorName << getID();

    emplaceDefaultParameters(params);

    auto helper = dynamic_cast<ColorHistDetectorHelper*> (detectorHelper);
    if (helper == nullptr)
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

    const auto label = Detector::generateLabel(bodyPart, j0, j1, detectorName.str(),
      params.at(COMMON_DETECTOR_PARAMETERS::USE_CH_DETECTOR().name()),
      comparer);

    return label;
  }

  void ColorHistDetector::calculateLabelScore(Frame * workFrame, 
    DetectorHelper * detectorHelper, LimbLabel & label, 
    std::map <std::string, float> params) const
  {
    emplaceDefaultParameters(params);

    std::stringstream detectorName;
    detectorName << getID();

    auto helper = dynamic_cast<ColorHistDetectorHelper*> (detectorHelper);
    if (helper == nullptr)
    {
      const std::string str =
        "Wrong type: detectorHelper is not ColorHistDetectorHelper";
      DebugMessage(str, 1);
      throw std::invalid_argument(str);
    }

    const auto &comparer = [&]()
    {
      return compare(workFrame, helper->pixelLabels, label);
    };

    Detector::addLabelScore(label, detectorName.str(), 
      params.at(COMMON_DETECTOR_PARAMETERS::USE_CH_DETECTOR().name()), 
      comparer);
  }

  //Used only as prevent a warning for "const uint8_t nBins";
  std::vector <Frame*> ColorHistDetector::getFrames() const
  {
    return m_frames;
  }

  //Used only as prevent a warning for "const uint8_t nBins";
  ColorHistDetector &ColorHistDetector::operator=(
    const ColorHistDetector &c)
  {
    if (&c == this)
      return *this;

    m_frames = c.getFrames();
    return *this;
  }

  void ColorHistDetector::emplaceDefaultParameters(std::map<std::string, float>& params) const
  {
    Detector::emplaceDefaultParameters(params);
    spelHelper::mergeParameters(params, COMMON_CH_DETECTOR_PARAMETERS::getParameters());
  }

  void ColorHistDetector::checknBins(const uint8_t n)
  {
    if (n == 0)
    {
      const std::string str = "nBins can't be zero";
      DebugMessage(str, 1);
      throw std::invalid_argument(str);
    }
  }

  void ColorHistDetector::checknBins(void) const
  {
    ColorHistDetector::checknBins(nBins);
  }

  ColorHistDetectorHelper::ColorHistDetectorHelper()
  {
  }

  ColorHistDetectorHelper::~ColorHistDetectorHelper(void)
  {
    for (auto &p : pixelLabels)
      p.second.release();
  }
}
