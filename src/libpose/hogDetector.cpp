#include "hogDetector.hpp"

#define ERROR_HEADER __FILE__ << ":" << __LINE__ << ": "

HogDetector::HogDetector(void)
{
  id = 0x4844;
}

int HogDetector::getID(void)
{
  return id;
}

void HogDetector::setID(int _id)
{
  id = _id;
}

HogDetector::PartModel HogDetector::computeDescriptors(BodyPart bodyPart, Point2f j0, Point2f j1, Mat imgMat, int nbins, Size wndSize, Size blockSize, Size blockStride, Size cellSize, double wndSigma, double thresholdL2hys, bool gammaCorrection, int nlevels, int derivAperture, int histogramNormType)
{
  float boneLength = getBoneLength(j0, j1);
  if (boneLength < blockSize.width)
  {
    boneLength = static_cast <float> (blockSize.width);
  }
  else
  {
    boneLength = boneLength + blockSize.width - ((int)boneLength % blockSize.width);
  }
  float boneWidth = getBoneWidth(boneLength, bodyPart);
  if (boneWidth < blockSize.height)
  {
    boneWidth = static_cast <float> (blockSize.height);
  }
  else
  {
    boneWidth = boneWidth + blockSize.width - ((int)boneWidth % blockSize.height);
  }
  Size originalSize = Size(static_cast <uint32_t> (boneLength), static_cast <uint32_t> (boneWidth));
  POSERECT <Point2f> rect = getBodyPartRect(bodyPart, j0, j1, blockSize);

  float xmax, ymax, xmin, ymin;
  rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
  Point2f direction = j1 - j0;
  float rotationAngle = float(PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI));
  PartModel partModel;
  partModel.partModelRect = rect;
  Mat partImage = rotateImageToDefault(imgMat, partModel.partModelRect, rotationAngle, originalSize);
  Mat partImageResized = Mat(wndSize.height, wndSize.width, CV_8UC3, Scalar(255, 255, 255));
  resize(partImage, partImageResized, wndSize);
  partModel.partImage = partImageResized.clone();
  HOGDescriptor detector(wndSize, blockSize, blockStride, cellSize, nbins, derivAperture, wndSigma, histogramNormType, thresholdL2hys, gammaCorrection, nlevels);

  vector <float> descriptors;

  detector.compute(partImageResized, descriptors);

#ifdef DEBUG
      partModel.descriptors = descriptors;
#endif  // DEBUG

  vector <vector <uint32_t>> counter;

  uint32_t i, j, b;

  try
  {
    for (i = 0; i < wndSize.height; i += cellSize.height)
    {
      partModel.gradientStrengths.push_back(vector <vector <float>>());
      counter.push_back(vector <uint32_t>());
      for (j = 0; j < wndSize.width; j += cellSize.width)
      {
        partModel.gradientStrengths.at(i / cellSize.height).push_back(vector <float>());
        counter.at(i / cellSize.height).push_back(0);
        for (b = 0; b < nbins; b++)
        {
          partModel.gradientStrengths.at(i / cellSize.height).at(j / cellSize.width).push_back(0.0f);
        }
      }
    }
  }
  catch (...)
  {
    stringstream ss;
    ss << "Can't get gradientStrengths at [" << i / cellSize.height << "][" << j / cellSize.width << "]";
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif // DEBUG
    throw logic_error(ss.str());
  }

  uint32_t d = 0, n, k, r, c;
  try
  {
    // window rows
    for (n = 0; n + blockStride.height < wndSize.height; n += blockStride.height)
    {
      // window cols
      for (k = 0; k + blockStride.width < wndSize.width; k += blockStride.width)
      {
        // block rows
        for (r = n; r < n + blockSize.height; r += cellSize.height)
        {
          // block cols
          for (c = k; c < k + blockSize.width; c += cellSize.width)
          {
            // nbins
            for (b = 0; b < nbins; b++)
            {
              partModel.gradientStrengths.at(r / cellSize.height).at(c / cellSize.width).at(b) += descriptors.at(d);
              if (b == 0)
              {
                counter.at(r / cellSize.height).at(c / cellSize.width)++;
              }
              d++;
            }
          }
        }
      }
    }
  }
  catch (...)
  {
    stringstream ss;
    ss << "Descriptor parse error:" << endl << "Window row:\t" << n << "\tWindow col:\t" << k << endl << "Block row:\t" << r << "\tBlock col:\t" << c << endl << "NBins:\t" << b << endl;
    ss << "Total image rows:\t" << wndSize.height << "\tTotal image cols:\t" << wndSize.width << endl;
    ss << "Total descriptors:\t" << descriptors.size() << endl;   
    ss << "Trying to get descriptor at:\t" << d << endl;
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif // DEBUG
    throw logic_error(ss.str());
  }

  try
  {
    for (uint32_t i = 0; i < wndSize.height; i += cellSize.height)
    {
      for (uint32_t j = 0; j < wndSize.width; j += cellSize.width)
      {
        for (uint8_t b = 0; b < nbins; b++)
        {
          if (counter.at(i / cellSize.height).at(j / cellSize.width) == 0)
          {
            partModel.gradientStrengths.at(i / cellSize.height).at(j / cellSize.width).at(b) = 0;
          }
          else
          {
            partModel.gradientStrengths.at(i / cellSize.height).at(j / cellSize.width).at(b) /= counter.at(i / cellSize.height).at(j / cellSize.width);
          }
        }
      }
    }
  }
  catch (...)
  {
    stringstream ss;
    ss << "Can't get gradientStrengths at [" << i / cellSize.height << "][" << j / cellSize.width << "]";
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif // DEBUG
    throw logic_error(ss.str());
  }

  return partModel;
}

map <uint32_t, HogDetector::PartModel> HogDetector::computeDescriptors(Frame *frame, int nbins, Size blockSize, Size blockStride, Size cellSize, double wndSigma, double thresholdL2hys, bool gammaCorrection, int nlevels, int derivAperture, int histogramNormType)
{
  map <uint32_t, PartModel> parts;
  Size wndSize;
  Skeleton skeleton = frame->getSkeleton();
  tree <BodyPart> partTree = skeleton.getPartTree();
  Mat imgMat = frame->getImage();
  for (tree <BodyPart>::iterator part = partTree.begin(); part != partTree.end(); ++part)
  {
    try
    {
      wndSize = partSize.at(part->getPartID());
    }
    catch (...)
    {
      stringstream ss;
      ss << "Couldn't get partSize for part " << part->getPartID();
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    Point2f j0, j1;
    BodyJoint *joint = skeleton.getBodyJoint(part->getParentJoint());
    if (joint == 0)
    {
      stringstream ss;
      ss << "Invalid parent joint";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    j0 = joint->getImageLocation();
    joint = 0;
    joint = skeleton.getBodyJoint(part->getChildJoint());
    if (joint == 0)
    {
      stringstream ss;
      ss << "Invalid child joint";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    j1 = joint->getImageLocation();
    Point2f direction = j1 - j0; // used as estimation of the vector's direction
    float rotationAngle = float(PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI)); //bodypart tilt angle 
    part->setRotationSearchRange(rotationAngle);
    try
    {
      parts.insert(pair <uint32_t, PartModel>(part->getPartID(), computeDescriptors(*part, j0, j1, imgMat, nbins, wndSize, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels, derivAperture, histogramNormType)));
    }
    catch (logic_error err)
    {
      stringstream ss;
      ss << "Can't compute descriptors for the frame " << frame->getID() << " for the part " << part->getPartID() << endl;
      ss << "\t" << err.what();
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
  }
  skeleton.setPartTree(partTree);
  frame->setSkeleton(skeleton);
  return parts;
}

map <uint32_t, Size> HogDetector::getMaxBodyPartHeightWidth(vector <Frame*> frames, Size blockSize)
{
  map <uint32_t, Size> result;
  for (vector <Frame*>::iterator frame = frames.begin(); frame != frames.end(); ++frame)
  {
    if ((*frame)->getFrametype() != KEYFRAME && (*frame)->getFrametype() != LOCKFRAME)
    {
      continue;
    }
    Skeleton skeleton = (*frame)->getSkeleton();
    tree <BodyPart> bodyParts = skeleton.getPartTree();
    for (tree <BodyPart>::iterator bodyPart = bodyParts.begin(); bodyPart != bodyParts.end(); ++bodyPart)
    {
      Point2f j0, j1;
      BodyJoint *joint = skeleton.getBodyJoint(bodyPart->getParentJoint());
      if (joint == 0)
      {
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << "Invalid parent joint" << endl;
        break;
      }
      j0 = joint->getImageLocation();
      joint = 0;
      joint = skeleton.getBodyJoint(bodyPart->getChildJoint());
      if (joint == 0)
      {
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << "Invalid child joint" << endl;
        break;
      }
      j1 = joint->getImageLocation();
      float boneLength = getBoneLength(j0, j1);
      //TODO (Vitaliy Koshura): Check this!
      float boneWidth = 0;
      boneWidth = getBoneWidth(boneLength, *bodyPart);

      Size maxSize = Size(static_cast <uint32_t> (boneLength), static_cast <uint32_t> (boneWidth));
      if (result.size() > 0)
      {
        try
        {
          maxSize = result.at(bodyPart->getPartID());
        }
        catch (...){}
      }
      result[bodyPart->getPartID()] = Size(max(maxSize.width, static_cast <int> (boneLength)), max(maxSize.height, static_cast <int> (boneWidth)));
    }
  }
  // normalize
  for (map <uint32_t, Size>::iterator part = result.begin(); part != result.end(); ++part)
  {
    part->second.width += (blockSize.width - part->second.width % blockSize.width);
    part->second.height += (blockSize.height - part->second.height % blockSize.height);
  }
  return result;
}

void HogDetector::train(vector <Frame*> _frames, map <string, float> params)
{
  frames = _frames;

#ifdef DEBUG
  const uint8_t debugLevel = 5;
#else
  const uint8_t debugLevel = 1;
#endif // DEBUG
  const string sDebugLevel = "debugLevel";

  params.emplace(sDebugLevel, debugLevel);

  debugLevelParam = static_cast <uint8_t> (params.at(sDebugLevel));

  //TODO(Vitaliy Koshura): Make some of them as detector params
  Size blockSize = Size(16, 16);
  Size blockStride = Size(8, 8);
  Size cellSize = Size(8, 8);
  Size wndSize = Size(64, 128);
  const uint8_t nbins = 9;
  double wndSigma = -1;
  double thresholdL2hys = 0.2;
  bool gammaCorrection = true;
  int nlevels = 64;
  Size wndStride = Size(8, 8);
  Size padding = Size(32, 32);
  int derivAperture = 1;
  int histogramNormType = HOGDescriptor::L2Hys;

  savedCellSize = cellSize;
  savednbins = nbins;

  partSize = getMaxBodyPartHeightWidth(_frames, blockSize);

  for (vector <Frame*>::iterator frameNum = frames.begin(); frameNum != frames.end(); ++frameNum)
  {
    if ((*frameNum)->getFrametype() != KEYFRAME && (*frameNum)->getFrametype() != LOCKFRAME)
    {
      continue;
    }

    if (debugLevelParam >= 2)
      cerr << "Training on frame " << (*frameNum)->getID() << endl;

    try
    {
      partModels.insert(pair <uint32_t, map <uint32_t, PartModel>>((*frameNum)->getID(), computeDescriptors(*frameNum, nbins, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels, derivAperture, histogramNormType)));
    }
    catch (...)
    {
      break;
    }
  }
}

vector <vector <LimbLabel> > HogDetector::detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels)
{
  const float searchDistCoeff = 0.5;
  const string sSearchDistCoeff = "searchDistCoeff";

  const float minTheta = 90;
  const string sMinTheta = "minTheta";

  const float maxTheta = 100;
  const string sMaxTheta = "maxTheta";

  const float stepTheta = 10;
  const string sStepTheta = "stepTheta";

  const uint32_t uniqueLocationCandidates = 4;
  const string sUniqueLocationCandidates = "uniqueLocationCandidates";

  //const float scaleParam = 1;
  //const string sScaleParam = "scaleParam";

  const float searchDistCoeffMult = 1.25;
  const string sSearchDistCoeffMult = "searchDistCoeffMult";

  const float useHoGdet = 1.0f;
  const string sUseHoGdet = "useHoGdet";

#ifdef DEBUG
  const uint8_t debugLevel = 5;
#else
  const uint8_t debugLevel = 1;
#endif // DEBUG
  const string sDebugLevel = "debugLevel";

  const float rotationThreshold = 0.025f;
  const string sRotationThreshold = "rotationThreshold";

  const float isWeakTreshhold = 0.1f;
  const string sIsWeakTreshhold = "isWeakTreshhold";

  // first we need to check all used params
  params.emplace(sSearchDistCoeff, searchDistCoeff);
  params.emplace(sMinTheta, minTheta);
  params.emplace(sMaxTheta, maxTheta);
  params.emplace(sStepTheta, stepTheta);
  params.emplace(sUniqueLocationCandidates, uniqueLocationCandidates);
  //params.emplace(sScaleParam, scaleParam);
  params.emplace(sSearchDistCoeffMult, searchDistCoeffMult);
  params.emplace(sUseHoGdet, useHoGdet);
  params.emplace(sDebugLevel, debugLevel);
  params.emplace(sRotationThreshold, rotationThreshold);
  params.emplace(sIsWeakTreshhold, isWeakTreshhold);

  debugLevelParam = static_cast <uint8_t> (params.at(sDebugLevel));

  //TODO(Vitaliy Koshura): Make some of them as detector params
  Size blockSize = Size(16, 16);
  Size blockStride = Size(8, 8);
  Size cellSize = Size(8, 8);
  Size wndSize = Size(64, 128);
  const uint8_t nbins = 9;
  double wndSigma = -1;
  double thresholdL2hys = 0.2;
  bool gammaCorrection = true;
  int nlevels = 64;
  Size wndStride = Size(8, 8);
  Size padding = Size(32, 32);
  int derivAperture = 1;
  int histogramNormType = HOGDescriptor::L2Hys;

  stringstream detectorName;
  detectorName << getID();

  vector <vector <LimbLabel> > t;

  Skeleton skeleton = frame->getSkeleton();
  tree <BodyPart> partTree = skeleton.getPartTree();
  tree <BodyPart>::iterator iteratorBodyPart;

  Mat maskMat = frame->getMask();

  for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
  {
    vector <LimbLabel> labels;
    vector <Point2f> uniqueLocations;
    vector <LimbLabel> sortedLabels;
    vector <vector <LimbLabel>> allLabels;
    Point2f j0, j1;

    try
    {
      j0 = skeleton.getBodyJoint(iteratorBodyPart->getParentJoint())->getImageLocation();
      j1 = skeleton.getBodyJoint(iteratorBodyPart->getChildJoint())->getImageLocation();
    }
    catch (...)
    {
      stringstream ss;
      ss << "Can't get joints";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }

    float boneLength = getBoneLength(j0, j1);
    float boxWidth = getBoneWidth(boneLength, *iteratorBodyPart);
    Point2f direction = j1 - j0;
    float theta = float(PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI));
    float minDist = boxWidth * 0.2f;
    if (minDist < 2) minDist = 2;
    float searchDistance = iteratorBodyPart->getSearchRadius();
    try
    {
      if (searchDistance <= 0)
        searchDistance = boneLength * params.at(sSearchDistCoeff);
    }
    catch (...)
    {
      stringstream ss;
      ss << "Maybe there is no '" << sSearchDistCoeff << "' param";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    float minTheta = 0, maxTheta = 0, stepTheta = 0;
    try
    {
      minTheta = params.at(sMinTheta);
    }
    catch (...)
    {
      stringstream ss;
      ss << "Maybe there is no '" << sMinTheta << "' param";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    try
    {
      maxTheta = params.at(sMaxTheta);
    }
    catch (...)
    {
      stringstream ss;
      ss << "Maybe there is no '" << sMaxTheta << "' param";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    try
    {
      stepTheta = params.at(sStepTheta);
    }
    catch (...)
    {
      stringstream ss;
      ss << "Maybe there is no '" << sStepTheta << "' param";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    Point2f suggestStart = 0.5 * j1 + 0.5 * j0;
    for (float x = suggestStart.x - searchDistance * 0.5f; x < suggestStart.x + searchDistance * 0.5f; x += minDist)
    {
      for (float y = suggestStart.y - searchDistance * 0.5f; y < suggestStart.y + searchDistance * 0.5f; y += minDist)
      {
        if (x < maskMat.cols && y < maskMat.rows)
        {
          uint8_t mintensity = 0;
          try
          {
            mintensity = maskMat.at<uint8_t>((int)y, (int)x);
          }
          catch (...)
          {
            stringstream ss;
            ss << "Can't get value in maskMat at " << "[" << y << "][" << x << "]";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
          bool blackPixel = mintensity < 10;
          if (!blackPixel)
          {
            float deltaTheta = abs(iteratorBodyPart->getRotationSearchRange()) + abs(rotationThreshold);
            for (float rot = theta - deltaTheta; rot < theta + deltaTheta; rot += stepTheta)
            {
              Point2f p0 = Point2f(0, 0);
              Point2f p1 = Point2f(1.0, 0);
              p1 *= boneLength;
              p1 = PoseHelper::rotatePoint2D(p1, p0, rot);
              Point2f mid = 0.5 * p1;
              p1 = p1 + Point2f(x, y) - mid;
              p0 = Point2f(x, y) - mid;
              Size size;
              try
              {
                size = partSize.at(iteratorBodyPart->getPartID());
              }
              catch (...)
              {
                stringstream ss;
                ss << "Can't get partSize for body part " << iteratorBodyPart->getPartID();
                if (debugLevelParam >= 1)
                {
                  cerr << ERROR_HEADER << ss.str() << endl;
                  throw logic_error(ss.str());
                }
              }
              LimbLabel generatedLabel = generateLabel(frame, *iteratorBodyPart, p0, p1, computeDescriptors(*iteratorBodyPart, p0, p1, frame->getImage(), nbins, size, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels, derivAperture, histogramNormType), useHoGdet, nbins);
              sortedLabels.push_back(generatedLabel);
            }
          }
        }
      }
    }
    if (sortedLabels.size() == 0)
    {
      for (float rot = theta - minTheta; rot < theta + maxTheta; rot += stepTheta)
      {
        Point2f p0 = Point2f(0, 0);
        Point2f p1 = Point2f(1.0, 0);
        p1 *= boneLength;
        p1 = PoseHelper::rotatePoint2D(p1, p0, rot);
        Point2f mid = 0.5 * p1;
        p1 = p1 + Point2f(suggestStart.x, suggestStart.y) - mid;
        p0 = Point2f(suggestStart.x, suggestStart.y) - mid;
        Size size;
        try
        {
          size = partSize.at(iteratorBodyPart->getPartID());
        }
        catch (...)
        {
          stringstream ss;
          ss << "Can't get partSize for body part " << iteratorBodyPart->getPartID();
          if (debugLevelParam >= 1)
          {
            cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
        }
        LimbLabel generatedLabel = generateLabel(frame, *iteratorBodyPart, p0, p1, computeDescriptors(*iteratorBodyPart, p0, p1, frame->getImage(), nbins, size, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels, derivAperture, histogramNormType), useHoGdet, nbins);
        sortedLabels.push_back(generatedLabel);
      }
    }
    float uniqueLocationCandidates = 0;
    try
    {
      uniqueLocationCandidates = params.at(sUniqueLocationCandidates);
    }
    catch (...)
    {
      stringstream ss;
      ss << "Maybe there is no '" << sUniqueLocationCandidates << "' param";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    if (sortedLabels.size() > 0)
    {
      sort(sortedLabels.begin(), sortedLabels.end());
      Mat locations(frame->getImage().cols, frame->getImage().rows, DataType<uint32_t>::type);
      for (int32_t i = 0; i < frame->getImage().cols; i++)
      {
        for (int32_t j = 0; j < frame->getImage().rows; j++)
        {
          try
          {
            locations.at<uint32_t>(i, j) = 0;
          }
          catch (...)
          {
            stringstream ss;
            ss << "There is no value of locations at " << "[" << i << "][" << j << "]";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
        }
      }
      for (uint32_t i = 0; i < sortedLabels.size(); i++)
      {
        uint32_t x = (uint32_t)sortedLabels.at(i).getCenter().x;
        uint32_t y = (uint32_t)sortedLabels.at(i).getCenter().y;
        try
        {
          if (locations.at<uint32_t>(x, y) < uniqueLocationCandidates)
          {
            try
            {
              labels.push_back(sortedLabels.at(i));
            }
            catch (...)
            {
              stringstream ss;
              ss << "Maybe there is no value of sortedLabels at " << "[" << i << "]";
              if (debugLevelParam >= 1)
                cerr << ERROR_HEADER << ss.str() << endl;
              throw logic_error(ss.str());
            }
            locations.at<uint32_t>(x, y) += 1;
          }
        }
        catch (...)
        {
          stringstream ss;
          ss << "Maybe there is no value of locations at " << "[" << x << "][" << y << "]";
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << ss.str() << endl;
          throw logic_error(ss.str());
        }
      }
      locations.release();
    }
    PoseHelper::RecalculateScoreIsWeak(labels, detectorName.str(), isWeakTreshhold);
    t.push_back(labels);
  }
  return merge(limbLabels, t);
}

LimbLabel HogDetector::generateLabel(Frame *frame, BodyPart bodyPart, Point2f j0, Point2f j1, PartModel descriptors, float _useHoGdet, uint8_t nbins)
{
  labelModels[frame->getID()][bodyPart.getPartID()].push_back(descriptors);
  vector <Score> s;
  Point2f boxCenter = j0 * 0.5 + j1 * 0.5;
  float rot = float(PoseHelper::angle2D(1, 0, j1.x - j0.x, j1.y - j0.y) * (180.0 / M_PI));
  POSERECT <Point2f> rect = getBodyPartRect(bodyPart, j0, j1);
  stringstream detectorName;
  detectorName << getID();

  uint32_t totalPixels = 0;
  uint32_t inMaskPixels = 0;
  float boneLength = getBoneLength(j0, j1); // distance between joints
  Mat imgMat = frame->getImage(); // copy image from the frame
  Mat maskMat = frame->getMask(); // copy mask from the frame 
  float xmax, ymax, xmin, ymin;
  rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax); // highlight the extreme points of the body part rect

  // Scan the area near the bodypart center
  for (int32_t i = int32_t(boxCenter.x - boneLength * 0.5); i < int32_t(boxCenter.x + boneLength * 0.5); i++)
  {
    for (int32_t j = int32_t(boxCenter.y - boneLength * 0.5); j < int32_t(boxCenter.y + boneLength * 0.5); j++)
    {
      if (i < maskMat.cols && j < maskMat.rows) // if the point is within the image
      {
        if (i <= xmax && i >= xmin && j <= ymax && j >= ymin) // if the point within the highlight area
        {
          if (rect.containsPoint(Point2f((float)i, (float)j)) > 0) // if the point belongs to the rectangle
          {
            totalPixels++; // counting of the contained pixels
            uint8_t mintensity = 0;
            try
            {
              mintensity = maskMat.at<uint8_t>(j, i); // copy current point mask value 
            }
            catch (...)
            {
              maskMat.release();
              imgMat.release();
              if (debugLevelParam >= 2)
                cerr << ERROR_HEADER << "Dirty label!" << endl;
              Score sc(-1.0f, detectorName.str(), _useHoGdet);
              s.push_back(sc);
              return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s, true); // create the limb label
            }
            bool blackPixel = mintensity < 10; // pixel is not significant if the mask value is less than this threshold
            if (!blackPixel)
            {
              inMaskPixels++; // counting of the all scanned pixels
            }
          }
        }
      }
    }
  }
  maskMat.release();
  imgMat.release();
  float score = compare(bodyPart, descriptors, nbins);
  score *= ((float)inMaskPixels / (float)totalPixels);
  Score sc(score, detectorName.str(), _useHoGdet);
  s.push_back(sc);
  return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s);
}

float HogDetector::compare(BodyPart bodyPart, PartModel model, uint8_t nbins)
{
  float score = 0;
  uint32_t totalcount = 0;
  for (map <uint32_t, map <uint32_t, PartModel>>::iterator framePartModels = partModels.begin(); framePartModels != partModels.end(); ++framePartModels)
  {
    for (map <uint32_t, PartModel>::iterator partModel = framePartModels->second.begin(); partModel != framePartModels->second.end(); ++partModel)
    {
      if (partModel->first != static_cast <uint32_t> (bodyPart.getPartID()))
      {
        continue;
      }
      else
      {
        if (model.gradientStrengths.size() != partModel->second.gradientStrengths.size())
        {
          stringstream ss;
          ss << "Invalid descriptor count. Need: " << model.gradientStrengths.size() << ". Have: " << partModel->second.gradientStrengths.size();
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << ss.str() << endl;
          throw logic_error(ss.str());
        }
        uint32_t count = 0;
        for (uint32_t i = 0; i < model.gradientStrengths.size(); i++)
        {
          if (model.gradientStrengths.at(i).size() != partModel->second.gradientStrengths.at(i).size())
          {
            stringstream ss;
            ss << "Invalid descriptor count. Need: " << model.gradientStrengths.at(i).size() << ". Have: " << partModel->second.gradientStrengths.at(i).size();
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
          for (uint32_t j = 0; j < model.gradientStrengths.at(i).size(); j++)
          {
            for (uint8_t b = 0; b < nbins; b++)
            {
              try
              {
                count++;
                score += abs(model.gradientStrengths.at(i).at(j).at(b) - partModel->second.gradientStrengths.at(i).at(j).at(b));
                //score += sqrt(pow(model.gradientStrengths.at(i).at(j).at(b), 2) - pow(partModel->second.gradientStrengths.at(i).at(j).at(b), 2));
              }
              catch (...)
              {
                stringstream ss;
                ss << "Can't get some descriptor at [" << i << "][" << j << "][" << b << "]";
                if (debugLevelParam >= 1)
                  cerr << ERROR_HEADER << ss.str() << endl;
                throw logic_error(ss.str());
              }
            }
          }
        }
        totalcount += count;
        break;
      }
    }
  }
  return score /= totalcount;
}

map <uint32_t, map <uint32_t, vector <HogDetector::PartModel>>> HogDetector::getLabelModels(void)
{
  return labelModels;
}

map <uint32_t, map <uint32_t, HogDetector::PartModel>> HogDetector::getPartModels(void)
{
  return partModels;
}

Size HogDetector::getCellSize(void)
{
  return savedCellSize;
}

uint8_t HogDetector::getnbins(void)
{
  return savednbins;
}
