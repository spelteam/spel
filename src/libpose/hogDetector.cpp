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
  HOGDescriptor detector(wndSize, blockSize, blockStride, cellSize, nbins, derivAperture, wndSigma, histogramNormType, thresholdL2hys, gammaCorrection, nlevels);
  detector.compute(partImageResized, partModel.descriptors);  
  return partModel;
}

map <uint32_t, HogDetector::PartModel> HogDetector::computeDescriptors(Frame *frame, int nbins, Size blockSize, Size blockStride, Size cellSize, double wndSigma, double thresholdL2hys, bool gammaCorrection, int nlevels, int derivAperture, int histogramNormType)
{
  map <uint32_t, PartModel> parts;
  Size wndSize;
  tree <BodyPart> partTree = frame->getSkeleton().getPartTree();
  Mat imgMat = frame->getImage();
  Skeleton skeleton = frame->getSkeleton();
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
    part->second.height += (blockSize.height -  part->second.height % blockSize.height);
  }
  return result;
}

void HogDetector::train(vector <Frame*> _frames, map <string, float> params)
{
  frames = _frames;

  const uint8_t debugLevel = 1;
  const string sDebugLevel = "debugLevel";

  params.emplace(sDebugLevel, debugLevel);

  debugLevelParam = static_cast <uint8_t> (params.at(sDebugLevel));

//TODO(Vitaliy Koshura): Make some of them as detector params
  Size blockSize = Size(16, 16);
  Size blockStride = Size(8,8);
  Size cellSize = Size(8,8);
  Size wndSize = Size(64, 128);
  const int nbins = 9;
  double wndSigma = -1;
  double thresholdL2hys = 0.2;
  bool gammaCorrection = true;
  int nlevels = 64;
  Size wndStride = Size(8, 8);
  Size padding = Size(32, 32);
  int derivAperture = 1;
  int histogramNormType = HOGDescriptor::L2Hys;

  partSize = getMaxBodyPartHeightWidth(_frames, blockSize);

  for (vector <Frame*>::iterator frameNum = frames.begin(); frameNum != frames.end(); ++frameNum)
  {
    if ((*frameNum)->getFrametype() != KEYFRAME && (*frameNum)->getFrametype() != LOCKFRAME)
    {
      continue;
    }

    if (debugLevelParam >= 2)
      cerr << "Training on frame " << (*frameNum)->getID() << endl;
    vector <Rect> found;
    vector <Rect> found_filtered;

    try
    {
      partModels.insert(pair <uint32_t, map <uint32_t, PartModel>> ((*frameNum)->getID(), computeDescriptors(*frameNum, nbins, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels, derivAperture, histogramNormType)));
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

  const uint8_t debugLevel = 1;
  const string sDebugLevel = "debugLevel";

  params.emplace(sDebugLevel, debugLevel);

  // first we need to check all used params
  params.emplace(sSearchDistCoeff, searchDistCoeff);
  params.emplace(sMinTheta, minTheta);
  params.emplace(sMaxTheta, maxTheta);
  params.emplace(sStepTheta, stepTheta);
  params.emplace(sUniqueLocationCandidates, uniqueLocationCandidates);
  //params.emplace(sScaleParam, scaleParam);
  params.emplace(sSearchDistCoeffMult, searchDistCoeffMult);
  params.emplace(sUseHoGdet, useHoGdet);

  debugLevelParam = static_cast <uint8_t> (params.at(sDebugLevel));

//TODO(Vitaliy Koshura): Make some of them as detector params
  Size blockSize = Size(16, 16);
  Size blockStride = Size(8,8);
  Size cellSize = Size(8,8);
  Size wndSize = Size(64, 128);
  const int nbins = 9;
  double wndSigma = -1;
  double thresholdL2hys = 0.2;
  bool gammaCorrection = true;
  int nlevels = 64;
  Size wndStride = Size(8, 8);
  Size padding = Size(32, 32);
  int derivAperture = 1;
  int histogramNormType = HOGDescriptor::L2Hys;

  vector <Rect> found;
  vector <Rect> found_filtered;

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
    float searchDistance = 0;
    try
    {
      float mult = partTree.depth(iteratorBodyPart) * params.at(sSearchDistCoeffMult);
      if (mult == 0) mult = 1;
      searchDistance = boneLength * params.at(sSearchDistCoeff) * mult;
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
            for (float rot = theta - minTheta; rot < theta + maxTheta; rot += stepTheta)
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
              LimbLabel generatedLabel = generateLabel(*iteratorBodyPart, p0, p1, computeDescriptors(*iteratorBodyPart, p0, p1, frame->getImage(), nbins, size, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels, derivAperture, histogramNormType), useHoGdet);
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
        LimbLabel generatedLabel = generateLabel(*iteratorBodyPart, p0, p1, computeDescriptors(*iteratorBodyPart, p0, p1, frame->getImage(), nbins, size, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels, derivAperture, histogramNormType), useHoGdet);
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
    t.push_back(labels);
  }
  return merge(limbLabels, t);
}

LimbLabel HogDetector::generateLabel(BodyPart bodyPart, Point2f j0, Point2f j1, PartModel descriptors, float _useHoGdet)
{
  vector <Score> s;
  Point2f boxCenter = j0 * 0.5 + j1 * 0.5;
  float rot = float(PoseHelper::angle2D(1, 0, j1.x - j0.x, j1.y - j0.y) * (180.0 / M_PI));
  POSERECT <Point2f> rect = getBodyPartRect(bodyPart, j0, j1);
  stringstream detectorName;
  detectorName << getID();

  float score = compare(bodyPart, descriptors);
  Score sc(score, detectorName.str(), _useHoGdet);
  s.push_back(sc);
  return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s);
}

float HogDetector::compare(BodyPart bodyPart, PartModel model)
{
  float score = 0;
  uint32_t count = 0;
  for (map <uint32_t, map <uint32_t, PartModel>>::iterator framePartModels = partModels.begin(); framePartModels != partModels.end(); ++framePartModels)
  {
    count++;
    for (map <uint32_t, PartModel>::iterator partModel = framePartModels->second.begin(); partModel != framePartModels->second.end(); ++partModel)
    {
      if (partModel->first != static_cast <uint32_t> (bodyPart.getPartID()))
      {
        continue;
      }
      else
      {
        if (model.descriptors.size() != partModel->second.descriptors.size())
        {
          stringstream ss;
          ss << "Invalid descriptor count. Need: " << partModel->second.descriptors.size() << ". Have: " << model.descriptors.size();
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << ss.str() << endl;
          throw logic_error(ss.str());
        }
        for (uint32_t i = 0; i < model.descriptors.size(); ++i)
        {
          try
          {
            score += pow(model.descriptors.at(i) - partModel->second.descriptors.at(i), 2);
          }
          catch (...)
          {
            stringstream ss;
            ss << "Can't get some descriptor at " << i;
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
        }
        score /= model.descriptors.size();
        break;
      }
    }
  }
  return score /= count;
}
