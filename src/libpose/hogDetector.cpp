#include "hogDetector.hpp"

#define ERROR_HEADER __FILE__ << ":" << __LINE__ << ": "

int HogDetector::getID(void)
{
  return id;
}

void HogDetector::setID(int _id)
{
  id = _id;
}

map <uint32_t, HogDetector::PartModel> HogDetector::computeDescriptors(Frame *frame, int nbins, Size wndSize, Size wndStride, Size blockSize, Size blockStride, Size cellSize, double wndSigma, double thresholdL2hys, bool gammaCorrection, int nlevels)
{
  map <uint32_t, PartModel> parts;
  HOGDescriptor detector(wndSize, blockSize, blockStride, cellSize, nbins, wndSigma, thresholdL2hys, gammaCorrection, nlevels);
  tree <BodyPart> partTree = frame->getSkeleton().getPartTree();
  Mat imgMat = frame->getImage();
  for (tree <BodyPart>::iterator part = partTree.begin(); part != partTree.end(); ++part)
  {
    Point2f j0, j1;
    BodyJoint *joint = frame->getSkeleton().getBodyJoint(part->getParentJoint());
    if (joint == 0)
    {
#ifdef DEBUG
      cerr << ERROR_HEADER << "Invalid parent joint" << endl;
#endif  // DEBUG
      break;
    }
    j0 = joint->getImageLocation();
    joint = 0;
    joint = frame->getSkeleton().getBodyJoint(part->getChildJoint());
    if (joint == 0)
    {
#ifdef DEBUG
      cerr << ERROR_HEADER << "Invalid child joint" << endl;
#endif  // DEBUG
      break;
    }
    j1 = joint->getImageLocation();
    Point2f boxCenter = j0 * 0.5 + j1 * 0.5;
    float boneLength = getBoneLength(j0, j1);
    float rot = float(PoseHelper::angle2D(1, 0, j1.x - j0.x, j1.y - j0.y) * (180.0 / M_PI));
    POSERECT <Point2f> rect = getBodyPartRect(*part, j0, j1, wndSize);

    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    Point2f polyCenter = Point2f(boneLength * 0.5f, 0.f);
    Point2f direction = j1 - j0;
    float rotationAngle = float(PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI));
    PartModel partModel;
    partModel.partModelRect = rect;
    Mat partImage = rotateImageToDefault(imgMat, partModel.partModelRect, boxCenter, polyCenter, rotationAngle, wndSize);
    detector.compute(partImage, partModel.descriptors);
    parts.insert(pair <uint32_t, PartModel>(part->getPartID(), partModel));
  }
  return parts;
}

map <PHPoint<uint32_t>, vector <float>> HogDetector::computeDescriptors(HOGDescriptor detector, Size wndSize, Size wndStride, Size blockSize, Size blockStride, Size cellSize, int nbins, Frame *frame)
{
  uint64_t d = 0;
  uint32_t i = 0, j = 0, n = 0, k = 0, r = 0, c = 0, b = 0, row = 0, col = 0;
  Mat img = frame->getImage();

  vector <float> descriptors;

  detector.compute(img, descriptors);

#ifdef DEBUG
  cout << "Decriptors: " << descriptors.size() << endl;
#endif  // DEBUG

  map <PHPoint<uint32_t>, vector <float>> currentFrameRawDescriptors;

  for (uint32_t rows = 0; rows < img.rows; rows++)
  {
    for (uint32_t cols = 0; cols < img.cols; cols++)
    {
      currentFrameRawDescriptors.insert(pair <PHPoint<uint32_t>, vector <float>>(PHPoint<uint32_t>(rows, cols), vector <float>()));
    }
  }

  try
  {
    // image rows
    for (i = 0; i + wndSize.height < img.rows + wndStride.height; i += wndStride.height)
    {
      // image cols
      for (j = 0; j + wndSize.width < img.cols + wndStride.width; j += wndStride.width)
      {
        // window rows
        for (n = i; n + blockStride.height < i + wndSize.height; n += blockStride.height)
        {
          // window cols
          for (k = j; k + blockStride.width < j + wndSize.width; k += blockStride.width)
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
                  // cell rows
                  for (row = r; row < r + cellSize.height; row++)
                  {
                    // cell cols
                    for (col = c; col < c + cellSize.width; col++)
                    {
                      // use only first nbins count of descriptors
                      if (currentFrameRawDescriptors[PHPoint<uint32_t>(row, col)].size() < nbins)
                      {
                        currentFrameRawDescriptors[PHPoint<uint32_t>(row, col)].push_back(descriptors.at(d));
                      }
                    }
                  }
                  d++;
                }
              }
            }
          }
        }
      }
    }
  }
  catch (...)
  {
    stringstream ss;
    ss << "Descriptor parse error:" << endl << "Image row:\t" << i << "\tImage col:\t" << j << endl << "Window row:\t" << n << "\tWindow col:\t" << k << endl << "Block row:\t" << r << "\tBlock col:\t" << c << endl << "NBins:\t" << b << endl;
    ss << "Total image rows:\t" << img.rows << "\tTotal image cols:\t" << img.cols << endl;
    ss << "Total descriptors:\t" << descriptors.size() << endl;
    ss << "Trying to get Point at:\t" << row << ":" << col << endl;
    ss << "Trying to get descriptor at:\t" << d << endl;
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif // DEBUG
    throw logic_error(ss.str());
  }

  if (d < descriptors.size())
  {
    stringstream ss;
    ss << "Error. Not all descriptors were parsed" << endl;
    ss << "Last parsed descriptor:\t" << d << endl;
    ss << "Total descriptor count:\t" << descriptors.size() << endl;
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif // DEBUG
    throw logic_error(ss.str());
  }

  rawDescriptors.insert(pair <uint32_t, map <PHPoint<uint32_t>, vector <float>>>(frame->getID(), currentFrameRawDescriptors));
  return currentFrameRawDescriptors;
}

void HogDetector::parseBodyPartDescriptors(Frame *frame, map <PHPoint<uint32_t>, vector <float>> currentFrameRawDescriptors)
{

  tree <BodyPart> trBodyPart = frame->getSkeleton().getPartTree();
  map <uint32_t, map<PHPoint<float>, vector <float>>> frameDescriptors;
  map <uint32_t, PartModel> framePartModels;
  for (tree <BodyPart>::iterator part = trBodyPart.begin(); part != trBodyPart.end(); ++part)
  {
    map <PHPoint <float>, vector <float>> partMap;
    POSERECT<Point2f> rect = part->getPartPolygon();
    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);

    map <PHPoint<uint32_t>, vector <float>> partModelPartDescriptors;
    Point2f j0, j1;
    BodyJoint *joint = frame->getSkeleton().getBodyJoint(part->getParentJoint());
    if (joint == 0)
    {
#ifdef DEBUG
      cerr << ERROR_HEADER << "Invalid parent joint" << endl;
#endif  // DEBUG
      break;
    }
    j0 = joint->getImageLocation();
    joint = 0;
    joint = frame->getSkeleton().getBodyJoint(part->getChildJoint());
    if (joint == 0)
    {
#ifdef DEBUG
      cerr << ERROR_HEADER << "Invalid child joint" << endl;
#endif  // DEBUG
      break;
    }
    j1 = joint->getImageLocation();
    float boneLength = getBoneLength(j0, j1);
    //TODO (Vitaliy Koshura): Check this!
    float boneWidth = 0;
    boneWidth = getBoneWidth(boneLength, *part);
    Point2f boxCenter = j0 * 0.5 + j1 * 0.5;
    Point2f polyCenter = Point2f(boneLength * 0.5f, 0.f);
    Point2f direction = j1 - j0;
    float rotationAngle = float(PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI));
    PartModel partModel;
    partModel.partModelRect = getBodyPartRect(*part, j0, j1);
    (PoseHelper::rotatePoint2D(Point_<float>(partModel.partModelRect.point1), Point_<float>((CvPoint2D32f)polyCenter), rotationAngle) + Point_<float>((CvPoint2D32f)(boxCenter - polyCenter)));
    (PoseHelper::rotatePoint2D(Point_<float>(partModel.partModelRect.point2), Point_<float>((CvPoint2D32f)polyCenter), rotationAngle) + Point_<float>((CvPoint2D32f)(boxCenter - polyCenter)));
    (PoseHelper::rotatePoint2D(Point_<float>(partModel.partModelRect.point3), Point_<float>((CvPoint2D32f)polyCenter), rotationAngle) + Point_<float>((CvPoint2D32f)(boxCenter - polyCenter)));
    (PoseHelper::rotatePoint2D(Point_<float>(partModel.partModelRect.point4), Point_<float>((CvPoint2D32f)polyCenter), rotationAngle) + Point_<float>((CvPoint2D32f)(boxCenter - polyCenter)));
    for (uint32_t x = (uint32_t)xmin; x <= (uint32_t)xmax; x++)
    {
      for (uint32_t y = (uint32_t)ymin; y <= (uint32_t)ymax; y++)
      {
        PHPoint<uint32_t> phpoint(x, y);
        if (rect.containsPoint(phpoint))
        {
          partMap.insert(pair <PHPoint<float>, vector<float>>(PHPoint<float>((float)x, (float)y), currentFrameRawDescriptors.at(phpoint)));
          partModel.partDescriptors.insert(pair <PHPoint <uint32_t>, vector <float>>((PoseHelper::rotatePoint2D(Point_<uint32_t>(x, y), Point_<uint32_t>((CvPoint2D32f)polyCenter), rotationAngle) + Point_<uint32_t>((CvPoint2D32f)(boxCenter - polyCenter))), currentFrameRawDescriptors.at(phpoint)));
        }
      }
    }
    frameDescriptors.insert(pair <uint32_t, map <PHPoint<float>, vector<float>>>(part->getPartID(), partMap));
    framePartModels.insert(pair <uint32_t, PartModel>(part->getPartID(), partModel));
  }
  frameBodyPartDescriptors.insert(pair <uint32_t, map <uint32_t, map<PHPoint<float>, vector <float>>>>(frame->getID(), frameDescriptors));
  rawPartModelDescriptors.insert(pair <uint32_t, map <uint32_t, PartModel>>(frame->getID(), framePartModels));
}

map <uint32_t, Size> HogDetector::getMaxBodyPartHeightWidth(vector <Frame*> frames, Size blockSize)
{
  map <uint32_t, Size> result;
  for (vector <Frame*>::iterator frame = frames.begin(); frame != frames.end(); ++frame)
  {
    Skeleton skeleton = (*frame)->getSkeleton();
    tree <BodyPart> bodyParts = skeleton.getPartTree();
    for (tree <BodyPart>::iterator bodyPart = bodyParts.begin(); bodyPart != bodyParts.end(); ++bodyPart)
    {
      Point2f j0, j1;
      BodyJoint *joint = skeleton.getBodyJoint(bodyPart->getParentJoint());
      if (joint == 0)
      {
#ifdef DEBUG
        cerr << ERROR_HEADER << "Invalid parent joint" << endl;
#endif  // DEBUG
        break;
      }
      j0 = joint->getImageLocation();
      joint = 0;
      joint = skeleton.getBodyJoint(bodyPart->getChildJoint());
      if (joint == 0)
      {
#ifdef DEBUG
        cerr << ERROR_HEADER << "Invalid child joint" << endl;
#endif  // DEBUG
        break;
      }
      j1 = joint->getImageLocation();
      float boneLength = getBoneLength(j0, j1);
      //TODO (Vitaliy Koshura): Check this!
      float boneWidth = 0;
      boneWidth = getBoneWidth(boneLength, *bodyPart);

      Size maxSize = Size(boneLength, boneWidth);
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
    part->second.width += part->second.width % blockSize.width;
    part->second.height += part->second.height % blockSize.height;
  }
  return result;
}

//TODO (Vitaliy Koshura): Write real implementation here
void HogDetector::train(vector <Frame*> _frames, map <string, float> params)
{
  frames = _frames;
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
  double hitThreshold = 0;
  Size wndStride = Size(8, 8);
  Size padding = Size(32, 32);
  double scale0 = 1.05;
  int groupThreshold = 2;

  map <uint32_t, Size> partSize = getMaxBodyPartHeightWidth(_frames, blockSize);

  for (vector <Frame*>::iterator frameNum = frames.begin(); frameNum != frames.end(); ++frameNum)
  {
    if ((*frameNum)->getFrametype() != KEYFRAME && (*frameNum)->getFrametype() != LOCKFRAME)
    {
      continue;
    }

#ifdef DEBUG
    cerr << "Training on frame " << (*frameNum)->getID() << endl;
#endif  // DEBUG
    vector <Rect> found;
    vector <Rect> found_filtered;

    try
    {
      computeDescriptors(*frameNum, nbins, wndSize, wndStride, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels);
    }
    catch (...)
    {
      break;
    }
  }
}

//TODO (Vitaliy Koshura): Write real implementation here
vector <vector <LimbLabel> > HogDetector::detect(Frame *frame, map <string, float> params)
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

  const float scaleParam = 1;
  const string sScaleParam = "scaleParam";

  // first we need to check all used params
  params.emplace(sSearchDistCoeff, searchDistCoeff);
  params.emplace(sMinTheta, minTheta);
  params.emplace(sMaxTheta, maxTheta);
  params.emplace(sStepTheta, stepTheta);
  params.emplace(sUniqueLocationCandidates, uniqueLocationCandidates);
  params.emplace(sScaleParam, scaleParam);

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
  double hitThreshold = 0;
  Size wndStride = Size(8, 8);
  Size padding = Size(32, 32);
  double scale0 = 1.05;
  int groupThreshold = 2;

  vector <Rect> found;
  vector <Rect> found_filtered;

  HOGDescriptor detector(wndSize, blockSize, blockStride, cellSize, nbins, wndSigma, thresholdL2hys, gammaCorrection, nlevels);
  
  vector <vector <LimbLabel> > t;

  Skeleton skeleton = frame->getSkeleton();
  tree <BodyPart> partTree = skeleton.getPartTree();
  tree <BodyPart>::iterator iteratorBodyPart;

  Mat maskMat = frame->getMask();
  Frame *prevFrame = 0, *nextFrame = 0;
  uint32_t stepCount = 0;
  uint32_t step = 0;
  getNeighborFrame(frame, &prevFrame, &nextFrame, step, stepCount);
  if (prevFrame == 0)
  {
    stringstream ss;
    ss << "Couldn't find previous keyframe to the frame " << frame->getID();
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
    throw logic_error(ss.str());
  }
  if (nextFrame == 0)
  {
    stringstream ss;
    ss << "Couldn't find next feyframe to the frame " << frame->getID();
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
    throw logic_error(ss.str());
  }

  map <PHPoint<uint32_t>, vector <float>> descriptors = computeDescriptors(detector, wndSize, wndStride, blockSize, blockStride, cellSize, nbins, frame);

  for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
  {
    vector <LimbLabel> labels;
    vector <Point2f> uniqueLocations;
    vector <LimbLabel> sortedLabels;
    vector <vector <LimbLabel>> allLabels;
    Point2f j0;
    Point2f j1;

    getRawBodyPartPosition(frame, prevFrame, nextFrame, iteratorBodyPart->getParentJoint(), iteratorBodyPart->getChildJoint(), step, stepCount, j0, j1);

    float boneLength = getBoneLength(j0, j1);
    float boxWidth = getBoneWidth(boneLength, *iteratorBodyPart);
    Point2f direction = j1 - j0;
    float theta = float(PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI));
    float minDist = boxWidth * 0.2f;
    if (minDist < 2) minDist = 2;
    float searchDistance = 0;
    try
    {
      searchDistance = boneLength * params.at(sSearchDistCoeff);
    }
    catch (...)
    {
      stringstream ss;
      ss << "Maybe there is no '" << sSearchDistCoeff << "' param";
#ifdef DEBUG
      cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
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
#ifdef DEBUG
      cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
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
#ifdef DEBUG
      cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
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
#ifdef DEBUG
      cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
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
#ifdef DEBUG
            cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
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
              LimbLabel generatedLabel = generateLabel(*iteratorBodyPart, frame, p0, p1, descriptors, nbins);
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
        LimbLabel generatedLabel = generateLabel(*iteratorBodyPart, frame, p0, p1, descriptors, nbins);
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
#ifdef DEBUG
      cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
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
#ifdef DEBUG
            cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
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
#ifdef DEBUG
              cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
              throw logic_error(ss.str());
            }
            locations.at<uint32_t>(x, y) += 1;
          }
        }
        catch (...)
        {
          stringstream ss;
          ss << "Maybe there is no value of locations at " << "[" << x << "][" << y << "]";
#ifdef DEBUG
          cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
          throw logic_error(ss.str());
        }
      }
      locations.release();
    }
    t.push_back(labels);
  }
  map <int32_t, Mat>::iterator i;
  maskMat.release();

  return t;
}

LimbLabel HogDetector::generateLabel(BodyPart bodyPart, Frame *frame, Point2f j0, Point2f j1, map <PHPoint<uint32_t>, vector <float>> descriptors, uint8_t nBins)
{
  vector <Score> s;
  Mat maskMat = frame->getMask();
  Mat imgMat = frame->getImage();
  Point2f boxCenter = j0 * 0.5 + j1 * 0.5;
  float boneLength = getBoneLength(j0, j1);
  float rot = float(PoseHelper::angle2D(1, 0, j1.x - j0.x, j1.y - j0.y) * (180.0 / M_PI));
  POSERECT <Point2f> rect = getBodyPartRect(bodyPart, j0, j1);
  stringstream detectorName;
  detectorName << getID();

  float xmax, ymax, xmin, ymin;
  rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
  Point2f polyCenter = Point2f(boneLength * 0.5f, 0.f);
  Point2f direction = j1 - j0;
  float rotationAngle = float(PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI));
  PartModel partModel;
  partModel.partModelRect = rect;
  (PoseHelper::rotatePoint2D(Point_<float>(partModel.partModelRect.point1), Point_<float>((CvPoint2D32f)polyCenter), rotationAngle) + Point_<float>((CvPoint2D32f)(boxCenter - polyCenter)));
  (PoseHelper::rotatePoint2D(Point_<float>(partModel.partModelRect.point2), Point_<float>((CvPoint2D32f)polyCenter), rotationAngle) + Point_<float>((CvPoint2D32f)(boxCenter - polyCenter)));
  (PoseHelper::rotatePoint2D(Point_<float>(partModel.partModelRect.point3), Point_<float>((CvPoint2D32f)polyCenter), rotationAngle) + Point_<float>((CvPoint2D32f)(boxCenter - polyCenter)));
  (PoseHelper::rotatePoint2D(Point_<float>(partModel.partModelRect.point4), Point_<float>((CvPoint2D32f)polyCenter), rotationAngle) + Point_<float>((CvPoint2D32f)(boxCenter - polyCenter)));
  cout << "Generate label begin" << endl;
  for (uint32_t x = (uint32_t)xmin; x <= (uint32_t)xmax; x++)
  {
    for (uint32_t y = (uint32_t)ymin; y <= (uint32_t)ymax; y++)
    {
      PHPoint<uint32_t> phpoint(x, y);
      if (rect.containsPoint(phpoint))
      {
        phpoint = PoseHelper::rotatePoint2D(Point_<uint32_t>(x, y), Point_<uint32_t>((CvPoint2D32f)polyCenter), rotationAngle) + Point_<uint32_t>((CvPoint2D32f)(boxCenter - polyCenter));
        partModel.partDescriptors.insert(pair <PHPoint <uint32_t>, vector <float>>(phpoint, descriptors.at(phpoint)));
      }
    }
  }
  float score = partModel.compare(partModelAverageDescriptors.at(bodyPart.getPartID()), nBins);
  Score sc(score, detectorName.str());
  s.push_back(sc);
  cout << "Generate label end" << endl;
  return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s);
}

float HogDetector::PartModel::compare(PartModel ethalon, uint8_t nBins)
{
  cout << "Compare begin" << endl;
  uint32_t ethalonTotalCount = ethalon.partDescriptors.size();
  uint32_t totalCount = partDescriptors.size();
  uint32_t comparedCount = 0.0;
  float mMinX, mMinY, mMaxX, mMaxY, eMinX, eMinY, eMaxX, eMaxY, minX, minY, maxX, maxY;
  partModelRect.GetMinMaxXY(mMinX, mMinY, mMaxX, mMaxY);
  ethalon.partModelRect.GetMinMaxXY(eMinX, eMinY, eMaxX, eMaxY);
  minX = max(mMinX, eMinX);
  minY = max(mMinY, eMinY);
  maxX = min(mMaxX, eMaxX);
  maxY = min(mMaxY, eMaxY);
  float score = 0.0;

  for (float x = minX; x <= maxX; x++)
  {
    for (float y = minY; y <= maxY; y++)
    {
      vector <float> ev, mv;
      try
      {
        mv = partDescriptors.at(PHPoint<uint32_t>(x, y));
      }
      catch (...)
      {
        stringstream ss;
        ss << "Maybe there is no value of detection partDescriptors at " << "[" << x << "][" << y << "]";
#ifdef DEBUG
        cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
        continue;
      }
      try
      {
        ev = partDescriptors.at(PHPoint<uint32_t>(x, y));
      }
      catch (...)
      {
        stringstream ss;
        ss << "Maybe there is no value of ethalon partDescriptors at " << "[" << x << "][" << y << "]";
#ifdef DEBUG
        cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
        continue;
      }
      if (mv.size() < nBins)
      {
        stringstream ss;
        ss << "Not enough nbins in detection model: " << mv.size();
#ifdef DEBUG
        cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
        continue;
      }
      if (ev.size() < nBins)
      {
        stringstream ss;
        ss << "Not enough nbins in ethalon model: " << mv.size();
#ifdef DEBUG
        cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
        continue;
      }
      float localscore = 0.0;
      for (uint8_t c = 0; c < nBins; c++)
      {
        float m, e;
        try
        {
          m = mv.at(c);
        }
        catch (...)
        {
          stringstream ss;
          ss << "Can't get value of model: " << mv.size();
#ifdef DEBUG
          cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
          continue;
        }
        try
        {
          e = ev.at(c);
        }
        catch (...)
        {
          stringstream ss;
          ss << "Can't get value of ethalon: " << mv.size();
#ifdef DEBUG
          cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
          continue;
        }
        localscore += abs(m - e);
      }
      score += localscore / nBins;
      comparedCount++;
    }
  }
  score /= comparedCount;
  score += (comparedCount / totalCount);
  cout << "Compare end" << endl;
  return score; 
}
