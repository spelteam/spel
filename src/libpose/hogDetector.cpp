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

  for (uint32_t row = 0; row < img.rows; row++)
  {
    for (uint32_t col = 0; col < img.cols; col++)
    {
      currentFrameRawDescriptors.insert(pair <PHPoint<uint32_t>, vector <float>>(PHPoint<uint32_t>(row, col), vector <float>()));
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
                  for (row = r; row < cellSize.height; row++)
                  {
                    // cell cols
                    for (col = c; col < cellSize.width; col++)
                    {
                      currentFrameRawDescriptors[PHPoint<uint32_t>(row, col)].push_back(descriptors.at(d));
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
    float boneLength = (float)sqrt(PoseHelper::distSquared(j0, j1));
    //TODO (Vitaliy Koshura): Check this!
    float boneWidth = 0;
    boneWidth = boneLength / part->getLWRatio()/*skeleton.getScale() * boneLength * params.at(sScaleParam)*/;
    Point2f boxCenter = j0 * 0.5 + j1 * 0.5;
    Point2f c1 = Point2f(0.f, 0.5f * boneWidth);
    Point2f c2 = Point2f(boneLength, 0.5f * boneWidth);
    Point2f c3 = Point2f(boneLength, -0.5f * boneWidth);
    Point2f c4 = Point2f(0.f, -0.5f * boneWidth);
    Point2f polyCenter = Point2f(boneLength * 0.5f, 0.f);
    Point2f direction = j1 - j0;
    float rotationAngle = float(PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI));
    // rotate polygon and translate
    c1 = PoseHelper::rotatePoint2D(c1, polyCenter, rotationAngle) + boxCenter - polyCenter;
    c2 = PoseHelper::rotatePoint2D(c2, polyCenter, rotationAngle) + boxCenter - polyCenter;
    c3 = PoseHelper::rotatePoint2D(c3, polyCenter, rotationAngle) + boxCenter - polyCenter;
    c4 = PoseHelper::rotatePoint2D(c4, polyCenter, rotationAngle) + boxCenter - polyCenter;

    PartModel partModel;
    partModel.partModelRect = POSERECT <Point2f>(c1, c2, c3, c4);
    for (uint32_t x = (uint32_t)xmin; x <= (uint32_t)xmax; x++)
    {
      for (uint32_t y = (uint32_t)ymin; y <= (uint32_t)ymax; y++)
      {
        PHPoint<uint32_t> phpoint(x, y);
        if (rect.containsPoint(phpoint))
        {
          partMap.insert(pair <PHPoint<float>, vector<float>>(PHPoint<float>((float)x, (float)y), currentFrameRawDescriptors.at(phpoint)));
          partModel.partDescriptors.insert(pair <PHPoint <uint32_t>, vector <float>>((PoseHelper::rotatePoint2D(Point_<uint32_t>(x, y), Point_<uint32_t>(polyCenter), rotationAngle) + Point_<uint32_t>(boxCenter - polyCenter)), currentFrameRawDescriptors.at(phpoint)));
        }
      }
    }
    frameDescriptors.insert(pair <uint32_t, map <PHPoint<float>, vector<float>>>(part->getPartID(), partMap));
    framePartModels.insert(pair <uint32_t, PartModel>(part->getPartID(), partModel));
  }
  frameBodyPartDescriptors.insert(pair <uint32_t, map <uint32_t, map<PHPoint<float>, vector <float>>>>(frame->getID(), frameDescriptors));
  rawPartModelDescriptors.insert(pair <uint32_t, map <uint32_t, PartModel>>(frame->getID(), framePartModels));
}

//TODO (Vitaliy Koshura): Write real implementation here
void HogDetector::train(vector <Frame*> frames, map <string, float> params)
{
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

  HOGDescriptor detector(wndSize, blockSize, blockStride, cellSize, nbins, wndSigma, thresholdL2hys, gammaCorrection, nlevels);

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
      parseBodyPartDescriptors(*frameNum, computeDescriptors(detector, wndSize, wndStride, blockSize, blockStride, cellSize, nbins, *frameNum));
    }
    catch (...)
    {
      break;
    }
    partModelAverageDescriptors.clear();
    float factor = 1.0f / rawPartModelDescriptors.size();
    for (map <uint32_t, map <uint32_t, PartModel>>::iterator frameModel = rawPartModelDescriptors.begin(); frameModel != rawPartModelDescriptors.end(); ++frameModel)
    {
      for (map <uint32_t, PartModel>::iterator part = frameModel->second.begin(); part != frameModel->second.end(); ++part)
      {
        if (partModelAverageDescriptors.count(part->first) == 0)
        {
          PartModel partModel;
          partModel.partModelRect = part->second.partModelRect;
          for (map <PHPoint<uint32_t>, vector <float>>::iterator descriptors = part->second.partDescriptors.begin(); descriptors != part->second.partDescriptors.end(); ++descriptors)
          {
            if (descriptors->second.size() < nbins)
            {
              throw logic_error("Not enough descriptors");
            }
            else
            {
              vector <float> values;
              for (uint32_t i = 0; i < nbins; i++)
              {
                values.push_back(descriptors->second.at(i) * factor);
              }
              partModel.partDescriptors.insert(pair <PHPoint <uint32_t>, vector <float>>(descriptors->first, values));
            }
          }
          partModelAverageDescriptors.insert(pair <uint32_t, PartModel>(part->first, partModel));
        }
        else
        {
          for (map <PHPoint<uint32_t>, vector <float>>::iterator descriptors = part->second.partDescriptors.begin(); descriptors != part->second.partDescriptors.end(); ++descriptors)
          {
            if (descriptors->second.size() < nbins)
            {
              throw logic_error("Not enough descriptors");
            }
            else
            {
              for (uint32_t i = 0; i < nbins; i++)
              {
                partModelAverageDescriptors.at(part->first).partDescriptors.at(descriptors->first).at(i) += descriptors->second.at(i) * factor;
              }
            }
          }
          
        }
      }
    }

    /*Mat img = (*frameNum)->getImage();
    
    detector.detectMultiScale(img, found, hitThreshold, wndStride, padding, scale0, groupThreshold);
    for (size_t n = 0; n < found.size(); n++)
    {
      Rect r = found[n];
      for (size_t k = 0; k < found.size(); k++)
      {
        if (k != n && (r & found[k]) == r)
        {
          break;
        }
        if (k == found.size())
        {
          found_filtered.push_back(r);
        }
      }
    }*/
  }
}

//TODO (Vitaliy Koshura): Write real implementation here
vector <vector <LimbLabel> > HogDetector::detect(Frame *frame, map <string, float> params)
{
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
  //Mat img = frame->getImage();
  HOGDescriptor detector(wndSize, blockSize, blockStride, cellSize, nbins, wndSigma, thresholdL2hys, gammaCorrection, nlevels);

  computeDescriptors(detector, wndSize, wndStride, blockSize, blockStride, cellSize, nbins, frame);
  
  /*detector.detectMultiScale(img, found, hitThreshold, wndStride, padding, scale0, groupThreshold);
  for (size_t n = 0; n < found.size(); n++)
  {
    Rect r = found[n];
    for (size_t k = 0; k < found.size(); k++)
    {
      if (k != n && (r & found[k]) == r)
      {
        break;
      }
      if (k == found.size())
      {
        found_filtered.push_back(r);
      }
    }
  }*/
  vector <vector <LimbLabel> > t;
  return t;
}

