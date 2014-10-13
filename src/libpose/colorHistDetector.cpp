#include "colorHistDetector.hpp"

//TODO (Vitaliy Koshura): Need unit test
ColorHistDetector::PartModel::PartModel(uint8_t _nBins) : nBins(_nBins)
{
  partHistogramm.resize(nBins);
  for (uint8_t r = 0; r < nBins; r++)
  {
    partHistogramm[r].resize(nBins);
    for (uint8_t g = 0; g < nBins; g++)
    {
      partHistogramm[r][g].resize(nBins);
      for (int b = 0; b < nBins; b++)
      {
        partHistogramm[r][g][b] = 0.0;
      }
    }
  }
  sizeFG = 0;
  uniqueExists = false;
}

//TODO (Vitaliy Koshura): Need unit test
ColorHistDetector::ColorHistDetector(uint8_t _nBins) : nBins(_nBins)
{
}

int ColorHistDetector::getID(void)
{
  return id;
}

void ColorHistDetector::setID(int _id)
{
  id = _id;
}

void ColorHistDetector::train(vector <Frame*> _frames, map <string, float> params)
{
  const float scaleParam = 1;
  const string sScaleParam = "scaleParam";
// first we need to check all used params
  if (params.find(sScaleParam) == params.end())
  {
    params[sScaleParam] = scaleParam;
  }

  if (_frames.size() == 0)
    return;
  partModels.clear();
// find skeleton from first keyframe or lockframe
  Skeleton skeleton;
  bool bFind = false;
  for (vector <Frame*>::iterator i = _frames.begin(); i != _frames.end(); ++i)
  {
    Frame *f = *i;
    if (f->getFrametype() == KEYFRAME || f->getFrametype() == LOCKFRAME)
    {
      skeleton = f->getSkeleton();
      bFind = true;
      break;
    }
  }
  if (bFind == false)
  {
    cerr << __FILE__ << ":" << __LINE__ << ": " << "No neither keyrames nor lockframes" << endl;
    return;
  }
  tree <BodyPart> partTree = skeleton.getPartTree();
  tree <BodyPart>::iterator iteratorBodyPart;
  for (vector <Frame*>::iterator frameNum = _frames.begin(); frameNum != _frames.end(); ++frameNum)
  {
    if ((*frameNum)->getFrametype() != KEYFRAME && (*frameNum)->getFrametype() != LOCKFRAME)
    {
      break;
    }
    vector <vector <Point3i>> partPixelColours;  // vector of vectors, by limb indeces
    vector <vector <Point3i>> bgPixelColours;
    vector <int> blankPixels;  // pixels outside the mask
    skeleton = (*frameNum)->getSkeleton();
    vector <POSERECT <Point2f>> polygons;  // polygons for this frame
    vector <bool> polyDepth;
    for (uint32_t i = 0; i < skeleton.getPartTreeCount(); i++)
    {
      blankPixels.push_back(0);
      partPixelColours.push_back(vector <Point3i>());
      bgPixelColours.push_back(vector <Point3i>());
      polygons.push_back(POSERECT <Point2f>());
      polyDepth.push_back(0);
    }
    for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
    {
      int i = 0;
      Point2f j1, j0;  // the two joint for this bone
      Point2f c1, c2, c3, c4;  // the four corners of the rectangle
      BodyJoint *joint = 0;
      joint = iteratorBodyPart->getParentJoint();
      if (joint == 0)
      {
        cerr << __FILE__ << ":" << __LINE__ << ": " << "Invalid parent joint" << endl;
        break;
      }
      j0 = joint->getImageLocation();
      joint = 0;
      joint = iteratorBodyPart->getChildJoint();
      if (joint == 0)
      {
        cerr << __FILE__ << ":" << __LINE__ << ": " << "Invalid child joint" << endl;
        break;
      }
      j1 = joint->getImageLocation();
      float boneLength = sqrt(PoseHelper::distSquared(j0, j1));
//TODO (Vitaliy Koshura): Check this!
      float boneWidth = 0;
      try
      {
        boneWidth = skeleton.getScale() * iteratorBodyPart->getSpaceLength() * params.at(sScaleParam);
      }
      catch(...)
      {
        cerr << __FILE__ << ":" << __LINE__ << ": " << "Maybe there is no '" << sScaleParam << "' param" << endl;
        return;
      }
      Point2f boxCenter = j0 * 0.5 + j1 * 0.5;
      c1 = Point2f(0, 0.5 * boneWidth);
      c2 = Point2f(boneLength, 0.5 * boneWidth);
      c3 = Point2f(boneLength, -0.5 * boneWidth);
      c4 = Point2f(0, -0.5 * boneWidth);
      Point2f polyCenter = Point2f(boneLength * 0.5, 0);
      Point2f direction = j1 - j0;
      float rotationAngle = PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI);
// rotate polygon and translate
      c1 = PoseHelper::rotatePoint2D(c1, polyCenter, rotationAngle) + boxCenter - polyCenter;
      c2 = PoseHelper::rotatePoint2D(c2, polyCenter, rotationAngle) + boxCenter - polyCenter;
      c3 = PoseHelper::rotatePoint2D(c3, polyCenter, rotationAngle) + boxCenter - polyCenter;
      c4 = PoseHelper::rotatePoint2D(c4, polyCenter, rotationAngle) + boxCenter - polyCenter;
      POSERECT <Point2f> poserect(c1, c2, c3, c4);
      polygons[i] = poserect;
      polyDepth[i] = iteratorBodyPart->getParentJoint()->getDepthSign();
      i++;
    }
    Mat maskMat = (*frameNum)->getMask();
    Mat imgMat = (*frameNum)->getImage();
    for (int32_t i = 0; i < imgMat.cols; i++)
    {
      for (int32_t j = 0; j < imgMat.rows; j++)
      {
        Vec3b intensity = imgMat.at<Vec3b>(j, i);
        uint8_t blue = intensity.val[0];
        uint8_t green = intensity.val[1];
        uint8_t red = intensity.val[2];
        uint8_t mintensity = maskMat.at<uint8_t>(j, i);
        bool blackPixel = mintensity < 10;
        int partHit = -1;
        bool depthSign = false;
        for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
        {
          uint32_t partNumber = 0;
          bool bContainsPoint = polygons[partNumber].containsPoint(Point2f(i, j)) > 0;
          if (bContainsPoint && partHit == -1)
          {
            partHit = partNumber;
            depthSign = polyDepth[partNumber];
          }
          else if (bContainsPoint && depthSign == false && polyDepth[partNumber] == true)
          {
            partHit = partNumber;
            depthSign = polyDepth[partNumber];
          }

          partNumber++;
        }
        if (partHit != -1)
        {
          if (!blackPixel)
          {
            partPixelColours[partHit].push_back(Point3i(red, green, blue));
            for (uint32_t p = 0; p < skeleton.getPartTreeCount(); ++p)
            {
              if ((int32_t)p != partHit)
              {
                bgPixelColours[p].push_back(Point3i(red, green, blue));
              }
            }
          }
          else
          {
            blankPixels[partHit]++;
          }
        }
        else
        {
          for (uint32_t p = 0; p < skeleton.getPartTreeCount(); ++p)
          {
            bgPixelColours[p].push_back(Point3i(red, green, blue));
          }
        }
      }
    }
// init partModels
    for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart) 
    {
      int32_t partNumber = iteratorBodyPart->getPartID();
      if (partModels.find(partNumber) == partModels.end())
      {
        PartModel model(nBins);
        partModels.insert(std::pair <int32_t, PartModel> (partNumber, model));
      }
      try
      {
        PartModel &partModel = partModels.at(partNumber);
        addPartHistogramm(partModel, partPixelColours[partNumber], blankPixels[partNumber]);
        addBackgroundHistogramm(partModel, bgPixelColours[partNumber]);
      }
      catch(...)
      {
        cerr << __FILE__ << ":" << __LINE__ << ": " << "Could not find part model " << partNumber << endl;
        return;
      }
      
    }
    maskMat.release();
    imgMat.release();
  }
}

vector <vector <LimbLabel> > ColorHistDetector::detect(Frame *frame, map <string, float> params)
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
  if (params.find(sSearchDistCoeff) == params.end())
  {
    params[sSearchDistCoeff] = searchDistCoeff;
  }
  if (params.find(sMinTheta) == params.end())
  {
    params[sMinTheta] = minTheta;
  }
  if (params.find(sMaxTheta) == params.end())
  {
    params[sMaxTheta] = maxTheta;
  }
  if (params.find(sStepTheta) == params.end())
  {
    params[sStepTheta] = stepTheta;
  }
  if (params.find(sUniqueLocationCandidates) == params.end())
  {
    params[sUniqueLocationCandidates] = uniqueLocationCandidates;
  }
  if (params.find(sScaleParam) == params.end())
  {
    params[sScaleParam] = scaleParam;
  }
   
  vector <vector <LimbLabel> > t;
  Skeleton skeleton = frame->getSkeleton();
  tree <BodyPart> partTree = skeleton.getPartTree();
  tree <BodyPart>::iterator iteratorBodyPart;
  map <int32_t, Mat> pixelDistributions = buildPixelDistributions(frame);
  map <int32_t, Mat> pixelLabels = buildPixelLabels(frame, pixelDistributions);
  Mat maskMat = frame->getMask();
  for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
  {
    vector <LimbLabel> labels;
    BodyJoint *parentJoint = iteratorBodyPart->getParentJoint();
    BodyJoint *childJoint = iteratorBodyPart->getChildJoint();
    vector <Point2f> uniqueLocations;
    vector <LimbLabel> sortedLabels;
    vector <vector <LimbLabel>> allLabels;
    Point2f j0 = parentJoint->getImageLocation();
    Point2f j1 = childJoint->getImageLocation();
    float boneLength = (j0 == j1) ? 1.0 : sqrt(PoseHelper::distSquared(j0, j1));
    float boxWidth = 0;
    try
    {
      boxWidth = skeleton.getScale() * iteratorBodyPart->getSpaceLength() * params.at(sScaleParam);
    }
    catch (...)
    {
      cerr << __FILE__ << ":" << __LINE__ << ": " << "Maybe there is no '" << sScaleParam << "' param" << endl;
      return t;
    }
    Point2f direction = j1 - j0;
    float theta = PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI);
    float minDist = boxWidth * 0.2;
    if (minDist < 2) minDist = 2;
    float searchDistance = 0;
    try
    {
      searchDistance = boneLength * params.at(sSearchDistCoeff);
    }
    catch(...)
    {
      cerr << __FILE__ << ":" << __LINE__ << ": " << "Maybe there is no '" << sSearchDistCoeff << "' param" << endl;
      return t;
    }
    float minTheta = 0, maxTheta = 0, stepTheta = 0;
    try
    {
      minTheta = params.at(sMinTheta);
    }
    catch(...)
    {
      cerr << __FILE__ << ":" << __LINE__ << ": " << "Maybe there is no '" << sMinTheta << "' param" << endl;
      return t;
    }
    try
    {
      maxTheta = params.at(sMaxTheta);
    }
    catch(...)
    {
      cerr << __FILE__ << ":" << __LINE__ << ": " << "Maybe there is no '" << sMaxTheta << "' param" << endl;
      return t;
    }
     try
    {
      stepTheta = params.at(sStepTheta);
    }
    catch(...)
    {
      cerr << __FILE__ << ":" << __LINE__ << ": " << "Maybe there is no '" << sStepTheta << "' param" << endl;
      return t;
    }
    for (float x = 0; x < searchDistance * 0.5; x += minDist)
    {
      for (float y = 0; y < searchDistance * 0.5; y += minDist)
      {
        if (x < maskMat.cols && y < maskMat.rows)
        {
          uint8_t mintensity = maskMat.at<uint8_t>(y, x);
          bool blackPixel = mintensity < 10;
          if (!blackPixel)
          {
            vector <LimbLabel> locationLabels;
            for (float rot = theta - minTheta; rot < theta + maxTheta; rot += stepTheta)
            {
              Point2f p0 = Point2f(0, 0);
              Point2f p1 = Point2f(1.0, 0);
              p1 *= iteratorBodyPart->getSpaceLength();
              p1 = PoseHelper::rotatePoint2D(p1, p0, rot);
              Point2f mid = 0.5 * p1;
              p1 = p1 + Point2f(x, y) - mid;
              p0 = Point2f(x, y) - mid;
              LimbLabel generatedLabel = generateLabel(*iteratorBodyPart, frame, pixelDistributions, pixelLabels, p0, p1);
              sortedLabels.push_back(generatedLabel);
            }
          }
        }
      }
    }
    float uniqueLocationCandidates = 0;
    try
    {
      uniqueLocationCandidates = params.at(sUniqueLocationCandidates);
    }
    catch(...)
    {
      cerr << __FILE__ << ":" << __LINE__ << ": " << "Maybe there is no '" << sUniqueLocationCandidates << "' param" << endl;
      return t;
    }
    if (sortedLabels.size() > 0)
    {
      sort(sortedLabels.begin(), sortedLabels.end());
      Mat locations(frame->getImage().cols, frame->getImage().rows, DataType<uint32_t>::type);
      for (int32_t i = 0; i < frame->getImage().cols; i++)
      {
        for (int32_t j = 0; j < frame->getImage().rows; j++)
        {
          locations.at<uint32_t>(i, j) = 0;
        }
      }
      for (uint32_t i = 0; i < sortedLabels.size(); i++)
      {
        uint32_t x = sortedLabels.at(i).getCenter().x;
        uint32_t y = sortedLabels.at(i).getCenter().y;
        if (locations.at<uint32_t>(x, y) < uniqueLocationCandidates)
        {
          labels.push_back(sortedLabels.at(i));
          locations.at<uint32_t>(x, y) += 1;
        }
      }
      locations.release();
    }
    t.push_back(labels);
  }
  map <int32_t, Mat>::iterator i;
  for (i = pixelDistributions.begin(); i != pixelDistributions.end(); ++i)
  {
    i->second.release();
  }
  maskMat.release();
  return t;
}

uint8_t ColorHistDetector::getNBins(void)
{
  return nBins;
}

//TODO (Vitaliy Koshura): Need unit test
float ColorHistDetector::computePixelBelongingLikelihood(const PartModel &partModel, uint8_t r, uint8_t g, uint8_t b)
{
  uint8_t factor = ceil(pow(2, 8)/partModel.nBins);
  float isFG = partModel.partHistogramm[r/factor][g/factor][b/factor];
  return isFG;
}

//TODO (Vitaliy Koshura>: need unit test
void ColorHistDetector::setPartHistogramm(PartModel &partModel, const vector <Point3i> &partColors)
{
  // do not add sample if the number of pixels is zero
  if (partColors.size() == 0)
    return;
  partModel.uniqueExists = false;
  uint8_t factor = ceil(pow(2, 8)/partModel.nBins);  // divide the color space into bins
  partModel.sizeFG = partColors.size();
  partModel.fgNumSamples = 1;
  partModel.fgSampleSizes.clear();
  partModel.fgSampleSizes.push_back(partColors.size());

// clear histogram first
  for(uint8_t r = 0; r < partModel.nBins; r++)
  {
    for(uint8_t g = 0; g < partModel.nBins; g++)
    {
      for(uint8_t b = 0; b < partModel.nBins; b++)
      {
        partModel.partHistogramm[r][g][b] = 0.0;
      }
    }
  }
  for(uint32_t i = 0; i < partColors.size(); i++)
  {
    uint8_t r = partColors[i].x / factor;
    uint8_t g = partColors[i].y / factor;
    uint8_t b = partColors[i].z / factor;
    partModel.partHistogramm[r][g][b]++;
  }
  for(uint8_t r = 0; r < partModel.nBins; r++)
  {
    for(uint8_t g = 0; g < partModel.nBins; g++)
    {
      for(uint8_t b = 0; b < partModel.nBins; b++)
      {
// normalise the histograms
        partModel.partHistogramm[r][g][b] /= partModel.sizeFG;
      }
    }
  }
}

//TODO (Vitaliy Koshura>: need unit test
void ColorHistDetector::addPartHistogramm(PartModel &partModel, const vector <Point3i> &partColors, uint32_t nBlankPixels)
{
  if(partColors.size()==0) //do not add sample if the number of pixels is zero
    return;
  partModel.uniqueExists = false;
//un-normalise
  for(uint8_t r = 0; r < partModel.nBins; r++)
  {
    for(uint8_t g = 0; g < partModel.nBins; g++)
    {
      for(uint8_t b = 0; b < partModel.nBins; b++)
      {
        partModel.partHistogramm[r][g][b] *= partModel.sizeFG;
      }
    }
  }

  int factor = ceil(pow(2, 8)/partModel.nBins);//divide the color space into bins
  partModel.sizeFG += partColors.size();
  partModel.fgNumSamples++;
  partModel.fgSampleSizes.push_back(partColors.size());

  for(uint32_t i = 0; i < partColors.size(); i++)
  {
    uint8_t r = partColors[i].x / factor;
    uint8_t g = partColors[i].y / factor;
    uint8_t b = partColors[i].z / factor;
    partModel.partHistogramm[r][g][b]++;
  }

//renormalise
  for(uint8_t r = 0; r < partModel.nBins; r++)
  {
    for(uint8_t g = 0; g < partModel.nBins; g++)
    {
      for(uint8_t b = 0; b < partModel.nBins; b++)
      {
//normalise the histograms
        partModel.partHistogramm[r][g][b] /= partModel.sizeFG;
      }
    }
  }

  partModel.fgBlankSizes.push_back(nBlankPixels); //add the number of blank pixels for this model
}

float ColorHistDetector::getAvgSampleSizeFg(const PartModel &partModel)
{
  float sum = 0;
  for(uint32_t i = 0; i < partModel.fgSampleSizes.size(); i++)
  {
    sum += partModel.fgSampleSizes[i];
  }
  sum /= partModel.fgNumSamples;
  return sum;
}

float ColorHistDetector::getAvgSampleSizeFgBetween(const PartModel &partModel, uint32_t s1, uint32_t s2)
{
  if(s1 >= partModel.fgSampleSizes.size() || s2 >= partModel.fgSampleSizes.size())
    return 0;
  return (partModel.fgSampleSizes[s1] + partModel.fgSampleSizes[s2]) / 2.0;
}

//TODO (Vitaliy Koshura): Need unit test
// Euclidean distance between part histograms
float ColorHistDetector::matchPartHistogramsED(const PartModel &partModelPrev, const PartModel &partModel) 
{
  float distance = 0;
  for(uint8_t r = 0; r < partModel.nBins; r++)
  {
    for(uint8_t g = 0; g < partModel.nBins; g++)
    {
      for(uint8_t b = 0; b < partModel.nBins; b++)
      {
        //normalise the histograms
        distance += pow(partModel.partHistogramm[r][g][b] - partModelPrev.partHistogramm[r][g][b], 2);
      }
    }
  }
  float score = sqrt(distance);
  return score;
}

void ColorHistDetector::addBackgroundHistogramm(PartModel &partModel, const vector <Point3i> &bgColors)
{
  if (bgColors.size() == 0)
    return;
// unnormalise
  for (uint8_t r = 0; r < partModel.nBins; r++)
  {
    for (uint8_t g = 0; g < partModel.nBins; g++)
    {
      for (uint8_t b = 0; b < partModel.nBins; b++)
      {
        partModel.bgHistogramm[r][g][b] *= partModel.sizeBG;
      }
    }
  }
  uint32_t factor = ceil(pow(2, 8) / partModel.nBins);
  partModel.sizeBG += bgColors.size();
  partModel.bgNumSamples++;
  partModel.bgSampleSizes.push_back(bgColors.size());
  for (uint32_t i = 0; i < bgColors.size(); i++)
  {
    partModel.bgHistogramm[bgColors[i].x / factor][bgColors[i].y / factor][bgColors[i].z / factor]++;
  }
// renormalise
  for (uint8_t r = 0; r < partModel.nBins; r++)
  {
    for (uint8_t g = 0; g < partModel.nBins; g++)
    {
      for (uint8_t b = 0; b < partModel.nBins; b++)
      {
        partModel.bgHistogramm[r][g][b] /= (float)partModel.sizeBG;
      }
    }
  }
}

map <int32_t, Mat> ColorHistDetector::buildPixelDistributions(Frame *frame)
{
  Skeleton skeleton = frame->getSkeleton();
  tree <BodyPart> partTree = skeleton.getPartTree();
  tree <BodyPart>::iterator iteratorBodyPart;
  Mat imgMat = frame->getImage();
  Mat maskMat = frame->getMask();
  uint32_t width = imgMat.cols;
  uint32_t height = imgMat.rows;
  uint32_t mwidth = maskMat.cols;
  uint32_t mheight = maskMat.rows;
  map <int32_t, Mat> pixelDistributions;
  if (width != mwidth || height != mheight)
  {
    cerr << __FILE__ << ":" << __LINE__ << ": " << "Mask size not equal image size. Calculations are incorrect" << endl;
    return pixelDistributions;
  }
  for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
  {
    Mat t = Mat(width, height, DataType <float>::type);
    int partID = iteratorBodyPart->getPartID();
    try
    {
      PartModel partModel = partModels.at(partID);
      for (uint32_t x = 0; x < width; x++)
      {
        for (uint32_t y = 0; y < height; y++)
        {
          Vec3b intensity = imgMat.at<Vec3b>(y, x);
          uint8_t blue = intensity.val[0];
          uint8_t green = intensity.val[1];
          uint8_t red = intensity.val[2];
          uint8_t mintensity = maskMat.at<uint8_t>(y, x);
          bool blackPixel = mintensity < 10;
  
          t.at<float>(x, y) = blackPixel ? 0 : computePixelBelongingLikelihood(partModel, red, green, blue);
        }
      }
    }
    catch(...)
    {
      cerr << __FILE__ << ":" << __LINE__ << ": " << "Maybe couldn't find partModel " << partID << endl;
      return pixelDistributions;
    }
    pixelDistributions.insert(std::pair <int32_t, Mat> (partID, t));
    t.release();
  }
  imgMat.release();
  maskMat.release();
  return pixelDistributions;
}

map <int32_t, Mat> ColorHistDetector::buildPixelLabels(Frame *frame, map <int32_t, Mat> pixelDistributions)
{
  Mat maskMat = frame->getMask();
  uint32_t width = maskMat.cols;
  uint32_t height = maskMat.rows;
  Skeleton skeleton = frame->getSkeleton();
  tree <BodyPart> partTree = skeleton.getPartTree();
  tree <BodyPart>::iterator iteratorBodyPart;
  map <int32_t, Mat> pixelLabels;
  for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
  {
    Mat t = Mat(width, height, DataType <float>::type);
    Mat tt;
    try
    {
      tt = pixelDistributions.at(iteratorBodyPart->getPartID());
    }
    catch(...)
    {
      cerr << __FILE__ << ":" << __LINE__ << ": " << "Couldn't find distributions for body part " << iteratorBodyPart->getPartID() << endl;
      return pixelLabels;
    }
    for (uint32_t x = 0; x < width; x++)
    {
      for (uint32_t y = 0; y < height; y++)
      {
        uint8_t mintensity = maskMat.at<uint8_t>(y, x);
        bool blackPixel = mintensity < 10;
        if (!blackPixel)
        {
          float top = 0;
          float sum = 0;
          for (tree <BodyPart>::iterator i = partTree.begin(); i != partTree.end(); ++i)
          {
            Mat temp;
            try
            {
              temp = pixelDistributions.at(i->getPartID());
            }
            catch(...)
            {
              cerr << __FILE__ << ":" << __LINE__ << ": " << "Couldn't find pixel distributions for body part " << i->getPartID() << endl;
              return pixelLabels;
            }
            if (temp.at<float>(x, y) > top)
              top = temp.at<float>(x, y);
            sum += temp.at<float>(x, y);
            temp.release();
          }
          t.at<float>(x, y) = (top == 0) ? 0 : tt.at<float>(x, y) / (float)top;
        }
        else
        {
          t.at<float>(x, y) = 0;
        }
      }
    }
    pixelLabels.insert(std::pair<int32_t, Mat> (iteratorBodyPart->getPartID(), t));
    t.release();
  }
  maskMat.release();
  map <int32_t, Mat>::iterator i;
  for (i = pixelDistributions.begin(); i != pixelDistributions.end(); ++i)
  {
    i->second.release();
  }
  return pixelLabels;
}

LimbLabel ColorHistDetector::generateLabel(BodyPart bodyPart, Frame *frame, map <int32_t, Mat> pixelDistributions, map <int32_t, Mat> pixelLabels, Point2f j0, Point2f j1)
{
  Mat maskMat = frame->getMask();
  Mat imgMat = frame->getImage();
  Point2f boxCenter = j0 * 0.5 + j1 * 0.5;
  float x = boxCenter.x;
  float y = boxCenter.y;
  float boneLength = sqrt(PoseHelper::distSquared(j0, j1));
  float boxWidth = frame->getSkeleton().getScale() / bodyPart.getSpaceLength();
  float rot = PoseHelper::angle2D(1, 0, j1.x - j0.x, j1.y - j0.y) * (180.0 / M_PI);
  vector <Point3i> partPixelColours;
  Point2f c1, c2, c3, c4, polyCenter;
  c1 = Point2f(0, 0.5 * boxWidth);
  c2 = Point2f(boneLength, 0.5 * boxWidth);
  c3 = Point2f(boneLength, -0.5 * boxWidth);
  c4 = Point2f(0, -0.5 * boxWidth);
  polyCenter = Point2f(boneLength * 0.5, 0);
  c1 = PoseHelper::rotatePoint2D(c1, polyCenter, rot) + boxCenter - polyCenter;
  c2 = PoseHelper::rotatePoint2D(c2, polyCenter, rot) + boxCenter - polyCenter;
  c3 = PoseHelper::rotatePoint2D(c3, polyCenter, rot) + boxCenter - polyCenter;
  c4 = PoseHelper::rotatePoint2D(c4, polyCenter, rot) + boxCenter - polyCenter;
  POSERECT <Point2f> rect(c1, c2, c3, c4);
  uint32_t totalPixels = 0;
  uint32_t pixelsInMask = 0;
  uint32_t pixelsWithLabel = 0;
  float totalPixelLabelScore = 0;
  float pixDistAvg = 0;
  float pixDistNum = 0;
  if (getAvgSampleSizeFg(bodyPart.getPartID()) == 0)
  {
    vector <Score> v;
    maskMat.release();
    imgMat.release();
    return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), v);
  }
  for (int32_t i = x - boneLength * 0.5; i < x + boneLength * 0.5; i++)
  {
    for (int32_t j = y - boneLength * 0.5; j < y + boneLength * 0.5; j++)
    {
      if (i < maskMat.cols && j < maskMat.rows)
      {
        float xmax = rect.point1.x, ymax = rect.point1.y, xmin = rect.point1.x, ymin = rect.point1.y;
        if (rect.point1.x > rect.point2.x && rect.point1.x > rect.point3.x && rect.point1.x > rect.point4.x) xmax = rect.point1.x;
        else if (rect.point2.x > rect.point1.x && rect.point2.x > rect.point3.x && rect.point2.x > rect.point4.x) xmax = rect.point2.x;
        else if (rect.point3.x > rect.point1.x && rect.point3.x > rect.point2.x && rect.point3.x > rect.point4.x) xmax = rect.point3.x;
        else if (rect.point4.x > rect.point1.x && rect.point4.x > rect.point2.x && rect.point4.x > rect.point3.x) xmax = rect.point4.x;
        if (rect.point1.y > rect.point2.y && rect.point1.y > rect.point3.y && rect.point1.y > rect.point4.y) ymax = rect.point1.y;
        else if (rect.point2.y > rect.point1.y && rect.point2.y > rect.point3.y && rect.point2.y > rect.point4.y) ymax = rect.point2.y;
        else if (rect.point3.y > rect.point1.y && rect.point3.y > rect.point2.y && rect.point3.y > rect.point4.y) ymax = rect.point3.y;
        else if (rect.point4.y > rect.point1.y && rect.point4.y > rect.point2.y && rect.point4.y > rect.point3.y) ymax = rect.point4.y;
        if (rect.point1.x < rect.point2.x && rect.point1.x < rect.point3.x && rect.point1.x < rect.point4.x) xmin = rect.point1.x;
        else if (rect.point2.x < rect.point1.x && rect.point2.x < rect.point3.x && rect.point2.x < rect.point4.x) xmin = rect.point2.x;
        else if (rect.point3.x < rect.point1.x && rect.point3.x < rect.point2.x && rect.point3.x < rect.point4.x) xmin = rect.point3.x;
        else if (rect.point4.x < rect.point1.x && rect.point4.x < rect.point2.x && rect.point4.x < rect.point3.x) xmin = rect.point4.x;
        if (rect.point1.y < rect.point2.y && rect.point1.y < rect.point3.y && rect.point1.y < rect.point4.y) ymin = rect.point1.y;
        else if (rect.point2.y < rect.point1.y && rect.point2.y < rect.point3.y && rect.point2.y < rect.point4.y) ymin = rect.point2.y;
        else if (rect.point3.y < rect.point1.y && rect.point3.y < rect.point2.y && rect.point3.y < rect.point4.y) ymin = rect.point3.y;
        else if (rect.point4.y < rect.point1.y && rect.point4.y < rect.point2.y && rect.point4.y < rect.point3.y) ymin = rect.point4.y;

        if (i <= xmax && i >= xmin && j <= ymax && j >= ymin)
        {
          if (rect.containsPoint(Point2f(i,j)) > 0)
          {
            totalPixels++;
            uint8_t mintensity = maskMat.at<uint8_t>(j, i);
            bool blackPixel = mintensity < 10;
            if (!blackPixel)
            {
              try
              {
                pixDistAvg += pixelDistributions.at(bodyPart.getPartID()).at<float>(i, j);
              }
              catch(...)
              {
                cerr << __FILE__ << ":" << __LINE__ << ": " << "Couldn't find pixel distribution for body part " << bodyPart.getPartID() << endl;
                return LimbLabel();
              }
              pixDistNum++;
              try
              {
                if (pixelLabels.at(bodyPart.getPartID()).at<float>(i, j))
                {
                  pixelsWithLabel++;
                  totalPixelLabelScore += pixelLabels.at(bodyPart.getPartID()).at<float>(i, j);
                }
              }
              catch(...)
              {
                cerr << __FILE__ << ":" << __LINE__ << ": " << "Couldn't find pixel labels for body part " << bodyPart.getPartID() << endl;
                return LimbLabel();
              }
              Vec3b intensity = imgMat.at<Vec3b>(j, i);
              uint8_t blue = intensity.val[0];
              uint8_t green = intensity.val[1];
              uint8_t red = intensity.val[2];
              Point3i ptColor(red, green, blue);
              pixelsInMask++;
              partPixelColours.push_back(ptColor);
            }
          }
        }
      }
    }
  }
  maskMat.release();
  imgMat.release();
  float supportScore = 0;
  float inMaskSupportScore = 0;
  pixDistAvg /= (float)pixDistNum;
  if (partPixelColours.size() > 0)
  {
    supportScore = (float)totalPixelLabelScore / (float)totalPixels;
    inMaskSupportScore = (float)totalPixelLabelScore / (float)pixelsInMask;
    PartModel model(nBins);
    setPartHistogramm(model, partPixelColours);
    vector <Score> s;
    Score sc(1.0 - (supportScore + inMaskSupportScore), "");
    s.push_back(sc);
    return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s);
  }
  vector <Score> s;
  return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s);
}

