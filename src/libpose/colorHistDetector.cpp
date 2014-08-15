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
  for (uint32_t i = 0; i < _frames.size(); i++)
  {
    if (_frames.at(i)->getFrametype() == KEYFRAME || _frames.at(i)->getFrametype() == LOCKFRAME)
    {
      skeleton = _frames.at(i)->getSkeleton();
      break;
    }
    else if (i == _frames.size() - 1)
    {
      return;
    }
  }
  tree <BodyPart> partTree = skeleton.getPartTree();
  tree <BodyPart>::iterator iteratorBodyPart;
  for (uint32_t frameNum = 0; frameNum < _frames.size(); frameNum++)
  {
    if (_frames.at(frameNum)->getFrametype() != KEYFRAME && _frames.at(frameNum)->getFrametype() != LOCKFRAME)
    {
      break;
    }
    vector <vector <Point3i>> partPixelColours;  // vector of vectors, by limb indeces
    vector <vector <Point3i>> bgPixelColours;
    vector <int> blankPixels;  // pixels outside the mask
    skeleton = _frames.at(frameNum)->getSkeleton();
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
        cerr << "Invalid parent joint" << endl;
        break;
      }
      j0 = joint->getImageLocation();
      joint = 0;
      joint = iteratorBodyPart->getChildJoint();
      if (joint == 0)
      {
        cerr << "Invalid child joint" << endl;
        break;
      }
      j1 = joint->getImageLocation();
      float boneLength = sqrt(distSquared(j0, j1));
//TODO (Vitaliy Koshura): Check this!
      float boneWidth = skeleton.getScale() * iteratorBodyPart->getSpaceLength() * params.at(sScaleParam);
      Point2f boxCenter = j0 * 0.5 + j1 * 0.5;
      c1 = Point2f(0, 0.5 * boneWidth);
      c2 = Point2f(boneLength, 0.5 * boneWidth);
      c3 = Point2f(boneLength, -0.5 * boneWidth);
      c4 = Point2f(0, -0.5 * boneWidth);
      Point2f polyCenter = Point2f(boneLength * 0.5, 0);
      Point2f direction = j1 - j0;
      float rotationAngle = angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI);
// rotate polygon and translate
      c1 = rotatePoint2D(c1, polyCenter, rotationAngle) + boxCenter - polyCenter;
      c2 = rotatePoint2D(c2, polyCenter, rotationAngle) + boxCenter - polyCenter;
      c3 = rotatePoint2D(c3, polyCenter, rotationAngle) + boxCenter - polyCenter;
      c4 = rotatePoint2D(c4, polyCenter, rotationAngle) + boxCenter - polyCenter;
      POSERECT <Point2f> poserect(c1, c2, c3, c4);
      polygons[i] = poserect;
      polyDepth[i] = iteratorBodyPart->getParentJoint()->getDepthSign();
      i++;
    }
    Mat maskMat = _frames.at(frameNum)->getMask();
    Mat imgMat = _frames.at(frameNum)->getImage();
    for (int32_t i = 0; i < imgMat.cols; i++)
    {
      for (int32_t j = 0; j < imgMat.rows; j++)
      {
        Vec4b intensity = imgMat.at<Vec4b>(j, i);
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
      uint32_t partNumber = 0;
      PartModel model(nBins);
      addPartHistogramm(partModels.size(), partPixelColours[partNumber], blankPixels[partNumber]);
      addBackgroundHistogramm(partModels.size(), bgPixelColours[partNumber]);
      partModels.push_back(model);
    }
  }
}

//TODO (Vitaliy Koshura): Write real implementation here
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
  for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
  {
    vector <LimbLabel> labels;
    BodyJoint *parentJoint = iteratorBodyPart->getParentJoint();
    BodyJoint *childJoint = iteratorBodyPart->getChildJoint();
    vector <Mat> pixelLabels = buildPixelLabels(frame);
    vector <Point2f> uniqueLocations;
    vector <LimbLabel> sortedLabels;
    vector <vector <LimbLabel>> allLabels;
    Point2f j0 = parentJoint->getImageLocation();
    Point2f j1 = childJoint->getImageLocation();
    Point2f boxCenter = j0 * 0.5 + j1 * 0.5;
    float boneLength = (j0 == j1) ? 1.0 : sqrt(distSquared(j0, j1));
    float boxWidth = skeleton.getScale() * iteratorBodyPart->getSpaceLength() * params.at(sScaleParam);
    Point2f direction = j1 - j0;
    float theta = angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI);
    float minDist = boxWidth * 0.2;
    if (minDist < 2) minDist = 2;
    Mat maskMat = frame->getMask();
    float searchDistance = boneLength * params.at(sSearchDistCoeff);
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
            for (float rot = theta - params.at(sMinTheta); rot < theta + params.at(sMaxTheta); rot += params.at(sStepTheta))
            {
              LimbLabel generatedLabel = generateLabel(*iteratorBodyPart, frame, _pixelDistributions, pixelLabels);
              sortedLabels.push_back(generatedLabel);
            }
          }
        }
      }
    }
    if (sortedLabels.size() > 0)
    {
      sort(sortedLabels.begin(), sortedLabels.end());
      Mat locations(frame->getImage().cols, frame->getImage().rows, DataType<int>::type);
      for (int32_t i = 0; i < frame->getImage().cols; i++)
      {
        for (int32_t j = 0; j < frame->getImage().rows; j++)
        {
          locations.at<int>(i, j) = 0;
        }
      }
      for (uint32_t i = 0; i < sortedLabels.size(); i++)
      {
        uint32_t x = sortedLabels.at(i).getCenter().x;
        uint32_t y = sortedLabels.at(i).getCenter().y;
        if (locations.at<uint32_t>(x, y) < params.at(sUniqueLocationCandidates))
        {
          labels.push_back(sortedLabels.at(i));
          locations.at<uint32_t>(x, y) += 1;
        }
      }
    }
  t.push_back(labels);
  }
  return t;
}

uint8_t ColorHistDetector::getNBins(void)
{
  return nBins;
}

//TODO (Vitaliy Koshura): Need unit test
float ColorHistDetector::computePixelBelongingLikelihood(uint32_t nPartModel, uint8_t r, uint8_t g, uint8_t b)
{
  uint8_t factor = ceil(pow(2, 8)/partModels.at(nPartModel).nBins);
  float isFG = partModels.at(nPartModel).partHistogramm[r/factor][g/factor][b/factor];
  return isFG;
}

//TODO (Vitaliy Koshura>: need unit test
void ColorHistDetector::setPartHistogramm(uint32_t nPartModel, const vector <Point3i> &partColors)
{
  // do not add sample if the number of pixels is zero
  if (partColors.size() == 0)
    return;
  partModels.at(nPartModel).uniqueExists = false;
  uint8_t factor = ceil(pow(2, 8)/partModels.at(nPartModel).nBins);  // divide the color space into bins
  partModels.at(nPartModel).sizeFG = partColors.size();
  partModels.at(nPartModel).fgNumSamples = 1;
  partModels.at(nPartModel).fgSampleSizes.clear();
  partModels.at(nPartModel).fgSampleSizes.push_back(partColors.size());

// clear histogram first
  for(uint8_t r = 0; r < partModels.at(nPartModel).nBins; r++)
  {
    for(uint8_t g = 0; g < partModels.at(nPartModel).nBins; g++)
    {
      for(uint8_t b = 0; b < partModels.at(nPartModel).nBins; b++)
      {
        partModels.at(nPartModel).partHistogramm[r][g][b] = 0.0;
      }
    }
  }
  for(uint32_t i = 0; i < partColors.size(); i++)
  {
    uint8_t r = partColors[i].x / factor;
    uint8_t g = partColors[i].y / factor;
    uint8_t b = partColors[i].z / factor;
    partModels.at(nPartModel).partHistogramm[r][g][b]++;
  }
  for(uint8_t r = 0; r < partModels.at(nPartModel).nBins; r++)
  {
    for(uint8_t g = 0; g < partModels.at(nPartModel).nBins; g++)
    {
      for(uint8_t b = 0; b < partModels.at(nPartModel).nBins; b++)
      {
// normalise the histograms
        partModels.at(nPartModel).partHistogramm[r][g][b] /= partModels.at(nPartModel).sizeFG;
      }
    }
  }
}

//TODO (Vitaliy Koshura>: need unit test
void ColorHistDetector::addPartHistogramm(uint32_t nPartModel, const vector <Point3i> &partColors, uint32_t nBlankPixels)
{
  if(partColors.size()==0) //do not add sample if the number of pixels is zero
    return;
  partModels.at(nPartModel).uniqueExists = false;
//un-normalise
  for(uint8_t r = 0; r < partModels.at(nPartModel).nBins; r++)
  {
    for(uint8_t g = 0; g < partModels.at(nPartModel).nBins; g++)
    {
      for(uint8_t b = 0; b < partModels.at(nPartModel).nBins; b++)
      {
        partModels.at(nPartModel).partHistogramm[r][g][b] *= partModels.at(nPartModel).sizeFG;
      }
    }
  }

  int factor = ceil(pow(2, 8)/partModels.at(nPartModel).nBins);//divide the color space into bins
  partModels.at(nPartModel).sizeFG += partColors.size();
  partModels.at(nPartModel).fgNumSamples++;
  partModels.at(nPartModel).fgSampleSizes.push_back(partColors.size());

  for(uint32_t i = 0; i < partColors.size(); i++)
  {
    uint8_t r = partColors[i].x / factor;
    uint8_t g = partColors[i].y / factor;
    uint8_t b = partColors[i].z / factor;
    partModels.at(nPartModel).partHistogramm[r][g][b]++;
  }

//renormalise
  for(uint8_t r = 0; r < partModels.at(nPartModel).nBins; r++)
  {
    for(uint8_t g = 0; g < partModels.at(nPartModel).nBins; g++)
    {
      for(uint8_t b = 0; b < partModels.at(nPartModel).nBins; b++)
      {
//normalise the histograms
        partModels.at(nPartModel).partHistogramm[r][g][b] /= partModels.at(nPartModel).sizeFG;
      }
    }
  }

  partModels.at(nPartModel).fgBlankSizes.push_back(nBlankPixels); //add the number of blank pixels for this model
}

float ColorHistDetector::getAvgSampleSizeFg(uint32_t nPartModel)
{
  float sum = 0;
  for(uint32_t i = 0; i < partModels.at(nPartModel).fgSampleSizes.size(); i++)
  {
    sum += partModels.at(nPartModel).fgSampleSizes[i];
  }
  sum /= partModels.at(nPartModel).fgNumSamples;
  return sum;
}

float ColorHistDetector::getAvgSampleSizeFgBetween(uint32_t nPartModel, uint32_t s1, uint32_t s2)
{
  if(s1 >= partModels.at(nPartModel).fgSampleSizes.size() || s2 >= partModels.at(nPartModel).fgSampleSizes.size())
    return 0;
  return (partModels.at(nPartModel).fgSampleSizes[s1] + partModels.at(nPartModel).fgSampleSizes[s2]) / 2.0;
}

//TODO (Vitaliy Koshura): Need unit test
// Euclidean distance between part histograms
float ColorHistDetector::matchPartHistogramsED(uint32_t nPartModelPrev, uint32_t nPartModel, uint32_t prevSizeIndex, uint32_t nextSizeIndex) 
{
    float avgSample = getAvgSampleSizeFg(nPartModel);
    float pAvgSample = getAvgSampleSizeFgBetween(nPartModelPrev, prevSizeIndex, nextSizeIndex);
    float avgRatio = avgSample / (avgSample + pAvgSample);
    float alpha = 0.15;  // the impact factor of size difference
    float diff = exp((-1.0 / alpha) * fabs(avgRatio - 0.5)); 
    float distance = 0;
    for(uint8_t r = 0; r < partModels.at(nPartModel).nBins; r++)
    {
        for(uint8_t g = 0; g < partModels.at(nPartModel).nBins; g++)
        {
            for(uint8_t b = 0; b < partModels.at(nPartModel).nBins; b++)
            {
                //normalise the histograms
                distance += pow(partModels.at(nPartModel).partHistogramm[r][g][b] - partModels.at(nPartModelPrev).partHistogramm[r][g][b], 2);
            }
        }
    }
    float score = sqrt(distance);
    return score;
}

void ColorHistDetector::addBackgroundHistogramm(uint32_t nPartModel, const vector <Point3i> &bgColors)
{
  if (bgColors.size() == 0)
    return;
// unnormalise
  for (uint8_t r = 0; r < partModels.at(nPartModel).nBins; r++)
  {
    for (uint8_t g = 0; g < partModels.at(nPartModel).nBins; g++)
    {
      for (uint8_t b = 0; b < partModels.at(nPartModel).nBins; b++)
      {
        partModels.at(nPartModel).bgHistogramm[r][g][b] *= partModels.at(nPartModel).sizeBG;
      }
    }
  }
  uint32_t factor = ceil(pow(2, 8) / partModels.at(nPartModel).nBins);
  partModels.at(nPartModel).sizeBG += bgColors.size();
  partModels.at(nPartModel).bgNumSamples++;
  partModels.at(nPartModel).bgSampleSizes.push_back(bgColors.size());
  for (uint32_t i = 0; i < bgColors.size(); i++)
  {
    partModels.at(nPartModel).bgHistogramm[bgColors[i].x / factor][bgColors[i].y / factor][bgColors[i].z / factor]++;
  }
// renormalise
  for (uint8_t r = 0; r < partModels.at(nPartModel).nBins; r++)
  {
    for (uint8_t g = 0; g < partModels.at(nPartModel).nBins; g++)
    {
      for (uint8_t b = 0; b < partModels.at(nPartModel).nBins; b++)
      {
        partModels.at(nPartModel).bgHistogramm[r][g][b] /= (float)partModels.at(nPartModel).sizeBG;
      }
    }
  }
}

/*
   Return the angle between two vectors on a plane
   The angle is from vector 1 to vector 2, positive anticlockwise
   The result is between -pi -> pi
*/
//TODO (Vitaliy Koshura): Need unit test
double ColorHistDetector::angle2D(double x1, double y1, double x2, double y2)
{
    double dtheta, theta1, theta2;
    theta1 = atan2(y1, x1);
    theta2 = atan2(y2, x2);
    dtheta = theta2 - theta1;
    while (dtheta > M_PI)
        dtheta -= (M_PI * 2.0);
    while (dtheta < -M_PI)
        dtheta += (M_PI * 2.0);
    return(dtheta);
}

vector <Mat> ColorHistDetector::buildPixelDistributions(Frame *frame)
{
  Skeleton skeleton = frame->getSkeleton();
  tree <BodyPart> partTree = skeleton.getPartTree();
  tree <BodyPart>::iterator iteratorBodyPart;
  Mat imgMat = frame->getImage();
  Mat maskMat = frame->getMask();
  uint32_t width = imgMat.cols;
  uint32_t height = imgMat.rows;
  vector <Mat> pixelDistributions;
  for (uint32_t nPartCount = 0; nPartCount < skeleton.getPartTreeCount(); nPartCount++)
  {
    pixelDistributions.push_back(Mat(width, height, DataType <float>::type));
  }
  for (uint32_t x = 0; x < width; x++)
  {
    for (uint32_t y = 0; y < height; y++)
    {
      Vec4b intensity = imgMat.at<Vec4b>(y, x);
      uint8_t blue = intensity.val[0];
      uint8_t green = intensity.val[1];
      uint8_t red = intensity.val[2];
      uint8_t mintensity = maskMat.at<uint8_t>(y, x);
      bool blackPixel = mintensity < 10;
      for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
      {
        pixelDistributions.at(iteratorBodyPart->getPartID()).at<float>(x, y) = blackPixel ? 0 :
          computePixelBelongingLikelihood(iteratorBodyPart->getPartID(), red, green, blue);
      }
    }
  } 
  _pixelDistributions = pixelDistributions;
  return pixelDistributions;
}

vector <Mat> ColorHistDetector::buildPixelLabels(Frame *frame)
{
  vector <Mat> pixelDistributions = buildPixelDistributions(frame);
  Mat maskMat = frame->getMask();
  uint32_t width = maskMat.cols;
  uint32_t height = maskMat.rows;
  Skeleton skeleton = frame->getSkeleton();
  tree <BodyPart> partTree = skeleton.getPartTree();
  tree <BodyPart>::iterator iteratorBodyPart;
  vector <Mat> pixelLabels;
  for (uint32_t nPartCount = 0; nPartCount < skeleton.getPartTreeCount(); nPartCount++)
  {
    pixelLabels.push_back(Mat(width, height, DataType <float>::type));
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
        for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
        {
          if (pixelDistributions.at(iteratorBodyPart->getPartID()).at<float>(x, y) > top)
            top = pixelDistributions.at(iteratorBodyPart->getPartID()).at<float>(x, y);
          sum += pixelDistributions.at(iteratorBodyPart->getPartID()).at<float>(x, y);
        }
        for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
        {
          pixelLabels.at(iteratorBodyPart->getPartID()).at<float>(x, y) = (top == 0) ? 0 :
            pixelDistributions.at(iteratorBodyPart->getPartID()).at<float>(x, y) / (float)top;
        }
      }
      else
      {
        for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
        {
          pixelLabels.at(iteratorBodyPart->getPartID()).at<float>(x, y) = 0;
        }
      }
    }
  }
  return pixelLabels;
}

LimbLabel ColorHistDetector::generateLabel(BodyPart bodyPart, Frame *frame, vector <Mat> pixelDistributions, vector <Mat> pixelLabels)
{
  Mat maskMat = frame->getMask();
  Mat imgMat = frame->getImage();
  Point2f j0 = bodyPart.getParentJoint()->getImageLocation();
  Point2f j1 = bodyPart.getChildJoint()->getImageLocation();
  Point2f boxCenter = j0 * 0.5 + j1 * 0.5;
  float x = boxCenter.x;
  float y = boxCenter.y;
  float boneLength = sqrt(distSquared(j0, j1));
  float boxWidth = frame->getSkeleton().getScale() / bodyPart.getSpaceLength();
  float rot = angle2D(1, 0, j1.x - j0.x, j1.y - j0.y) * (180.0 / M_PI);
  vector <Point3i> partPixelColours;
  Point2f c1, c2, c3, c4, polyCenter;
  c1 = Point2f(0, 0.5 * boxWidth);
  c2 = Point2f(boneLength, 0.5 * boxWidth);
  c3 = Point2f(boneLength, -0.5 * boxWidth);
  c4 = Point2f(0, -0.5 * boxWidth);
  polyCenter = Point2f(boneLength * 0.5, 0);
  c1 = rotatePoint2D(c1, polyCenter, rot) + boxCenter - polyCenter;
  c2 = rotatePoint2D(c2, polyCenter, rot) + boxCenter - polyCenter;
  c3 = rotatePoint2D(c3, polyCenter, rot) + boxCenter - polyCenter;
  c4 = rotatePoint2D(c4, polyCenter, rot) + boxCenter - polyCenter;
  POSERECT <Point2f> rect(c1, c2, c3, c4);
  uint32_t totalPixels = 0;
  uint32_t pixelsInMask = 0;
  uint32_t pixelsWithLabel = 0;
  float totalPixelLabelScore = 0;
  float labelProduct = 1.0;
  float labelAverage = 0;
  float pixDistAvg = 0;
  float pixDistNum = 0;
  if (getAvgSampleSizeFg(bodyPart.getPartID()) == 0)
  {
    vector <Score> v;
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
              pixDistAvg += pixelDistributions.at(bodyPart.getPartID()).at<float>(i, j);
              pixDistNum++;
              if (pixelLabels.at(bodyPart.getPartID()).at<float>(i, j))
              {
                pixelsWithLabel++;
                totalPixelLabelScore += pixelLabels.at(bodyPart.getPartID()).at<float>(i, j);
              }
              Vec4b intensity = imgMat.at<Vec4b>(j, i);
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
  float maskScore = 0;
  float colScore = 1;
  float sizeScore = 0;
  float supportScore = 0;
  float interpolationSimilarityScore = 0;
  float inMaskSupportScore = 0;
  labelAverage = (float)totalPixelLabelScore / (float)pixelsWithLabel;
  pixDistAvg /= (float)pixDistNum;
  if (partPixelColours.size() > 0)
  {
    maskScore = (float)pixelsInMask / (float)totalPixels;
    supportScore = (float)totalPixelLabelScore / (float)totalPixels;
    inMaskSupportScore = (float)totalPixelLabelScore / (float)pixelsInMask;
    sizeScore = 1.0;
    PartModel model(8);
    partModels.push_back(model);
    uint32_t n = partModels.size() - 1;
    setPartHistogramm(n, partPixelColours);
    colScore = matchPartHistogramsED(n, bodyPart.getPartID(), 0, 0);
    partModels.pop_back();
    interpolationSimilarityScore = distSquared(Point2f(x, y), boxCenter);
    vector <Score> s;
    Score sc(1.0 - (supportScore + inMaskSupportScore), "");
    s.push_back(sc);
    return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s);
  }
  vector <Score> s;
  return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s);
}

