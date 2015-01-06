#include "hogDetector.hpp"

int HogDetector::getID(void)
{
  return id;
}

void HogDetector::setID(int _id)
{
  id = _id;
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
    Mat img = (*frameNum)->getImage();

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
    uint64_t d = 0;
    uint32_t i = 0, j = 0, n = 0, k = 0, r = 0, c = 0, b = 0, row = 0, col = 0;

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
      cerr << "Descriptor parse error:" << endl << "Image row:\t" << i << "\tImage col:\t" << j << endl << "Window row:\t" << n << "\tWindow col:\t" << k << endl << "Block row:\t" << r << "\tBlock col:\t" << c << endl << "NBins:\t" << b << endl;
      cerr << "Total image rows:\t" << img.rows << "\tTotal image cols:\t" << img.cols << endl;
      cerr << "Total descriptors:\t" << descriptors.size() << endl;
      cerr << "Trying to get Point at:\t" << row << ":" << col << endl;
      cerr << "Trying to get descriptor at:\t" << d << endl;
      break;
    }

    if (d < descriptors.size())
    {
      cerr << "Error. Not all descriptors were parsed" << endl;
      cerr << "Last parsed descriptor:\t" << d << endl;
      cerr << "Total descriptor count:\t" << descriptors.size() << endl;
      break;
    }

    rawDescriptors.insert(pair <uint32_t, map <PHPoint<uint32_t>, vector <float>>>((*frameNum)->getID(), currentFrameRawDescriptors));

    tree <BodyPart> trBodyPart = (*frameNum)->getSkeleton().getPartTree();
    map <uint32_t, map<PHPoint<float>, vector <float>>> frameDescriptors;
    for (tree <BodyPart>::iterator part = trBodyPart.begin(); part != trBodyPart.end(); ++part)
    {
      map <PHPoint <float>, vector <float>> partMap;
      POSERECT<Point2f> rect = part->getPartPolygon();      
      float xmax, ymax, xmin, ymin;
      rect.GetMinMaxXY <float> (xmin, ymin, xmax, ymax);

      for (uint32_t x = (uint32_t)xmin; x <= (uint32_t)xmax; x++)
      {
        for (uint32_t y = (uint32_t)ymin; y <= (uint32_t)ymax; y++)
        {
          Point2f point((float)x, (float)y);
          PHPoint<uint32_t> phpoint(x, y);
          if (rect.containsPoint(point))
          {
            partMap.insert(pair <PHPoint<float>, vector<float>>(PHPoint<float>((float)x, (float)y), currentFrameRawDescriptors.at(phpoint)));
          }
        }
      }
      frameDescriptors.insert(pair <uint32_t, map <PHPoint<float>, vector<float>>>(part->getPartID(), partMap));
    }
    
    frameBodyPartDescriptors.insert(pair <uint32_t, map <uint32_t, map<PHPoint<float>, vector <float>>>>((*frameNum)->getID(), frameDescriptors));

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
    }
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
  Mat img = frame->getImage();
  HOGDescriptor detector(wndSize, blockSize, blockStride, cellSize, nbins, wndSigma, thresholdL2hys, gammaCorrection, nlevels);
  
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
  }
  vector <vector <LimbLabel> > t;
  return t;
}

