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
  const int nbins = 9;
  double imgSigma = -1;
  double thresholdL2hys = 0.2;
  bool gammaCorrection = true;
  int nlevels = 64;
  double hitThreshold = 0;
  Size imgStride = Size(8, 8);
  Size padding = Size(32, 32);
  double scale0 = 1.05;
  int groupThreshold = 2;

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
    Size imgSize = img.size();
    HOGDescriptor detector(imgSize, blockSize, blockStride, cellSize, nbins, imgSigma, thresholdL2hys, gammaCorrection, nlevels);
    detector.detectMultiScale(img, found, hitThreshold, imgStride, padding, scale0, groupThreshold);
    cout << "Found " << found.size() << endl;
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
  const int nbins = 9;
  double imgSigma = -1;
  double thresholdL2hys = 0.2;
  bool gammaCorrection = true;
  int nlevels = 64;
  double hitThreshold = 0;
  Size imgStride = Size(8, 8);
  Size padding = Size(32, 32);
  double scale0 = 1.05;
  int groupThreshold = 2;

  vector <Rect> found;
  vector <Rect> found_filtered;
  Mat img = frame->getImage();
  Size imgSize = img.size();
  HOGDescriptor detector(imgSize, blockSize, blockStride, cellSize, nbins, imgSigma, thresholdL2hys, gammaCorrection, nlevels);
  detector.detectMultiScale(img, found, hitThreshold, imgStride, padding, scale0, groupThreshold);
  cout << "Found " << found.size() << endl;
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

