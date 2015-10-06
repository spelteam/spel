#include "surfDetectorTest.hpp"

void SURFDetectorTest::train(vector <Frame*> _frames, map <string, float> params)
{
  SurfDetector::train(_frames, params);
}
map <uint32_t, vector <LimbLabel> > SURFDetectorTest::detect(Frame *frame, map <string, float> params, map <uint32_t, vector <LimbLabel>> limbLabels)
{
  return SurfDetector::detect(frame, params, limbLabels);
}

void SURFDetectorTest::DrawSpecific(string outFolder)
{
  drawSURFKeyPoints(getPartModels(), outFolder, Scalar(0, 255, 0));
}

bool SURFDetectorTest::drawSURFKeyPoints(map <uint32_t, map <uint32_t, PartModel>> partModels, string outFolder, Scalar color)
{
  ProjectLoader::CreateDirectorySystemIndependent(outFolder);
  string outFileName = outFolder;
  if (outFileName[outFileName.size()] != '/')
    outFileName += "/";
  outFileName += "img/";
  ProjectLoader::CreateDirectorySystemIndependent(outFileName);
  outFileName += "surf/";
  ProjectLoader::CreateDirectorySystemIndependent(outFileName);

  string tempFileName = outFileName;

  outFileName += "partModels/";
  ProjectLoader::CreateDirectorySystemIndependent(outFileName);

  for (map <uint32_t, map <uint32_t, PartModel>>::iterator i = partModels.begin(); i != partModels.end(); ++i)
  {
    Frame *frame = getFrame(i->first);
    if (frame == nullptr)
      continue;
    for (map <uint32_t, PartModel>::iterator j = i->second.begin(); j != i->second.end(); ++j)
    {
      string out = outFileName;
      stringstream ss;
      ss << i->first << "/";
      out += ss.str();
      ProjectLoader::CreateDirectorySystemIndependent(out);
      ss.str(string());
      ss.clear();
      ss << j->first << ".png";
      out += ss.str();
      Mat img = drawSURFKeyPoints(frame, j->second, color);

      cerr << "Writing file " << out << endl;
      imwrite(out, img);
      img.release();
    }
  }
  return true;
}

Mat SURFDetectorTest::drawSURFKeyPoints(Frame *frame, PartModel model, Scalar color)
{
  Mat source = frame->getImage();
  Mat destination = source.clone();
  drawKeypoints(source, model.keyPoints, destination, color, DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  return destination;
}
