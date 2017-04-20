// SPEL definitions
#include "predef.hpp"

#include "SURFDetector2Test.hpp"

void SURFDetector2Test::train(vector <Frame*> _frames, map <string, float> params)
{
  SURFDetector2::train(_frames, params);
}
map <uint32_t, vector <LimbLabel> > SURFDetector2Test::detect(Frame *frame, map <string, float> params, map <uint32_t, vector <LimbLabel>> limbLabels)
{
  return SURFDetector2::detect(frame, params, limbLabels);
  detected.emplace(std::pair<uint32_t, std::map<uint32_t, std::vector<cv::KeyPoint>>>(frame->getID(), getPartsKeypoints()));
}

void SURFDetector2Test::DrawSpecific(string outFolder)
{
  drawFramesKeyPoints(outFolder, Scalar(0, 255, 0));
}

bool SURFDetector2Test::drawFramesKeyPoints(string outFolder, Scalar color)
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

  for (map <uint32_t, map <uint32_t, vector<cv::KeyPoint>>>::iterator i = detected.begin(); i != detected.end(); ++i)
  {
    Frame *frame = getFrame(i->first);
    if (frame == nullptr)
      continue;
    for (map <uint32_t, vector<cv::KeyPoint>>::iterator j = i->second.begin(); j != i->second.end(); ++j)
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
      Mat img = drawKeyPoints(frame, j->second, color);

      cerr << "Writing file " << out << endl;
      imwrite(out, img);
      img.release();
    }
  }
  return true;
}

Mat SURFDetector2Test::drawKeyPoints(Frame *frame, std::vector<cv::KeyPoint> keypoints, Scalar color)
{
  Mat source = frame->getImage();
  Mat destination = source.clone();
  drawKeypoints(source, keypoints, destination, color, DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  return destination;
}
