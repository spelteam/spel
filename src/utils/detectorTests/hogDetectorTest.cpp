#include "hogDetectorTest.hpp"

void HoGDetectorTest::train(vector <Frame*> _frames, map <string, float> params)
{
  HogDetector::train(_frames, params);
}
map <uint32_t, vector <LimbLabel> > HoGDetectorTest::detect(Frame *frame, map <string, float> params, map <uint32_t, vector <LimbLabel>> limbLabels)
{
  return HogDetector::detect(frame, params, limbLabels);
}
void HoGDetectorTest::DrawSpecific(string outFolder)
{
  drawHoGDescriptors(getPartModels(), getLabelModels(), outFolder, Scalar(255, 0, 0), Scalar(0, 255, 0), 1, 1, getCellSize(), getnbins());
}

bool HoGDetectorTest::drawHoGDescriptors(map <uint32_t, map <uint32_t, PartModel>> partModels, map <uint32_t, map <uint32_t, vector <PartModel>>> labelModels, string outFolder, Scalar lineColor, Scalar descriptorColor, int lineWidth, int descriptorWidth, Size cellSize, uint8_t nbins)
{
  ProjectLoader::CreateDirectorySystemIndependent(outFolder);
  string outFileName = outFolder;
  if (outFileName[outFileName.size()] != '/')
    outFileName += "/";
  outFileName += "img/";
  ProjectLoader::CreateDirectorySystemIndependent(outFileName);
  outFileName += "hog/";
  ProjectLoader::CreateDirectorySystemIndependent(outFileName);

  string tempFileName = outFileName;

  outFileName += "partModels/";
  ProjectLoader::CreateDirectorySystemIndependent(outFileName);

  for (map <uint32_t, map <uint32_t, PartModel>>::iterator i = partModels.begin(); i != partModels.end(); ++i)
  {
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
      Mat img = drawHoGDescriptors(j->second, lineColor, descriptorColor, lineWidth, descriptorWidth, cellSize, nbins);

      cerr << "Writing file " << out << endl;
      imwrite(out, img);
      img.release();
    }
  }

  outFileName = tempFileName;
  outFileName += "labelModels/";
  ProjectLoader::CreateDirectorySystemIndependent(outFileName);

  for (map <uint32_t, map <uint32_t, vector <PartModel>>>::iterator i = labelModels.begin(); i != labelModels.end(); ++i)
  {
    for (map <uint32_t, vector <PartModel>>::iterator j = i->second.begin(); j != i->second.end(); ++j)
    {
      int c = 0;
      for (vector <PartModel>::iterator n = j->second.begin(); n != j->second.end(); ++n)
      {
        string out = outFileName;
        stringstream ss;
        ss << i->first << "/";
        out += ss.str();
        ProjectLoader::CreateDirectorySystemIndependent(out);
        ss.str(string());
        ss.clear();
        ss << j->first << "/";
        out += ss.str();
        ProjectLoader::CreateDirectorySystemIndependent(out);
        ss.str(string());
        ss.clear();
        ss << c << ".png";
        out += ss.str();
        Mat img = drawHoGDescriptors(*n, lineColor, descriptorColor, lineWidth, descriptorWidth, cellSize, nbins);

        cerr << "Writing file " << out << endl;
        imwrite(out, img);
        img.release();
        c++;
      }
    }
  }
  return true;
}

Mat HoGDetectorTest::drawHoGDescriptors(PartModel model, Scalar lineColor, Scalar descriptorColor, int lineWidth, int descriptorWidth, Size cellSize, uint8_t nbins)
{
  const int imgScale = 10;
  const int descriptorScale = 3;

  Mat img;
  resize(model.partImage, img, Size(model.partImage.cols * imgScale, model.partImage.rows * imgScale));
  float radRangeForOneBin = 3.14 / (float)nbins;
  int cells_in_x_dir = model.partImage.cols / cellSize.width;
  int cells_in_y_dir = model.partImage.rows / cellSize.height;

  for (int celly = 0; celly < cells_in_y_dir; celly++)
  {
    for (int cellx = 0; cellx < cells_in_x_dir; cellx++)
    {
      int drawX = cellx * cellSize.width;
      int drawY = celly * cellSize.height;

      int mx = drawX + cellSize.width / 2;
      int my = drawY + cellSize.height / 2;

      rectangle(img, Point(drawX * imgScale, drawY * imgScale), Point((drawX  * imgScale + cellSize.width * imgScale), (drawY  * imgScale + cellSize.height * imgScale)), lineColor, lineWidth);

      // draw in each cell all 9 gradient strengths
      for (int bin = 0; bin < nbins; bin++)
      {
        float currentGradStrength = model.gradientStrengths.at(celly).at(cellx).at(bin);

        // no line to draw?
        if (currentGradStrength == 0)
          continue;

        float currRad = bin * radRangeForOneBin + radRangeForOneBin / 2;

        float dirVecX = cos(currRad);
        float dirVecY = sin(currRad);
        float maxVecLen = cellSize.width / 2;

        // compute line coordinates
        float x1 = mx - dirVecX * currentGradStrength * maxVecLen * descriptorScale;
        float y1 = my - dirVecY * currentGradStrength * maxVecLen * descriptorScale;
        float x2 = mx + dirVecX * currentGradStrength * maxVecLen * descriptorScale;
        float y2 = my + dirVecY * currentGradStrength * maxVecLen * descriptorScale;

        // draw gradient visual_imagealization
        line(img, Point(x1 * imgScale, y1 * imgScale), Point(x2 * imgScale, y2 * imgScale), descriptorColor, descriptorWidth);

      } // for (all bins)

    } // for (cellx)
  } // for (celly)

  return img;
}
