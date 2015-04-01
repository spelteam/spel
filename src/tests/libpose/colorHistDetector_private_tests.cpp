#include <gtest/gtest.h>
#include <colorHistDetector.hpp>
#include "Keyframe.hpp"
#include "Lockframe.hpp"

//IT'S TEMPORARY TESTS
TEST(colorHistDetectorTest, PrivateFields)
{
    cout << "Test 'colorHistDetectorTest.PrivateFields' execution...";
    //Testing PartModel constructor
    uint8_t _nBins = 10;
    const int maxIndex = _nBins - 1;
    ColorHistDetector::PartModel x(_nBins);
    EXPECT_EQ(0.0f, x.partHistogramm[maxIndex][maxIndex][maxIndex]);
    EXPECT_EQ(x.nBins, x.partHistogramm[maxIndex][maxIndex].size());
    EXPECT_EQ(0.0f, x.bgHistogramm[maxIndex][maxIndex][maxIndex]);
    EXPECT_EQ(x.nBins, x.bgHistogramm[maxIndex][maxIndex].size());

    //Testing PartModel operator "="
    x.sizeFG = 1;
    x.sizeBG = 2;
    x.fgNumSamples = 3;
    x.bgNumSamples = 4;
    x.fgSampleSizes.push_back(100);
    x.bgSampleSizes.push_back(200);
    x.fgBlankSizes.push_back(300);
    ColorHistDetector::PartModel y = x;
    EXPECT_EQ(x.partHistogramm, y.partHistogramm);
    EXPECT_EQ(x.bgHistogramm, y.bgHistogramm);
    EXPECT_EQ(x.sizeFG, y.sizeFG);
    EXPECT_EQ(x.fgNumSamples, y.fgNumSamples);
    EXPECT_EQ(x.bgNumSamples, y.bgNumSamples);
    EXPECT_EQ(x.fgSampleSizes, y.fgSampleSizes);
    EXPECT_EQ(x.bgSampleSizes, y.bgSampleSizes);
    EXPECT_EQ(x.fgBlankSizes, y.fgBlankSizes);

    //Testing ColorHistDetector constructor with parameter "_nBins"
    ColorHistDetector chd1(_nBins);
    EXPECT_EQ(_nBins, chd1.nBins);

    //Testing  function computePixelBelongingLikelihood
    const uint8_t nBins = 8, outside = 255;
    const uint8_t _factor = static_cast<uint8_t> (ceil(pow(2, 8) / nBins));
    uint8_t i = 7;
    uint8_t t = i / _factor;
    ColorHistDetector chd2(nBins);
    ColorHistDetector::PartModel z(nBins);
    z.partHistogramm[t][t][t] = 3.14f;
    EXPECT_EQ(z.partHistogramm[t][t][t], chd2.computePixelBelongingLikelihood(z, i, i, i));
    EXPECT_EQ(0.f, chd2.computePixelBelongingLikelihood(z, outside, outside, outside));

    //Testing function setPartHistogramm
    const uint8_t ColorsCount = 255;
    const int K = RAND_MAX / ColorsCount;
    const uint8_t factor = static_cast<uint8_t> (ceil(pow(2, 8) / nBins));
    ColorHistDetector::PartModel partModel1(nBins), partModel2 = partModel1;
    vector <Point3i> Colors;
    for (int c = 0; c < ColorsCount; c++)
        Colors.push_back(Point3i(rand() / K, rand() / K, rand() / K));
    partModel1.sizeFG = ColorsCount;
    partModel1.fgNumSamples = 1;
    partModel1.fgSampleSizes.push_back(static_cast <uint32_t> (Colors.size()));
    for (uint32_t i = 0; i < ColorsCount; i++)
        partModel1.partHistogramm[Colors[i].x / factor][Colors[i].y / factor][Colors[i].z / factor]++;
    for (uint8_t r = 0; r < partModel1.nBins; r++)
    for (uint8_t g = 0; g < partModel1.nBins; g++)
    for (uint8_t b = 0; b < partModel1.nBins; b++)
        partModel1.partHistogramm[r][g][b] /= partModel1.sizeFG;

    ColorHistDetector chd3(nBins);
    chd3.setPartHistogramm(partModel2, Colors);
    EXPECT_EQ(partModel1.partHistogramm, partModel2.partHistogramm);
    EXPECT_EQ(partModel1.bgHistogramm, partModel2.bgHistogramm);
    EXPECT_EQ(partModel1.sizeFG, partModel2.sizeFG);
    EXPECT_EQ(partModel1.fgNumSamples, partModel2.fgNumSamples);
    EXPECT_EQ(partModel1.bgNumSamples, partModel2.bgNumSamples);
    EXPECT_EQ(partModel1.fgSampleSizes, partModel2.fgSampleSizes);
    EXPECT_EQ(partModel1.bgSampleSizes, partModel2.bgSampleSizes);
    EXPECT_EQ(partModel1.fgBlankSizes, partModel2.fgBlankSizes);

    //Testing function addPartHistogramm
    uint32_t nBlankPixels = 100;
    for (uint8_t r = 0; r < partModel1.nBins; r++)
    for (uint8_t g = 0; g < partModel1.nBins; g++)
    for (uint8_t b = 0; b < partModel1.nBins; b++)
        partModel1.partHistogramm[r][g][b] *= partModel1.sizeFG;
    partModel1.sizeFG += static_cast <uint32_t> (Colors.size());
    partModel1.fgNumSamples++;
    partModel1.fgSampleSizes.push_back(Colors.size());
    for (uint32_t i = 0; i < ColorsCount; i++)
        partModel1.partHistogramm[Colors[i].x / factor][Colors[i].y / factor][Colors[i].z / factor]++;
    for (uint8_t r = 0; r < partModel1.nBins; r++)
    for (uint8_t g = 0; g < partModel1.nBins; g++)
    for (uint8_t b = 0; b < partModel1.nBins; b++)
        partModel1.partHistogramm[r][g][b] /= partModel1.sizeFG;
    partModel1.fgBlankSizes.push_back(nBlankPixels);

    chd3.addPartHistogramm(partModel2, Colors, nBlankPixels);
    EXPECT_EQ(partModel1.partHistogramm, partModel2.partHistogramm);
    EXPECT_EQ(partModel1.bgHistogramm, partModel2.bgHistogramm);
    EXPECT_EQ(partModel1.sizeFG, partModel2.sizeFG);
    EXPECT_EQ(partModel1.fgNumSamples, partModel2.fgNumSamples);
    EXPECT_EQ(partModel1.bgNumSamples, partModel2.bgNumSamples);
    EXPECT_EQ(partModel1.fgSampleSizes, partModel2.fgSampleSizes);
    EXPECT_EQ(partModel1.bgSampleSizes, partModel2.bgSampleSizes);
    EXPECT_EQ(partModel1.fgBlankSizes, partModel2.fgBlankSizes);

    //Testing function getAvgSampleSizeFg
    float Sum = 0;
    for (uint32_t i = 0; i < partModel1.fgSampleSizes.size(); i++)
        Sum += partModel1.fgSampleSizes[i];
    Sum /= partModel1.fgNumSamples;
    EXPECT_EQ(Sum, chd3.getAvgSampleSizeFg(partModel2));

    //Testing function getAvgSampleSizeFgBetween
    uint32_t s1 = 0, s2 = 0;
    float f = (partModel1.fgSampleSizes[s1] + partModel1.fgSampleSizes[s2]) / 2.0f;
    EXPECT_EQ(f, chd3.getAvgSampleSizeFgBetween(partModel2, s1, s2));
    EXPECT_EQ(0.f, chd3.getAvgSampleSizeFgBetween(partModel2, static_cast <uint32_t> (partModel1.fgSampleSizes.size()), s2));

    //Testing function matchPartHistogramsED
    float distance = 0;
    for (uint8_t r = 0; r < partModel1.nBins; r++)
    for (uint8_t g = 0; g < partModel1.nBins; g++)
    for (uint8_t b = 0; b < partModel1.nBins; b++)
        distance += pow(partModel1.partHistogramm[r][g][b] - partModel1.partHistogramm[r][g][b], 2.0f);
    f = chd3.matchPartHistogramsED(partModel1, partModel1);
    EXPECT_EQ(sqrt(distance), f);

    //Testing function addBackgroundHistogramm
    vector <Point3i> cEmpty;
    for (uint8_t r = 0; r < partModel1.nBins; r++)
    for (uint8_t g = 0; g < partModel1.nBins; g++)
    for (uint8_t b = 0; b < partModel1.nBins; b++)
        partModel1.bgHistogramm[r][g][b] *= partModel1.sizeBG;
    partModel1.sizeBG += static_cast <uint32_t> (Colors.size());
    partModel1.bgNumSamples++;
    partModel1.bgSampleSizes.push_back(static_cast <uint32_t> (Colors.size()));
    for (uint32_t i = 0; i < Colors.size(); i++)
        partModel1.bgHistogramm[Colors[i].x / factor][Colors[i].y / factor][Colors[i].z / factor]++;
    for (uint8_t r = 0; r < partModel1.nBins; r++)
    for (uint8_t g = 0; g < partModel1.nBins; g++)
    for (uint8_t b = 0; b < partModel1.nBins; b++)
        partModel1.bgHistogramm[r][g][b] /= (float)partModel1.sizeBG;

    chd3.addBackgroundHistogramm(partModel2, cEmpty);
    EXPECT_NE(partModel1.bgHistogramm, partModel2.bgHistogramm);
    chd3.addBackgroundHistogramm(partModel2, Colors);
    EXPECT_EQ(partModel1.partHistogramm, partModel2.partHistogramm);
    EXPECT_EQ(partModel1.bgHistogramm, partModel2.bgHistogramm);
    EXPECT_EQ(partModel1.sizeFG, partModel2.sizeFG);
    EXPECT_EQ(partModel1.fgNumSamples, partModel2.fgNumSamples);
    EXPECT_EQ(partModel1.bgNumSamples, partModel2.bgNumSamples);
    EXPECT_EQ(partModel1.fgSampleSizes, partModel2.fgSampleSizes);
    EXPECT_EQ(partModel1.bgSampleSizes, partModel2.bgSampleSizes);
    EXPECT_EQ(partModel1.fgBlankSizes, partModel2.fgBlankSizes);

    //Temporary. Testing function buildPixelDistributions
    BodyPart bp1;
    tree<BodyPart> tbp;
    const uchar partCount = 4;
    tree<BodyPart>::iterator ti;
    Skeleton skeleton;
    ColorHistDetector chd(nBins);
    const uint32_t rows = 320, cols = 480;
    Mat imgMat = Mat(rows, cols, DataType <Vec3b>::type);
    Mat maskMat = Mat(rows, cols, DataType <uint8_t>::type);

    tbp.clear();
    bp1.setPartID(0);
    bp1.setLWRatio(0.25);
    ti = tbp.begin();
    ti = tbp.insert(ti, bp1);
    chd.partModels[0] = partModel1;
    for (int32_t id = 1; id <= partCount; id++)
    {
        chd.partModels[id] = partModel1;
        bp1.setPartID(id);
        ti = tbp.append_child(ti, bp1);
    }
    skeleton.setPartTree(tbp);
    Frame* frame1 = new(Keyframe);
    frame1->setSkeleton(skeleton);

    for (int x = 0; x < cols; x++)
    for (int y = 0; y < rows; y++)
    {
        imgMat.at<Vec3b>(y, x) = Vec3b(uchar(rand() / K), uchar(rand() / K), uchar(rand() / K));
        maskMat.at<uint8_t>(y, x) = uchar(rand() / K);
    }
    frame1->setImage(imgMat);
    frame1->setMask(maskMat);

    map <int32_t, Mat> pixelDistributions_e;
    for (tree<BodyPart>::iterator I = tbp.begin(); I != tbp.end(); ++I)
    {
        Mat t = Mat(cols, rows, DataType <float>::type);
        int ID = I->getPartID();
        ColorHistDetector::PartModel partModel = chd.partModels[ID];
        for (uint32_t x = 0; x < cols; x++)
        for (uint32_t y = 0; y < rows; y++)
        {
            Vec3b intensity = imgMat.at<Vec3b>(y, x);
            uint8_t blue = intensity.val[0];
            uint8_t green = intensity.val[1];
            uint8_t red = intensity.val[2];
            uint8_t mintensity = maskMat.at<uint8_t>(y, x);
            bool blackPixel = mintensity < 10;
            t.at<float>(x, y) = blackPixel ? 0 : chd.computePixelBelongingLikelihood(partModel, red, green, blue);
        }
        pixelDistributions_e.insert(pair <int32_t, Mat>(ID, t));
        t.release();
    }

    map <int32_t, Mat> pixelDistributions = chd.buildPixelDistributions(frame1);
    for (int i = 0; i < partCount; i++)
    for (uint32_t x = 0; x < cols; x++)
    for (uint32_t y = 0; y < rows; y++)
        EXPECT_EQ(pixelDistributions_e[i].at<float>(x, y), pixelDistributions[i].at<float>(x, y));
    EXPECT_EQ(pixelDistributions_e.size(), pixelDistributions.size());

    for (map <int32_t, Mat>::iterator I = pixelDistributions_e.begin(); I != pixelDistributions_e.end(); ++I)
        I->second.release();
    pixelDistributions_e.clear();

    //Temporary. Testing function BuildPixelLabels
    map <int32_t, Mat> pixelLabels_e;
    for (ti = tbp.begin(); ti != tbp.end(); ++ti)
    {
        Mat t = Mat(cols, rows, DataType <float>::type);
        Mat tt;
        tt = pixelDistributions.at(ti->getPartID());
        for (uint32_t x = 0; x < cols; x++)
        for (uint32_t y = 0; y < rows; y++)
        {
            uint8_t mintensity = maskMat.at<uint8_t>(y, x);
            bool blackPixel = mintensity < 10;
            if (!blackPixel)
            {
                float top = 0, sum = 0;
                for (tree <BodyPart>::iterator i = tbp.begin(); i != tbp.end(); ++i)
                {
                    Mat temp;
                    temp = pixelDistributions.at(i->getPartID());
                    if (temp.at<float>(x, y) > top)
                        top = temp.at<float>(x, y);
                    sum += temp.at<float>(x, y);
                    temp.release();
                }
                t.at<float>(x, y) = (top == 0) ? 0 : tt.at<float>(x, y) / top;
            }
            else
                t.at<float>(x, y) = 0;
        }
        pixelLabels_e.insert(pair<int32_t, Mat>(ti->getPartID(), t));
        t.release();
    }

    map<int32_t, Mat> pixelLabels_a = chd.buildPixelLabels(frame1, pixelDistributions);

    for (int i = 0; i < partCount; i++)
    for (uint32_t x = 0; x < cols; x++)
    for (uint32_t y = 0; y < rows; y++)
        EXPECT_EQ(pixelLabels_e[i].at<float>(x, y), pixelLabels_a[i].at<float>(x, y));
    EXPECT_EQ(pixelLabels_e.size(), pixelLabels_a.size());

    //Temporary. Testing function generateLabel
    map <int32_t, Mat> pixelLabels = chd.buildPixelLabels(frame1, pixelDistributions);
    Point2f j0 = Point2f(cols / 5, rows / 5), j1 = Point2f(cols / 7, rows / 7);
    vector <Score> s;
    vector <Point3i> partPixelColours;
    Point2f boxCenter = j0 * 0.5 + j1 * 0.5;
    float boneLength = chd.getBoneLength(j0, j1);
    float rot = float(PoseHelper::angle2D(1, 0, j1.x - j0.x, j1.y - j0.y) * (180.0 / M_PI));
    POSERECT <Point2f> rect = chd.getBodyPartRect(bp1, j0, j1);
    uint32_t totalPixels = 0, pixelsInMask = 0, pixelsWithLabel = 0;
    float totalPixelLabelScore = 0, pixDistAvg = 0, pixDistNum = 0;
    ColorHistDetector::PartModel model;
    stringstream detectorName;
    detectorName << chd.getID();
    model = chd.partModels.at(bp1.getPartID());

    if (chd.getAvgSampleSizeFg(model) == 0)
    {
        maskMat.release();
        imgMat.release();
    }
    for (int32_t i = int32_t(boxCenter.x - boneLength * 0.5); i < int32_t(boxCenter.x + boneLength * 0.5); i++)
    for (int32_t j = int32_t(boxCenter.y - boneLength * 0.5); j < int32_t(boxCenter.y + boneLength * 0.5); j++)
    if (i < maskMat.cols && j < maskMat.rows)
    {
        float xmax, ymax, xmin, ymin;
        rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
        if (i <= xmax && i >= xmin && j <= ymax && j >= ymin)
        if (rect.containsPoint(Point2f((float)i, (float)j)) > 0)
        {
            totalPixels++;
            uint8_t mintensity = 0;
            mintensity = maskMat.at<uint8_t>(j, i);
            bool blackPixel = mintensity < 10;
            if (!blackPixel)
            {
                pixDistAvg += pixelDistributions.at(bp1.getPartID()).at<float>(i, j);
                pixDistNum++;
                if (pixelLabels.at(bp1.getPartID()).at<float>(i, j))
                {
                    pixelsWithLabel++;
                    totalPixelLabelScore += pixelLabels.at(bp1.getPartID()).at<float>(i, j);
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

    float supportScore = 0, inMaskSupportScore = 0;
    pixDistAvg /= (float)pixDistNum;
    LimbLabel limbLabel_e;
    if (partPixelColours.size() > 0)
    {
        supportScore = (float)totalPixelLabelScore / (float)totalPixels;
        inMaskSupportScore = (float)totalPixelLabelScore / (float)pixelsInMask;
        ColorHistDetector::PartModel model(nBins);
        chd.setPartHistogramm(model, partPixelColours);
        float score = 1.0f - (supportScore + inMaskSupportScore);
        Score sc(score, detectorName.str());
        s.push_back(sc);
        limbLabel_e = LimbLabel(bp1.getPartID(), boxCenter, rot, rect.asVector(), s);
    }
    else
    {
        Score sc(1.0, detectorName.str());
        s.push_back(sc);
        limbLabel_e = LimbLabel(bp1.getPartID(), boxCenter, rot, rect.asVector(), s);
    }

    LimbLabel limbLabel_a = chd.generateLabel(bp1, frame1, pixelDistributions, pixelLabels, j0, j1, 1.0f);

    EXPECT_EQ(limbLabel_e.getLimbID(), limbLabel_a.getLimbID());
    EXPECT_EQ(limbLabel_e.getCenter(), limbLabel_a.getCenter());
    EXPECT_EQ(limbLabel_e.getCenter(), limbLabel_a.getCenter());
    EXPECT_EQ(limbLabel_e.getAngle(), limbLabel_a.getAngle());
    //EXPECT_EQ(limbLabel_e.getScores(), limbLabel_a.getScores());
    EXPECT_EQ(limbLabel_e.getPolygon(), limbLabel_a.getPolygon());
    EXPECT_EQ(limbLabel_e.getIsOccluded(), limbLabel_a.getIsOccluded());
    EXPECT_EQ(limbLabel_e.getIsWeak(), limbLabel_a.getIsWeak());

    for (map <int32_t, Mat>::iterator I = pixelDistributions.begin(); I != pixelDistributions.end(); ++I)
        I->second.release();

    chd.partModels.clear();
    imgMat.release();
    maskMat.release();
    delete frame1;
    tbp.clear();
    pixelDistributions.clear();
    cout << '\r';
}
