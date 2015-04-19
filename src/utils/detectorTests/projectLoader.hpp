#ifndef _PROJECT_LOADER_HPP_
#define _PROJECT_LOADER_HPP_

#include <iostream>
#include <fstream>
#include <list>

#include <tinyxml2.h>
#include <opencv2/opencv.hpp>
#if OpenCV_VERSION >= 3000
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#endif

#include <tree_util.hh>
#include <detector.hpp>
#include <keyframe.hpp>
#include <interpolation.hpp>
#include <solvlet.hpp>

using namespace std;
using namespace tinyxml2;

class ProjectLoader
{
  public:
    ProjectLoader(string _curFolder);
    void SetCurFolder(string _curFolder);
    bool Load(string fileName);
    bool Save(vector <vector <LimbLabel>> labels, string outFolder, int frameID);
    bool Draw(vector <vector <LimbLabel>> labels, Frame *frame, string outFolder, int frameID, Scalar color, Scalar optimalColor, int lineWidth);
    bool drawFrameSolvlets(Solvlet sol, Frame *frame, string outFolder, Scalar color, int lineWidth);
    void static ResizeImage(Mat &image, int32_t &cols, int32_t &rows);
    vector <Frame*> getFrames(void);
  private:
    string projectTitle;
    string imgFolderPath;
    string maskFolderPath;
    string camFolderPath;
    bool allowScaling;
    string simMathPath;
    string exportPath;

    string curFolder;
    vector <Frame*> vFrames;

    void BuildBodyPartTree(list <BodyPart> vBodyParts, tree <BodyPart> &trBodyPart, tree <BodyPart>::iterator &root);
    void AddChildBodyPartsToTree(list <BodyPart> &vBodyParts, tree <BodyPart> &trBodyPart, tree <BodyPart>::iterator &parent);

};

#endif  // _PROJECT_LOADER_HPP_

