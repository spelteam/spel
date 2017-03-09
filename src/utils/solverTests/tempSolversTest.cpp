// SPEL definitions
#include "predef.hpp"

#include <iostream>
//#include <thread>
#include "projectLoader.hpp"
#include "spelParameters.hpp"

#include "_Solver.hpp"
#include "spelGeometry.hpp"

using namespace std;
using namespace SPEL;

int main (int argc, char **argv)
{
#if defined(MEMORY_DEBUG) && defined(UNIX)
  Debug(libcw_do.on());
  Debug(dc::malloc.on());
#endif  // MEMORY_DEBUG && UNIX

    /*ios_base::sync_with_stdio(false); //turn off syncing of stdio for async operation
    unsigned int n = std::thread::hardware_concurrency();
    std::cout << n << " concurrent threads are supported.\n";*/ 
    std::cout << " Concerrent threads not used. \n";

    // Checking argv
    if (argc != 3)
    {
      cout << "Usage tempSolverTest [project.xml] [out directory]" << endl;
      return -1;
    }

    string curFolder = argv[1];
    curFolder = curFolder.substr(0, curFolder.find_last_of("/"));
    if (curFolder.back() != '/')
    {
      curFolder += '/';
    }

    // Loading a project
    ProjectLoader projectLoader(curFolder);
    cout << "Loading project..." << endl;
    if (projectLoader.Load(argv[1]) == true)
    {
      cout << "Project was successfully loaded" << endl;
    }
    else
    {
      cout << "Project was not loaded" << endl;
      return -1;
    }
    vector <Frame*> vFrames = projectLoader.getFrames();
    Sequence seq(0, "test", vFrames);

    //=============================================
    // Testing _Solver class

    // Set parameters
    map <string, float> params; //use the default params
    params.emplace("debugLevel", 1); //set the debug setting to highest (0,1,2,3)
    params.emplace("useDedefaultScale", 1.0f);
    params.emplace("defaultScale", 1.0f);
    params.emplace("useCSdet", 0);
    params.emplace("useHoGdet", 1);
    params.emplace("useSURFdet", 0);
    params.emplace("uniqueLocationCandidates", 0.1f);
    params.emplace("uniqueAngleCandidates", 0.1f);
    //params.emplace("searchDistCoeff", 2.0f);
    //params.emplace("searchDistCoeffMult", 1.25f);
    //params.emplace("stepTheta", 5.0f);
    //params.emplace("minHessian", 300.0f);
    //params.emplace("FixedWidthCells", 10.0f);
    //params.emplace("FixedLenghtCells", 10.0f);
    //params.emplace("FixedWidthCells", 4.0f);
    //params.emplace("FixedLenghtCells", 10.0f);    
    cv::Mat image = seq.getFrame(0)->getImage();
    params.emplace("maxFrameHeight", image.size().height);
    image.release();

    // Create ISM
    std::cout << "Create ISM\n";
    long int t0 = clock();
    ImagePixelSimilarityMatrix* M = new ImagePixelSimilarityMatrix();
    M->buildImageSimilarityMatrix(vFrames, 0, 0, false, false);
    long int t1 = clock();
    t1 = (t1 - t0) * 1000 / CLOCKS_PER_SEC;
    cout << "ISM creating time = " << t1 << " ms = " << t1 / 1000 << "s" << endl;

	std::cout << std::endl;

    // Calculate interpolation
    /*  //seq.computeInterpolation(params);
    //vFrames = seq.getFrames();
    clearSkeletons(vFrames);
    //interpolate2(vFrames);
    interpolate3(vFrames, M);
    //propagateKeyFrames(vFrames,M, 0.55f);*/
    
    // Put masks
    /*for (int i = 0; i < vFrames.size(); i++)
    {
      cv::Mat temp = vFrames[i]->getMask().clone();
      Skeleton skeleton = vFrames[i]->getSkeleton();
      putSkeletonMask(temp, skeleton, cv::Size(0, 0), 128);
      imwrite("mask" + to_string(i) + ".jpg", temp);
      temp.release();
    }*/

    // Put interpolation   
    /*for (int i = 0; i < vFrames.size(); i++)
    {
      cv::Mat image = vFrames[i]->getImage();
      Skeleton skeleton = vFrames[i]->getSkeleton();
      cv::Scalar color = cv::Scalar(192, 255, 192);
      if (vFrames[i]->getFrametype() == KEYFRAME)
      {
        color = cv::Scalar(0,0,255);
        cv::putText(image, "Keyframe", Point2f(20.0f, 30.0f), 0, 1.0f, color, 4, 1);
      }
      else
        cv::putText(image, "Interpolated skeleton", Point2f(20.0f, 30.0f), 0, 1.0f, color, 4, 1);
      putSkeleton(image, skeleton, color);

      stringstream S;
      if (i < 10) S << "0";
      if(i < 100) S << "0";
      S << i << ".jpg";
      imwrite("int" + S.str(), image);
      S.clear();
      image.release();
    }*/

    // Run _Solver
    Sequence seqi2(0, "", vFrames);
    cout << "Testing _Solver\n";
    _Solver _solver;
    t0 = clock();
    std::vector<Solvlet> seqSolves = _solver.solve(seqi2, params);
    t1 = clock();
    t1 = (t1 - t0)*1000 / CLOCKS_PER_SEC;
    cout << "Sequence solving time = " << t1 << " ms = " << t1 / 1000 << "s" << endl;

    // Put solves
    /*for (int i = 0; i < seqSolves.size(); i++)
    {
      int frameID = seqSolves[i].getFrameID();
      std::vector<LimbLabel> frameLabels = seqSolves[i].getLabels();
      cv::Mat image = seq.getFrame(frameID)->getImage();
      cv::Scalar color(255, 255, 255);

      putLabels(image, frameLabels, color);

      float y = image.size().height;
      cv::putText(image, "Solve", Point2f(20.0f, 30.0f), 0, 1.0f, color, 4, 1);

      if (params.at("useHoGdet") > 0.01f) cv::putText(image, "used HogDetector", Point2f(20.0f, y - 30.0f), 0, 0.7f, color, 2, 1);
      if (params.at("useCSdet") > 0.01f) cv::putText(image, "used ColorHistDetector", Point2f(20.f, y - 60.0f), 0, 0.7f, color, 2, 1);
      if (params.at("useSURFdet") > 0.01f) cv::putText(image, "used  SURF2Detector", Point2f(20.0f, y - 90.0f), 0, 0.7f, color, 2, 1);

      stringstream S;
      if (frameID < 10) S << "0";
      if (frameID < 100) S << "0";
      S << frameID << ".jpg";
      imwrite("solve" + S.str(), image);
      image.release();
    }*/

    // Put "toSkeleton()"
    /*vFrames = seqi2.getFrames();
    for (int i = 0; i < vFrames.size(); i++)    
    {
      int frameID = vFrames[i]->getID();
      cv::Mat image = seq.getFrame(frameID)->getImage();
      
      cv::Scalar color = Scalar(255, 255, 255);
      if (vFrames[i]->getFrametype() == KEYFRAME)
      {
        color = Scalar(0, 0, 255);
        cv::putText(image, "Keyframe", Point2f(20.0f, 30.0f), 0, 1.0f, color,4, 1);
      }
      else
      {
        float y = image.size().height;
        cv::putText(image, "Solve.toSkeleton", Point2f(20.0f, 30.0f), 0, 1.0f, color, 4, 1);
        if (params.at("useHoGdet") > 0.01f) cv::putText(image, "used HogDetector", Point2f(20.0f, y - 30.0f), 0, 0.7f, color, 2, 1);
        if (params.at("useCSdet") > 0.01f) cv::putText(image, "used ColorHistDetector", Point2f(20.f, y - 60.0f), 0, 0.7f, color, 2, 1);
        if (params.at("useSURFdet") > 0.01f) cv::putText(image, "used  SURF2Detector", Point2f(20.0f, y - 90.0f), 0, 0.7f, color, 2, 1);
      }
               
      putSkeleton(image, vFrames[i]->getSkeleton(), color);

      stringstream S;
      if (frameID < 10) S << "0";
      if (frameID < 100) S << "0";
      S << frameID << ".jpg";
      imwrite("toSkeleton" + S.str(), image);
      if (vFrames[i]->getFrametype() == KEYFRAME)
        imwrite("solve" + S.str(), image);
      S.clear();
      image.release();
    }*/

    //=============================================
    //draw the solution

    std::cout << "Solves size: " << seqSolves.size() << std::endl;

    for(uint32_t i = 0; i < seqSolves.size(); i++)
    {
      Solvlet solve = seqSolves[i];
      int frameID = solve.getFrameID();
      projectLoader.drawFrameSolvlets(solve, vFrames[frameID], argv[2], Scalar(0,0,255), 2);
    }

#if defined(MEMORY_DEBUG) && defined(UNIX)
    Debug(list_allocations_on(libcw_do));
#endif  // MEMORY_DEBUG && UNIX   
    return 0;
}

