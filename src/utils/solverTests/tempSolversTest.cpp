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

    // Checking argv
    if (argc < 3)
    {
      cout << "Usage tempSolverTest [project.xml] [out directory]" << endl;
      return -1;
    }
    string outDirectory = "";
    bool useVisualization = false;
    if (argc > 3)
    {
      outDirectory = argv[2];
      if (outDirectory[outDirectory.size()] != '/')
        outDirectory += "/";
      ProjectLoader::CreateDirectorySystemIndependent(outDirectory);
      useVisualization = true;
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
    cout << "Frames.size = " << vFrames.size() << endl;
    string LogFileName = outDirectory + "_Solver_Log.txt";
    ofstream logFile(LogFileName);
    ofstream* logStream = &logFile;//&cout

    long int t0, t1;
    //=============================================
    // Testing _Solver class

    // Set parameters
    map <string, float> params; //use the default params
    SpelObject::setDebugLevel(1);
    params.emplace("debugLevel", 1); //set the debug setting to highest (0,1,2,3)
    
    params.emplace("useCSdet", 0.0f);
    params.emplace("useHoGdet", 1.0f);
    params.emplace("useSURFdet", 0.0f);
    //params.emplace("searchDistCoeff", 2.0f);
    //params.emplace("searchDistCoeffMult", 2.25f);
    //params.emplace("stepTheta", 5.0f);
    params.emplace("uniqueLocationCandidates", 0.1f);
    params.emplace("uniqueAngleCandidates", 0.1f);  
    params.emplace("useDedefaultScale", 1.0f);
    params.emplace("defaultScale", 1.0f);   
    params.emplace("maxFrameHeight", seq.getFrame(0)->getMask().size().height);

    // Create ISM
    /*
    *logStream << "\nCreate ISM\n";
    t0 = clock();
    ImagePixelSimilarityMatrix* M = new ImagePixelSimilarityMatrix();
    M->buildImageSimilarityMatrix(vFrames, 0, 0, false, false);
    t1 = clock();
    t1 = (t1 - t0) * 1000 / CLOCKS_PER_SEC;
    *logStream << "ISM creating time = " << t1 << " ms = " << t1 / 1000 << "s - Ok" << endl<< endl;
    //M->write("tempSolverTest.ism");*/

    // Calculate interpolation
    t0 = clock();
    clearSkeletons(vFrames);
    /*seq.estimateUniformScale(params);
    seq.computeInterpolation(params);*/
    /*interpolate2(vFrames);
    propagateKeyFrames(vFrames, M, 0.55f);
    interpolate3(vFrames, M);*/
    std::vector<vector<Frame*>> slices = _Solver::createSlices(vFrames);
    for (uint32_t i = 0; i < slices.size(); i++)
      if (slices[i].size() < 13)
        interpolate2(slices[i]);
    seq.setFrames(vFrames);
    t1 = clock();
    *logStream << "Iterpolation creating time = " << t1 << " ms = " << t1 / 1000 << "s - Ok" << endl;

    // Put masks
    if (useVisualization)
    for (int i = 0; i < vFrames.size(); i++)
    {
      cv::Mat temp = vFrames[i]->getMask().clone();
      Skeleton skeleton = vFrames[i]->getSkeleton();
      putSkeletonMask(temp, skeleton, cv::Size(0, 0), 128);
      imwrite(outDirectory + "mask" + to_string(i, 3) + ".jpg", temp);
      temp.release();
    }

    // Put interpolation   
    if (useVisualization)
    for (int i = 0; i < vFrames.size(); i++)
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

      imwrite(outDirectory + "int" + to_string(i, 3) + ".jpg", image);
      image.release();
    }

    // Run _Solver
    *logStream << "Testing _Solver\n";

    _Solver _solver;
    _solver.setLogStream(logStream);
    t0 = clock();
    std::vector<Solvlet> seqSolves = _solver.solve(seq, params);
    t1 = clock();
    t1 = (t1 - t0)*1000 / CLOCKS_PER_SEC;
    *logStream << "Sequence solving time = " << t1 << " ms = " << t1 / 1000 << "s" << endl;

    // Put solves
    if (useVisualization)
    for (int i = 0; i < seqSolves.size(); i++)
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

      imwrite(outDirectory + "solve" + to_string(frameID, 3) + ".jpg", image);
      image.release();
    }

    // Put "toSkeleton()"
    vFrames.clear();
    vFrames = seq.getFrames();
    if (useVisualization)
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

      string S = to_string(frameID, 3) + ".jpg";
      imwrite(outDirectory + "toSkeleton" + S, image);
      if (vFrames[i]->getFrametype() == KEYFRAME)
        imwrite(outDirectory + "solve" + S, image);
      image.release();
    }

    //=============================================
    //draw the solution

    DebugMessage("Log file:\n" + LogFileName, 1);
    *logStream << "Solves size: " << seqSolves.size() << std::endl;
    logFile.close();

    for(uint32_t i = 0; i < seqSolves.size(); i++)
    {
      Solvlet solve = seqSolves[i];
      int frameID = solve.getFrameID();
      projectLoader.drawFrameSolvlets(solve, vFrames[frameID], argv[2], Scalar(0, 0, 255), 2);
    }

#if defined(MEMORY_DEBUG) && defined(UNIX)
    Debug(list_allocations_on(libcw_do));
#endif  // MEMORY_DEBUG && UNIX   
    return 0;
}