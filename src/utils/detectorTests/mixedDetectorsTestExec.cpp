// SPEL definitions
#include "predef.hpp"

#include <iostream>
#include "colorHistDetectorTest.hpp"
#include "hogDetectorTest.hpp"
#include "surfDetectorTest.hpp"
#include "projectLoader.hpp"

using namespace std;

int main(int argc, char **argv)
{
  map <uint32_t, map <uint32_t, vector <LimbLabel>>> limbLabels;

  bool chdRun = true;
  bool hdRun = true;
  bool sdRun = true;

  if (argc < 3)
  {
    cout << "Usage:\t" << argv[0] << " [project.xml] [out directory] [--no-draw] [--no-colorhistdetector] [--no-hogdetector] [--no-surfdetector]" << endl;
    return -1;
  }

  if (argc > 3)
  {
    for (int i = 3; i < argc; i++)
    {
      if (strcmp(argv[i], "--no-colorhistdetector") == 0)
        chdRun = false;
      if (strcmp(argv[i], "--no-hogdetector") == 0)
        hdRun = false;
      if (strcmp(argv[i], "--no-surfdetector") == 0)
        sdRun = false;
    }
  }

  cout << "Run:" << (chdRun ? " ColorHistDetector" : "") << (hdRun ? " HoGDetector" : "") << (sdRun ? " SURFDetector" : "") << endl;
  cout << "Skip:" << (chdRun ? "": " ColorHistDetector") << (hdRun ? "": " HoGDetector") << (sdRun ? "": " SURFDetector") << endl;

  if (chdRun)
  {
    cout << "ColorHistDetector..." << endl;
    ColorHistDetectorTest chdtest;
    chdtest.Run(argc, argv, &limbLabels);
  }
 
  if (hdRun)
  {
    cout << "HoGDetector..." << endl;
    HoGDetectorTest htest;
    htest.Run(argc, argv, &limbLabels);
  }

  if (sdRun)
  {
    cout << "SurfDetector..." << endl;
    SURFDetectorTest stest;
    stest.Run(argc, argv, &limbLabels);
  }

  return 0;
}

