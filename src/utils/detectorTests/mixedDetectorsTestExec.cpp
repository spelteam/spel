#include <iostream>
#include "colorHistDetectorTest.hpp"
#include "hogDetectorTest.hpp"
#include "surfDetectorTest.hpp"
#include "projectLoader.hpp"

using namespace std;

int main(int argc, char **argv)
{
  cout << "ColorHistDetector..." << endl;
  ColorHistDetectorTest chdtest;
  chdtest.Run(argc, argv);
 
  cout << "HoGDetector..." << endl;
  HoGDetectorTest htest;
  htest.Run(argc, argv);

  cout << "SurfDetector..." << endl;
  SURFDetectorTest stest;
  stest.Run(argc, argv);

  return 0;
}

