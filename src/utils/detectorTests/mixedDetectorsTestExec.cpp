#include <iostream>
#include "colorHistDetectorTest.hpp"
#include "hogDetectorTest.hpp"
#include "surfDetectorTest.hpp"
#include "projectLoader.hpp"

using namespace std;

int main(int argc, char **argv)
{
  map <uint32_t, vector <vector <LimbLabel>>> limbLabels;

  cout << "ColorHistDetector..." << endl;
  ColorHistDetectorTest chdtest;
  chdtest.Run(argc, argv, &limbLabels);
 
  cout << "HoGDetector..." << endl;
  HoGDetectorTest htest;
  htest.Run(argc, argv, &limbLabels);

  cout << "SurfDetector..." << endl;
  SURFDetectorTest stest;
  stest.Run(argc, argv, &limbLabels);

  return 0;
}

