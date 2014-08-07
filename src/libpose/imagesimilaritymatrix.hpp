#ifndef _IMAGESIMILARITYMATRIX_HPP_
#define _IMAGESIMILARITYMATRIX_HPP_

#include <vector>
#include "frame.hpp"
#include <opencv2/opencv.hpp>
#include <tree_util.hh>

using namespace std;
using namespace cv;

class ImageSimilarityMatrix
{
  public:
    //constructors
    ImageSimilarityMatrix();
    ImageSimilarityMatrix(const ImageSimilarityMatrix& m);
    ImageSimilarityMatrix(const vector<Frame*>& frames);
    
    //destructor
    ~ImageSimilarityMatrix();

    void buildImageSimilarityMatrix(const vector<Frame*>& frames);
    void buildMaskSimilarityMatrix(const vector<Frame*>& frames);

    bool read(string filename);
    bool write(string filename);

    float min();
    float mean();
    float max();

    float at(int row, int col);

    float getPathCost(vector<int> path); //get cost for path through ISM

    int size();

    bool operator==(const ImageSimilarityMatrix &s);
    bool operator!=(const ImageSimilarityMatrix &s);
    ImageSimilarityMatrix & operator=(const ImageSimilarityMatrix &s);

  private:
    Mat imageSimilarityMatrix; //the image similarity matrix
    
};

#endif  // _IMAGESIMILARITYMATRIX_HPP_

