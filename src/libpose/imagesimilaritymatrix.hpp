#ifndef _IMAGESIMILARITYMATRIX_HPP_
#define _IMAGESIMILARITYMATRIX_HPP_

#include <vector>
#include "frame.hpp"
#include <opencv2/opencv.hpp>
#include <tree_util.hh>
#include <string>

using namespace std;
using namespace cv;

class ImageSimilarityMatrix
{
  public:

    //constructors
    ImageSimilarityMatrix(void);
    ImageSimilarityMatrix(const ImageSimilarityMatrix& m);
    ImageSimilarityMatrix(const vector<Frame*>& frames);
    
    //destructor
    ~ImageSimilarityMatrix(void);

    void buildImageSimilarityMatrix(const vector<Frame*>& frames);
    void buildMaskSimilarityMatrix(const vector<Frame*>& frames);

    bool read(string filename);
    bool write(string filename) const;

    float min() const;
    float mean() const;
    float max() const;

    float at(int row, int col) const;

    float getPathCost(vector<int> path) const; //get cost for path through ISM

    uint size() const;

    bool operator==(const ImageSimilarityMatrix &s) const;
    bool operator!=(const ImageSimilarityMatrix &s) const;
    ImageSimilarityMatrix & operator=(const ImageSimilarityMatrix &s);

    // template <class T> inline std::string to_string (const T& t) 
    // {
    //     std::stringstream ss;
    //     ss << t;
    //     return ss.str();
    // }


  private:
    Mat imageSimilarityMatrix; //the image similarity matrix
    
};

#endif  // _IMAGESIMILARITYMATRIX_HPP_

