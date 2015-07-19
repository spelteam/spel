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

    ///constructors
    ImageSimilarityMatrix(void);
    ImageSimilarityMatrix(const ImageSimilarityMatrix& m);
    ImageSimilarityMatrix(const vector<Frame*>& frames);
    
    ///destructor
    ~ImageSimilarityMatrix(void);

    void buildImageSimilarityMatrix(const vector<Frame*>& frames, int maxFrameHeight=0);
    void buildMaskSimilarityMatrix(const vector<Frame*>& frames, int maxFrameHeight=0);

    bool read(string filename);
    bool write(string filename) const;

    float min() const;
    float mean() const;
    float max() const;
    float stddev() const;

    float at(int row, int col) const;
    Point2f getShift(int row, int col) const;
    ///get cost for path through ISM
    float getPathCost(vector<int> path) const; 

    uint32_t size() const;

    bool operator==(const ImageSimilarityMatrix &s) const;
    bool operator!=(const ImageSimilarityMatrix &s) const;
    ImageSimilarityMatrix & operator=(const ImageSimilarityMatrix &s);

    Mat clone(); //return a Mat clone of ISM

    // template <class T> inline std::string to_string (const T& t) 
    // {
    //     std::stringstream ss;
    //     ss << t;
    //     return ss.str();
    // }


  private:

    void computeMSMcell(Frame* left, Frame* right, int maxFrameHeight);
    void computeISMcell(Frame* left, Frame* right, int maxFrameHeight);
    ///the image similarity matrix
    Mat imageSimilarityMatrix;
    Mat imageShiftMatrix;
    
};

#endif  // _IMAGESIMILARITYMATRIX_HPP_

