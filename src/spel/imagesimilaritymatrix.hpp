#ifndef _IMAGESIMILARITYMATRIX_HPP_
#define _IMAGESIMILARITYMATRIX_HPP_

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

// SPEL definitions
#include "predef.hpp"

// STL
#include <vector>
#include <string>
#include <fstream>
#include <future>

// OpenCV
#include <opencv2/opencv.hpp>

// tree.hh
#include <tree_util.hh>

#include "frame.hpp"

namespace SPEL
{
  class ImageSimilarityMatrix
  {
  public:

    ///constructors
    ImageSimilarityMatrix(void) ;
    ImageSimilarityMatrix(const ImageSimilarityMatrix &m) ;
    ImageSimilarityMatrix(ImageSimilarityMatrix &&m) ;
    ///destructor
    virtual ~ImageSimilarityMatrix(void) ;
    ///operators
    bool operator==(const ImageSimilarityMatrix &s) const ;
    bool operator!=(const ImageSimilarityMatrix &s) const ;
    ImageSimilarityMatrix & operator=(const ImageSimilarityMatrix &s) ;
    ImageSimilarityMatrix & operator=(ImageSimilarityMatrix &&s) ;

    uint32_t getID(void);
    void setID(uint32_t _id);

    uint32_t size() const ;

    void buildImageSimilarityMatrix(const std::vector<Frame*>& frames, const int maxFrameHeight = 0);

    // Alternative buildMSM function
    // Test version
    // "Erode" and "Dilate" - erode or dilate iteration count (= 0 .. 3, 0 = don't used), - narrowing and/or broadening the mask. 
    // "UseRGBScore" - must be "false" for build MSM, must be "true" for build ISM
    // "inverseScore" = "false": value Score = 1  correspond to 100% masks overlap, "inverseScore" = "true": value Score = 0  correspond to 100% masks overlap.
    //  Diagonal elements always be set to bad score
    void buildImageSimilarityMatrix(const std::vector<Frame*>& frames,  int Erode, int Dilate, bool UseRGBScore = false, bool inverseScore = true);
    void buildMSM_OnMaskArea(const std::vector<Frame*>& frames, bool UseRGBScore, bool inverseScore);
    //- alternative MSM

    bool read(const std::string &filename) ;
    bool write(const std::string &filename) const ;

    float min() const ;
    float mean() const;
    float max() const ;
    float stddev() const;

    float at(const int row, const int col) const;
    cv::Point2f getShift(const int row, const int col) const;
    ///get cost for path through ISM
    float getPathCost(const std::vector<int> &path) const;

    cv::Mat clone() const ; //return a Mat clone of ISM
  protected:
    uint32_t id;
    ///the image similarity matrix
    cv::Mat imageSimilarityMatrix;
    cv::Mat imageShiftMatrix;

    cv::Point2f calculateDistance(const cv::Mat &imgMatOne, const cv::Mat &imgMatTwo) const ;
    virtual void computeISMcell(Frame* left, Frame* right, const int maxFrameHeight) = 0;
  private:
#ifdef DEBUG
    FRIEND_TEST(ImageSimilarityMatrixTests_, computeISMcell);
    FRIEND_TEST(ImageSimilarityMatrixTests_, computeMSMcell);
    FRIEND_TEST(ImageSimilarityMatrixTests_, calculateDistance);
    FRIEND_TEST(MaskSimilarityMatrixTests, DefaultConstructor);
    FRIEND_TEST(MaskSimilarityMatrixTests, CopyConstructor);
    FRIEND_TEST(MaskSimilarityMatrixTests, MoveConstructor);
    FRIEND_TEST(ImageSimilarityMatrixTests_, CalculateConstructor);
    FRIEND_TEST(ImageHogSimilarityMatrix, frames_ISM);
#endif  // DEBUG    
  };
}
#endif  // _IMAGESIMILARITYMATRIX_HPP_
