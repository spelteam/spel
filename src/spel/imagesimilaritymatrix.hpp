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
    ImageSimilarityMatrix(void) noexcept;
    ImageSimilarityMatrix(const ImageSimilarityMatrix& m) noexcept;
    ImageSimilarityMatrix(const std::vector<Frame*>& frames);

    ///destructor
    virtual ~ImageSimilarityMatrix(void) noexcept;

    virtual void buildImageSimilarityMatrix(const std::vector<Frame*>& frames, int maxFrameHeight = 0);
    virtual void buildMaskSimilarityMatrix(const std::vector<Frame*>& frames, int maxFrameHeight = 0);

    virtual bool read(std::string filename);
    virtual bool write(std::string filename) const;

    virtual float min() const;
    virtual float mean() const;
    virtual float max() const;
    virtual float stddev() const;

    virtual float at(int row, int col) const;
    virtual cv::Point2f getShift(int row, int col) const;
    ///get cost for path through ISM
    virtual float getPathCost(std::vector<int> path) const;

    virtual uint32_t size() const;

    virtual bool operator==(const ImageSimilarityMatrix &s) const;
    virtual bool operator!=(const ImageSimilarityMatrix &s) const;
    virtual ImageSimilarityMatrix & operator=(const ImageSimilarityMatrix &s);

    virtual cv::Mat clone(); //return a Mat clone of ISM

  protected:
#ifdef DEBUG
    FRIEND_TEST(ImageSimilarityMatrixTests_, computeISMcell);
    FRIEND_TEST(ImageSimilarityMatrixTests_, computeMSMcell);
#endif  // DEBUG

    virtual void computeMSMcell(Frame* left, Frame* right, int maxFrameHeight);
    virtual void computeISMcell(Frame* left, Frame* right, int maxFrameHeight);
    ///the image similarity matrix
    cv::Mat imageSimilarityMatrix;
    cv::Mat imageShiftMatrix;

  };
}
#endif  // _IMAGESIMILARITYMATRIX_HPP_
