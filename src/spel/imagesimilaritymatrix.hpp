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

    ///constructor
    ImageSimilarityMatrix(void) noexcept;
    ///destructor
    virtual ~ImageSimilarityMatrix(void) noexcept;

    virtual uint32_t getID(void);
    virtual void setID(uint32_t _id);

    virtual uint32_t size() const noexcept = 0;

    virtual void buildImageSimilarityMatrix(const std::vector<Frame*>& frames, const int maxFrameHeight = 0) = 0;
    virtual void buildMaskSimilarityMatrix(const std::vector<Frame*>& frames, const int maxFrameHeight = 0) = 0;

    virtual bool read(const std::string &filename) noexcept = 0;
    virtual bool write(const std::string &filename) const noexcept = 0;

    virtual float min() const noexcept = 0;
    virtual float mean() const = 0;
    virtual float max() const noexcept = 0;
    virtual float stddev() const = 0;

    virtual float at(const int row, const int col) const = 0;
    virtual cv::Point2f getShift(const int row, const int col) const = 0;
    ///get cost for path through ISM
    virtual float getPathCost(const std::vector<int> &path) const = 0;

  protected:
#ifdef DEBUG
    FRIEND_TEST(ImageSimilarityMatrixTests_, computeISMcell);
    FRIEND_TEST(ImageSimilarityMatrixTests_, computeMSMcell);
#endif  // DEBUG
    uint32_t id;

    virtual void computeMSMcell(const Frame* left, const Frame* right, const int maxFrameHeight) = 0;
    virtual void computeISMcell(const Frame* left, const Frame* right, const int maxFrameHeight) = 0;
  };
}
#endif  // _IMAGESIMILARITYMATRIX_HPP_
