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
    ImageSimilarityMatrix(const ImageSimilarityMatrix &m) noexcept;
    ImageSimilarityMatrix(ImageSimilarityMatrix &&m) noexcept;
    ///destructor
    virtual ~ImageSimilarityMatrix(void) noexcept;
    ///operators
    virtual bool operator==(const ImageSimilarityMatrix &s) const noexcept;
    virtual bool operator!=(const ImageSimilarityMatrix &s) const noexcept;
    virtual ImageSimilarityMatrix & operator=(const ImageSimilarityMatrix &s) noexcept;
    virtual ImageSimilarityMatrix & operator=(ImageSimilarityMatrix &&s) noexcept;

    virtual uint32_t getID(void);
    virtual void setID(uint32_t _id);

    virtual uint32_t size() const noexcept;

    virtual void buildImageSimilarityMatrix(const std::vector<Frame*>& frames, const int maxFrameHeight = 0);

    virtual bool read(const std::string &filename) noexcept;
    virtual bool write(const std::string &filename) const noexcept;

    virtual float min() const noexcept;
    virtual float mean() const;
    virtual float max() const noexcept;
    virtual float stddev() const;

    virtual float at(const int row, const int col) const;
    virtual cv::Point2f getShift(const int row, const int col) const;
    ///get cost for path through ISM
    virtual float getPathCost(const std::vector<int> &path) const;
    virtual cv::Point2f calculateDistance(const cv::Mat &imgMatOne, const cv::Mat &imgMatTwo) const noexcept;

    virtual cv::Mat clone() const noexcept; //return a Mat clone of ISM
  protected:
#ifdef DEBUG
    FRIEND_TEST(ImageSimilarityMatrixTests_, computeISMcell);
    FRIEND_TEST(ImageSimilarityMatrixTests_, computeMSMcell);
#endif  // DEBUG
    uint32_t id;

    ///the image similarity matrix
    cv::Mat imageSimilarityMatrix;
    cv::Mat imageShiftMatrix;

    virtual void computeISMcell(const Frame* left, const Frame* right, const int maxFrameHeight) = 0;
  };
}
#endif  // _IMAGESIMILARITYMATRIX_HPP_
