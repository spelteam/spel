#ifndef _IMAGEPIXELSIMILARITYMATRIX_HPP_
#define _IMAGEPIXELSIMILARITYMATRIX_HPP_

// SPEL definitions
#include "predef.hpp"

#include "imagesimilaritymatrix.hpp"

namespace SPEL
{
  class ImagePixelSimilarityMatrix : public ImageSimilarityMatrix
  {
  public:
    ///constructors
    ImagePixelSimilarityMatrix(void) noexcept;
    ImagePixelSimilarityMatrix(const ImagePixelSimilarityMatrix &m) noexcept;
    ImagePixelSimilarityMatrix(const std::vector<Frame*> &frames) noexcept;
    ImagePixelSimilarityMatrix(ImagePixelSimilarityMatrix &&m) noexcept;
    ///destructor
    virtual ~ImagePixelSimilarityMatrix(void) noexcept;
    ///operators
    virtual bool operator==(const ImagePixelSimilarityMatrix &s) const noexcept;
    virtual bool operator!=(const ImagePixelSimilarityMatrix &s) const noexcept;
    virtual ImageSimilarityMatrix & operator=(const ImagePixelSimilarityMatrix &s) noexcept;
    virtual ImageSimilarityMatrix & operator=(ImagePixelSimilarityMatrix &&s) noexcept;

    virtual void buildImageSimilarityMatrix(const std::vector<Frame*>& frames, const int maxFrameHeight = 0);
    virtual void buildMaskSimilarityMatrix(const std::vector<Frame*>& frames, const int maxFrameHeight = 0);

    uint32_t size() const noexcept;

    virtual bool read(const std::string &filename) noexcept;
    virtual bool write(const std::string &filename) const noexcept;

    virtual float min() const noexcept;
    virtual float mean() const;
    virtual float max() const noexcept;
    virtual float stddev() const;

    virtual float at(const int row, const int col) const;
    virtual cv::Point2f getShift(const int row, const int col) const;

    virtual float getPathCost(const std::vector<int> &path) const;

    virtual cv::Mat clone() const noexcept; //return a Mat clone of ISM
  protected:
    ///the image similarity matrix
    cv::Mat imageSimilarityMatrix;
    cv::Mat imageShiftMatrix;

    virtual void computeMSMcell(const Frame* left, const Frame* right, const int maxFrameHeight);
    virtual void computeISMcell(const Frame* left, const Frame* right, const int maxFrameHeight);

    virtual cv::Point2f calculateDistance(const cv::Mat &imgMatOne, const cv::Mat &imgMatTwo) const noexcept;
  };
}

#endif  // _IMAGEPIXELSIMILARITYMATRIX_HPP_
