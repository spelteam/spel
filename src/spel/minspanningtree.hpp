#ifndef _MINSPANNINGTREE_HPP_
#define _MINSPANNINGTREE_HPP_

// SPEL definitions
#include "predef.hpp"

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

// STL
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// tree.hh
#include <tree_util.hh>

#include "frame.hpp"
#include "imagesimilaritymatrix.hpp"

namespace SPEL
{
  class MinSpanningTree
  {
  public:
    MinSpanningTree(void);
    MinSpanningTree(const ImageSimilarityMatrix& ism, int rootNode, int treeSize, float threshold);
    MinSpanningTree(const MinSpanningTree& mst);
    virtual ~MinSpanningTree(void);
    ///build the MST
    virtual void build(const ImageSimilarityMatrix& ism, int rootNode, int treeSize, float threshold);

    virtual MinSpanningTree& operator=(const MinSpanningTree& _MST);

    virtual tree<int> getMST(void) const;

    ///get size
    virtual uint32_t size(void) const;

  private:
#ifdef DEBUG
    FRIEND_TEST(MinSpanningTree, CopyConstructor);
    FRIEND_TEST(MinSpanningTree, getMST);
    FRIEND_TEST(MinSpanningTree, AssigmentOperator);
#endif  // DEBUG
    tree<int> mst;
  };

}

#endif  // _MINSPANNINGTREE_HPP_
