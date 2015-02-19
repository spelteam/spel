#ifndef _MINSPANNINGTREE_HPP_
#define _MINSPANNINGTREE_HPP_

#include <vector>
#include "frame.hpp"
#include "imagesimilaritymatrix.hpp"
#include <opencv2/opencv.hpp>
#include <tree_util.hh>

using namespace std;
using namespace cv;
using namespace kptree;

class MinSpanningTree
{
  public:
    MinSpanningTree(void);
    MinSpanningTree(const ImageSimilarityMatrix& ism, int rootNode, int treeSize, float threshold);
    MinSpanningTree(const MinSpanningTree& mst);
    ~MinSpanningTree(void);

    void build(const ImageSimilarityMatrix& ism, int rootNode, int treeSize, float threshold); //build the MST

    MinSpanningTree& operator=(const MinSpanningTree& _MST);

    tree<int> getMST(void) const;

    //get size
    uint32_t size(void) const;

  private:
    tree<int> mst;
};

#endif  // _MINSPANNINGTREE_HPP_

